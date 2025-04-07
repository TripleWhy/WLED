#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
/ Fireworks in starburst effect
/ based on the video: https://www.reddit.com/r/arduino/comments/c3sd46/i_made_this_fireworks_effect_for_my_led_strips/
/ Speed sets frequency of new starbursts, intensity is the intensity of the burst
*/
#ifdef ESP8266
  #define STARBURST_MAX_FRAG   8 //52 bytes / star
#else
  #define STARBURST_MAX_FRAG  10 //60 bytes / star
#endif

class StarburstEffect : public BaseEffect<StarburstEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    //each needs 20+STARBURST_MAX_FRAG*4 bytes
    struct Star {
        CRGB     color;
        uint32_t birth  =0;
        uint32_t last   =0;
        float    vel    =0;
        uint16_t pos    =-1;
        float    fragment[STARBURST_MAX_FRAG];
    };

private:
    using Self = StarburstEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char* const metaData = "Fireworks Starburst@Chance,Fragments,,,,,Overlay;,!;!;;pal=11,m12=0";
    static constexpr const uint8_t effectId = FX_MODE_STARBURST;

    explicit StarburstEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        unsigned maxData = FAIR_DATA_PER_SEG; //ESP8266: 256 ESP32: 640
        {
            unsigned segs = strip.getActiveSegmentsNum();
            if (segs <= (strip.getMaxSegments() /2))
                maxData *= 2; //ESP8266: 512 if <= 8 segs ESP32: 1280 if <= 16 segs
            if (segs <= (strip.getMaxSegments() /4))
                maxData *= 2; //ESP8266: 1024 if <= 4 segs ESP32: 2560 if <= 8 segs
        }
        unsigned maxStars = maxData / sizeof(Star); //ESP8266: max. 4/9/19 stars/seg, ESP32: max. 10/21/42 stars/seg

        unsigned numStars = 1 + (coordinate.width >> 3);
        if (numStars > maxStars)
            numStars = maxStars;

        stars.resize(numStars);
        if (stars.size() != numStars) {
            stars.clear();
            return;
        }
        stars.shrink_to_fit();

        uint32_t it = strip.now;

        float          maxSpeed                = 375.0f;  // Max velocity
        float          particleIgnition        = 250.0f;  // How long to "flash"
        float          particleFadeTime        = 1500.0f; // Fade out time

        for (unsigned j = 0; j < numStars; j++)
        {
            // speed to adjust chance of a burst, max is nearly always.
            if (hw_random8((144-(SEGMENT.speed >> 1))) == 0 && stars[j].birth == 0)
            {
                // Pick a random color and location.
                unsigned startPos = hw_random16(coordinate.width-1);
                float multiplier = (float)(hw_random8())/255.0f * 1.0f;

                stars[j].color = CRGB(SEGMENT.color_wheel(hw_random8()));
                stars[j].pos = startPos;
                stars[j].vel = maxSpeed * (float)(hw_random8())/255.0f * multiplier;
                stars[j].birth = it;
                stars[j].last = it;
                // more fragments means larger burst effect
                int num = hw_random8(3,6 + (SEGMENT.intensity >> 5));

                for (int i=0; i < STARBURST_MAX_FRAG; i++) {
                    if (i < num)
                        stars[j].fragment[i] = startPos;
                    else
                        stars[j].fragment[i] = -1;
                }
            }
        }

        if (!SEGMENT.check2)
            buffer.fill(SEGCOLOR(1));

        for (unsigned j=0; j<numStars; j++)
        {
            if (stars[j].birth != 0) {
                float dt = (it-stars[j].last)/1000.0;

                for (int i=0; i < STARBURST_MAX_FRAG; i++) {
                    int var = i >> 1;

                    if (stars[j].fragment[i] > 0) {
                        //all fragments travel right, will be mirrored on other side
                        stars[j].fragment[i] += stars[j].vel * dt * (float)var/3.0;
                    }
                }
                stars[j].last = it;
                stars[j].vel -= 3*stars[j].vel*dt;
            }

            CRGB c = stars[j].color;

            // If the star is brand new, it flashes white briefly.
            // Otherwise it just fades over time.
            float fade = 0.0f;
            float age = it-stars[j].birth;

            if (age < particleIgnition) {
                c = CRGB(color_blend(WHITE, RGBW32(c.r,c.g,c.b,0), uint8_t(254.5f*((age / particleIgnition)))));
            } else {
                // Figure out how much to fade and shrink the star based on
                // its age relative to its lifetime
                if (age > particleIgnition + particleFadeTime) {
                    fade = 1.0f;                  // Black hole, all faded out
                    stars[j].birth = 0;
                    c = CRGB(SEGCOLOR(1));
                } else {
                    age -= particleIgnition;
                    fade = (age / particleFadeTime);  // Fading star
                    c = CRGB(color_blend(RGBW32(c.r,c.g,c.b,0), SEGCOLOR(1), uint8_t(254.5f*fade)));
                }
            }

            float particleSize = (1.0f - fade) * 2.0f;

            for (size_t index=0; index < STARBURST_MAX_FRAG*2; index++) {
                bool mirrored = index & 0x1;
                unsigned i = index >> 1;
                if (stars[j].fragment[i] > 0) {
                    float loc = stars[j].fragment[i];
                    if (mirrored)
                        loc -= (loc-stars[j].pos)*2;
                    unsigned start = loc - particleSize;
                    unsigned end = loc + particleSize;
                    if (start < 0)
                        start = 0;
                    if (start == end)
                        end++;
                    if (end > coordinate.width)
                        end = coordinate.width;
                    for (unsigned p = start; p < end; p++) {
                        buffer.setPixelColor(p, RGBW32(c.r, c.g, c.b, 0));
                    }
                }
            }
        }
    }

private:
    SegmentAllocator<Star>::vector stars{};
};

