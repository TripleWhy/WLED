#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
  Aurora effect
*/

//CONFIG
#ifdef ESP8266
  #define W_MAX_COUNT  9          //Number of simultaneous waves
#else
  #define W_MAX_COUNT 20          //Number of simultaneous waves
#endif
#define W_MAX_SPEED 6             //Higher number, higher speed
#define W_WIDTH_FACTOR 6          //Higher number, smaller waves

class AuroraEffect : public BaseEffect<AuroraEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    //24 bytes
    class AuroraWave {
    private:
        uint16_t ttl;
        CRGB basecolor;
        float basealpha;
        uint16_t age;
        uint16_t width;
        float center;
        bool goingleft;
        float speed_factor;
        bool alive = true;

    public:
        void init(uint32_t segment_length, CRGB color) {
            ttl = hw_random16(500, 1501);
            basecolor = color;
            basealpha = hw_random8(60, 101) / (float)100;
            age = 0;
            width = hw_random16(segment_length / 20, segment_length / W_WIDTH_FACTOR); //half of width to make math easier
            if (!width)
                width = 1;
            center = hw_random8(101) / (float)100 * segment_length;
            goingleft = hw_random8(0, 2) == 0;
            speed_factor = (hw_random8(10, 31) / (float)100 * W_MAX_SPEED / 255);
            alive = true;
        }

        CRGB getColorForLED(int ledIndex) {
          if(ledIndex < center - width || ledIndex > center + width)
              return 0; //Position out of range of this wave

          CRGB rgb;

          //Offset of this led from center of wave
          //The further away from the center, the dimmer the LED
          float offset = ledIndex - center;
          if (offset < 0)
              offset = -offset;
          float offsetFactor = offset / width;

          //The age of the wave determines it brightness.
          //At half its maximum age it will be the brightest.
          float ageFactor = 0.1;
          if((float)age / ttl < 0.5) {
              ageFactor = (float)age / (ttl / 2);
          } else {
              ageFactor = (float)(ttl - age) / ((float)ttl * 0.5);
          }

          //Calculate color based on above factors and basealpha value
          float factor = (1 - offsetFactor) * ageFactor * basealpha;
          rgb.r = basecolor.r * factor;
          rgb.g = basecolor.g * factor;
          rgb.b = basecolor.b * factor;

          return rgb;
        };

        //Change position and age of wave
        //Determine if its sill "alive"
        void update(uint32_t segment_length, uint32_t speed) {
            if(goingleft) {
                center -= speed_factor * speed;
            } else {
                center += speed_factor * speed;
            }

          age++;

          if(age > ttl) {
              alive = false;
          } else {
            if(goingleft) {
                if(center + width < 0) {
                    alive = false;
                }
            } else {
                if(center - width > segment_length) {
                    alive = false;
                }
            }
          }
        };

        bool stillAlive() {
            return alive;
        };
    };

private:
    using Self = AuroraEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Aurora@!,!;1,2,3;!;;sx=24,pal=50";
    static constexpr const uint8_t effectId = FX_MODE_AURORA;

    explicit AuroraEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        //Intensity slider changed or first call
        if(previousIntensity != SEGMENT.intensity || SEGENV.call == 0) {
            wavecount = map(SEGMENT.intensity, 0, 255, 2, W_MAX_COUNT);
            previousIntensity = SEGMENT.intensity;

            waves.resize(wavecount);
            if (waves.size() != wavecount) {
                waves.clear();
                return;
            }
            waves.shrink_to_fit();

            for (int i = 0; i < wavecount; i++) {
                waves[i].init(coordinate.width, CRGB(SEGMENT.color_from_palette(hw_random8(), false, false, hw_random8(0, 3))));
            }
        }

        for (int i = 0; i < wavecount; i++) {
            //Update values of wave
            waves[i].update(coordinate.width, SEGMENT.speed);

            if(!(waves[i].stillAlive())) {
                //If a wave dies, reinitialize it starts over.
                waves[i].init(coordinate.width, CRGB(SEGMENT.color_from_palette(hw_random8(), false, false, hw_random8(0, 3))));
            }
        }

        uint8_t backlight = 1; //dimmer backlight if less active colors
        if (SEGCOLOR(0)) backlight++;
        if (SEGCOLOR(1)) backlight++;
        if (SEGCOLOR(2)) backlight++;
        //Loop through LEDs to determine color
        for (unsigned i = 0; i < coordinate.width; i++) {
            CRGB mixedRgb = CRGB(backlight, backlight, backlight);

            //For each LED we must check each wave if it is "active" at this position.
            //If there are multiple waves active on a LED we multiply their values.
            for (int  j = 0; j < wavecount; j++) {
                CRGB rgb = waves[j].getColorForLED(i);

                if(rgb != CRGB(0)) {
                    mixedRgb += rgb;
                }
            }

            buffer.setPixelColor(i, RGBW32(mixedRgb.r, mixedRgb.g, mixedRgb.b,0));
        }
    }

private:
    uint16_t previousIntensity{};
    uint16_t wavecount{};
    SegmentAllocator<AuroraWave>::vector waves{};
};


