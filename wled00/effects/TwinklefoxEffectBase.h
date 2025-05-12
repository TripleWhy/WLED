#pragma once

#include "../FX.h"
#include "Effect.h"

//  This function loops over each pixel, calculates the
//  adjusted 'clock' that this pixel should use, and calls
//  "CalculateOneTwinkle" on each pixel.  It then displays
//  either the twinkle color of the background color,
//  whichever is brighter.
class TwinklefoxEffectBase : public Effect {
private:
    using Self = TwinklefoxEffectBase;
    using Base = Effect;

public:
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d1;

    explicit constexpr TwinklefoxEffectBase(const EffectInformation& ei, bool cat)
        : Base{ei},
          cat{cat}
    {
    }

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        // "PRNG16" is the pseudorandom number generator
        // It MUST be reset to the same starting value each time
        // this function is called, so that the sequence of 'random'
        // numbers that it generates is (paradoxically) stable.
        PRNG16 = 11337;

        // Calculate speed
        if (parameters.speed > 100) aux0 = 3 + ((255 - parameters.speed) >> 3);
        else aux0 = 22 + ((100 - parameters.speed) >> 1);

        // Set up the background color, "bg".
        bg = SEGCOLOR(1);
        unsigned bglight = bg.getAverageLight();
        if (bglight > 64) {
            bg = color_fade(bg, 16, true); // very bright, so scale to 1/16th
        } else if (bglight > 16) {
            bg = color_fade(bg, 64, true); // not that bright, so scale to 1/4th
        } else {
            bg = color_fade(bg, 86, true); // dim, scale to 1/3rd.
        }

        backgroundBrightness = bg.getAverageLight();
        return true;
    }


    uint32_t getPixelColorImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        PRNG16 = (uint16_t)(PRNG16 * 2053) + 1384; // next 'random' number
        unsigned myclockoffset16= PRNG16; // use that number as clock offset
        PRNG16 = (uint16_t)(PRNG16 * 2053) + 1384; // next 'random' number
        // use that number as clock speed adjustment factor (in 8ths, from 8/8ths to 23/8ths)
        unsigned myspeedmultiplierQ5_3 =  ((((PRNG16 & 0xFF)>>4) + (PRNG16 & 0x0F)) & 0x0F) + 0x08;
        uint32_t myclock30 = (uint32_t)((strip.now * myspeedmultiplierQ5_3) >> 3) + myclockoffset16;
        unsigned  myunique8 = PRNG16 >> 8; // get 'salt' value for this pixel

        // We now have the adjusted 'clock' for this pixel, now we call
        // the function that computes what color the pixel should be based
        // on the "brightness = f( time )" idea.
        CRGBW c = twinklefox_one_twinkle(parameters, myclock30, myunique8, cat);

        unsigned cbright = c.getAverageLight();
        int deltabright = cbright - backgroundBrightness;
        if (deltabright >= 32 || (bg==0)) {
            // If the new pixel is significantly brighter than the background color,
            // use the new color.
            return RGBW32(c.r, c.g, c.b, 0);
        } else if (deltabright > 0) {
            // If the new pixel is just slightly brighter than the background color,
            // mix a blend of the new color and the background color
            return color_blend(bg, c, uint8_t(deltabright * 8));
        } else {
            // if the new pixel is not at all brighter than the background color,
            // just use the background color.
            return RGBW32(bg.r, bg.g, bg.b, 0);
        }
    }

private:
    //  TwinkleFOX by Mark Kriegsman: https://gist.github.com/kriegsman/756ea6dcae8e30845b5a
    //
    //  TwinkleFOX: Twinkling 'holiday' lights that fade in and out.
    //  Colors are chosen from a palette. Read more about this effect using the link above!
    CRGBW twinklefox_one_twinkle(TransitionableParameters& parameters, uint32_t ms, uint8_t salt, bool cat)
    {
        // Overall twinkle speed (changed)
        unsigned ticks = ms / aux0;
        unsigned fastcycle8 = uint8_t(ticks);
        uint16_t slowcycle16 = (ticks >> 8) + salt;
        slowcycle16 += sin8_t(slowcycle16);
        slowcycle16 = (slowcycle16 * 2053) + 1384;
        uint8_t slowcycle8 = (slowcycle16 & 0xFF) + (slowcycle16 >> 8);

        // Overall twinkle density.
        // 0 (NONE lit) to 8 (ALL lit at once).
        // Default is 5.
        unsigned twinkleDensity = (parameters.intensity >> 5) +1;

        unsigned bright = 0;
        if (((slowcycle8 & 0x0E)/2) < twinkleDensity) {
            unsigned ph = fastcycle8;
            // This is like 'triwave8', which produces a
            // symmetrical up-and-down triangle sawtooth waveform, except that this
            // function produces a triangle wave with a faster attack and a slower decay
            if (cat) //twinklecat, variant where the leds instantly turn on
            {
                bright = 255 - ph;
            } else { //vanilla twinklefox
                if (ph < 86) {
                bright = ph * 3;
                } else {
                    ph -= 86;
                    bright = 255 - (ph + (ph/2));
                }
            }
        }

        unsigned hue = slowcycle8 - salt;
        CRGBW c;
        if (bright > 0) {
            c = SEGPALETTE.ColorFromPalette(hue, bright, NOBLEND);
            if (!parameters.check1) {
                // This code takes a pixel, and if its in the 'fading down'
                // part of the cycle, it adjusts the color a little bit like the
                // way that incandescent bulbs fade toward 'red' as they dim.
                if (fastcycle8 >= 128)
                {
                    unsigned cooling = (fastcycle8 - 128) >> 4;
                    c.g = qsub8(c.g, cooling);
                    c.b = qsub8(c.b, cooling * 2);
                }
            }
        } else {
            c = 0; // black
        }
        return c;
    }

private:
    const bool cat;

    uint16_t PRNG16{};
    CRGBW bg{};
    unsigned backgroundBrightness{};
    uint16_t aux0{};
};
