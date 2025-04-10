#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
  TV Simulator
  Modified and adapted to WLED by Def3nder, based on "Fake TV Light for Engineers" by Phillip Burgess https://learn.adafruit.com/fake-tv-light-for-engineers/arduino-sketch
*/

class TvSimulatorEffect : public BaseEffect<TvSimulatorEffect> {
private:
    using Self = TvSimulatorEffect;
    using Base = BaseEffect<Self>;

public:
    static constexpr const char metaData[] PROGMEM = "TV Simulator@!,!;;!;01";
    static constexpr const uint8_t effectId = FX_MODE_TV_SIMULATOR;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d0;

    using Base::Base;

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        int nr, ng, nb, r, g, b, i, hue;
        uint8_t  sat, bri, j;

        uint8_t colorSpeed     = map(SEGMENT.speed,     0, UINT8_MAX,  1, 20);
        uint8_t colorIntensity = map(SEGMENT.intensity, 0, UINT8_MAX, 10, 30);

        i = SEGMENT.speed << 8 | SEGMENT.intensity;
        if (i != sliderValues) {
            sliderValues = i;
            aux1 = 0;
        }

            // create a new sceene
            if (((strip.now - sceeneStart) >= sceeneDuration) || aux1 == 0) {
                sceeneStart    = strip.now;                                               // remember the start of the new sceene
                sceeneDuration = hw_random16(60* 250* colorSpeed, 60* 750 * colorSpeed);    // duration of a "movie sceene" which has similar colors (5 to 15 minutes with max speed slider)
                sceeneColorHue = hw_random16(   0, 768);                                    // random start color-tone for the sceene
                sceeneColorSat = hw_random8 ( 100, 130 + colorIntensity);                   // random start color-saturation for the sceene
                sceeneColorBri = hw_random8 ( 200, 240);                                    // random start color-brightness for the sceene
                aux1 = 1;
                aux0 = 0;
            }

            // slightly change the color-tone in this sceene
            if (aux0 == 0) {
                // hue change in both directions
                j = hw_random8(4 * colorIntensity);
                hue = (hw_random8() < 128) ? ((j < sceeneColorHue)       ? sceeneColorHue - j : 767 - sceeneColorHue - j) :  // negative
                                             ((j + sceeneColorHue) < 767 ? sceeneColorHue + j : sceeneColorHue + j - 767) ;  // positive

                // saturation
                j = hw_random8(2 * colorIntensity);
                sat = (sceeneColorSat - j) < 0 ? 0 : sceeneColorSat - j;

                // brightness
                j = hw_random8(100);
                bri = (sceeneColorBri - j) < 0 ? 0 : sceeneColorBri - j;

                // calculate R,G,B from HSV
                // Source: https://blog.adafruit.com/2012/03/14/constant-brightness-hsb-to-rgb-algorithm/
                { // just to create a local scope for  the variables
                    uint8_t temp[5], n = (hue >> 8) % 3;
                    uint8_t x = ((((hue & 255) * sat) >> 8) * bri) >> 8;
                    uint8_t s = (  (256 - sat) * bri) >> 8;
                    temp[0] = temp[3] =       s;
                    temp[1] = temp[4] =   x + s;
                    temp[2] =           bri - x;
                    actualColorR = temp[n + 2];
                    actualColorG = temp[n + 1];
                    actualColorB = temp[n    ];
                }
            }
            // Apply gamma correction, further expand to 16/16/16
            nr = (uint8_t)gamma8(actualColorR) * 257; // New R/G/B
            ng = (uint8_t)gamma8(actualColorG) * 257;
            nb = (uint8_t)gamma8(actualColorB) * 257;

        if (aux0 == 0) {  // initialize next iteration
            aux0 = 1;

            // randomize total duration and fade duration for the actual color
            totalTime = hw_random16(250, 2500);                   // Semi-random pixel-to-pixel time
            fadeTime  = hw_random16(0, totalTime);   // Pixel-to-pixel transition time
            if (hw_random8(10) < 3) fadeTime = 0;                 // Force scene cut 30% of time

            startTime = strip.now;
        } // end of initialization

        // how much time is elapsed ?
        elapsed = strip.now - startTime;

        // fade from prev color to next color
        if (elapsed < fadeTime) {
            r = map(elapsed, 0, fadeTime, previousR, nr);
            g = map(elapsed, 0, fadeTime, previousG, ng);
            b = map(elapsed, 0, fadeTime, previousB, nb);
        } else { // Avoid divide-by-zero in map()
            r = nr;
            g = ng;
            b = nb;
        }
        // if total duration has passed, remember last color and restart the loop
        if (elapsed >= totalTime) {
            previousR = nr; // Prev RGB = new RGB
            previousG = ng;
            previousB = nb;
            aux0 = 0;
        }

        color = RGBW32(r >> 8, g >> 8, b >> 8, 0);  // Quantize to 8-bit
    }

    uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        return color;
    }

private:
    uint32_t totalTime = 0;
    uint32_t fadeTime  = 0;
    uint32_t startTime = 0;
    uint32_t elapsed   = 0;
    uint32_t pixelNum  = 0;
    uint16_t sliderValues = 0;
    uint32_t sceeneStart    = 0;
    uint32_t sceeneDuration = 0;
    uint16_t sceeneColorHue = 0;
    uint8_t  sceeneColorSat = 0;
    uint8_t  sceeneColorBri = 0;
    uint8_t  actualColorR = 0;
    uint8_t  actualColorG = 0;
    uint8_t  actualColorB = 0;
    uint16_t previousR = 0;
    uint16_t previousG = 0;
    uint16_t previousB = 0;

    uint16_t aux0{};
    uint16_t aux1{};

    uint32_t color{};
};


