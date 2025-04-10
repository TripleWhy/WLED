#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/////////////////////////
//     2D DNA Spiral   //
/////////////////////////
// By: ldirko  https://editor.soulmatelights.com/gallery/512-dna-spiral-variation , modified by: Andrew Tuline
class DnaSpiral2dEffect : public BaseEffect<DnaSpiral2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = DnaSpiral2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "DNA Spiral@Scroll speed,Y frequency,Blur,,,Smear;;!;2;c1=0";
    static constexpr const uint8_t effectId = FX_MODE_2DDNASPIRAL;

    explicit DnaSpiral2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        if (SEGENV.call == 0) {
            buffer.fill(BLACK);
        }

        unsigned speeds = SEGMENT.speed/2 + 7;
        unsigned freq = SEGMENT.intensity/8;

        uint32_t ms = strip.now / 20;
        buffer.fadeToBlackBy(135);

        for (int i = 0; i < rows; i++) {
            int x  = beatsin8_t(speeds, 0, cols - 1, 0, i * freq) + beatsin8_t(speeds - 7, 0, cols - 1, 0, i * freq + 128);
            int x1 = beatsin8_t(speeds, 0, cols - 1, 0, 128 + i * freq) + beatsin8_t(speeds - 7, 0, cols - 1, 0, 128 + 64 + i * freq);
            unsigned hue = (i * 128 / rows) + ms;
            // skip every 4th row every now and then (fade it more)
            if ((i + ms / 8) & 3) {
                // draw a gradient line between x and x1
                x = x / 2; x1 = x1 / 2;
                unsigned steps = abs8(x - x1) + 1;
                bool positive = (x1 >= x);                         // direction of drawing
                for (size_t k = 1; k <= steps; k++) {
                    unsigned rate = k * 255 / steps;
                    //unsigned dx = lerp8by8(x, x1, rate);
                    unsigned dx = positive? (x + k-1) : (x - k+1);   // behaves the same as "lerp8by8" but does not create holes
                    //buffer.setPixelColor(dx, i, ColorFromPalette(SEGPALETTE, hue, 255, LINEARBLEND).nscale8_video(rate));
                    buffer.addPixelColor(dx, i, ColorFromPalette(SEGPALETTE, hue, 255, LINEARBLEND)); // use setPixelColorXY for different look
                    buffer.fadePixelColor(dx, i, rate);
                }
                buffer.setPixelColor(x, i, DARKSLATEGRAY);
                buffer.setPixelColor(x1, i, WHITE);
            }
        }
        buffer.blur(((uint16_t)SEGMENT.custom1 * 3) / (6 + SEGMENT.check1), SEGMENT.check1);
    }

private:
};


#endif //WLED_DISABLE_2D
