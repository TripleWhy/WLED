#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

////////////////////////////
//     2D Colored Bursts  //
////////////////////////////
// By: ldirko   https://editor.soulmatelights.com/gallery/819-colored-bursts , modified by: Andrew Tuline
class ColoredBursts2dEffect : public BaseEffect<ColoredBursts2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = ColoredBursts2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "Colored Bursts@Speed,# of lines,,,Blur,Gradient,Smear,Dots;;!;2;c3=16";
    static constexpr const uint8_t effectId = FX_MODE_2DCOLOREDBURSTS;

    explicit ColoredBursts2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        if (SEGENV.call == 0) {
            aux0 = 0; // start with red hue
        }

        const bool dot  = SEGMENT.check3;
        const bool grad = SEGMENT.check1;

        byte numLines = SEGMENT.intensity/16 + 1;

        aux0++;  // hue
        buffer.fadeToBlackBy(40 - SEGMENT.check2 * 8);
        for (size_t i = 0; i < numLines; i++) {
            byte x1 = beatsin8_t(2 + SEGMENT.speed/16, 0, (cols - 1));
            byte x2 = beatsin8_t(1 + SEGMENT.speed/16, 0, (rows - 1));
            byte y1 = beatsin8_t(5 + SEGMENT.speed/16, 0, (cols - 1), 0, i * 24);
            byte y2 = beatsin8_t(3 + SEGMENT.speed/16, 0, (rows - 1), 0, i * 48 + 64);
            uint32_t color = ColorFromPalette(SEGPALETTE, i * 255 / numLines + (aux0&0xFF), 255, LINEARBLEND);

            byte xsteps = abs8(x1 - y1) + 1;
            byte ysteps = abs8(x2 - y2) + 1;
            byte steps = xsteps >= ysteps ? xsteps : ysteps;
            //Draw gradient line
            for (size_t j = 1; j <= steps; j++) {
                uint8_t rate = j * 255 / steps;
                byte dx = lerp8by8(x1, y1, rate);
                byte dy = lerp8by8(x2, y2, rate);
                //buffer.setPixelColor(dx, dy, grad ?  color_fade(color, (255-rate), true) : color); // use addPixelColorXY for different look
                buffer.addPixelColor(dx, dy, color); // use setPixelColorXY for different look
                if (grad)
                    buffer.fadePixelColor(dx, dy, rate);
            }

            if (dot) { //add white point at the ends of line
                buffer.setPixelColor(x1, x2, WHITE);
                buffer.setPixelColor(y1, y2, DARKSLATEGRAY);
            }
        }
        buffer.blur(SEGMENT.custom3>>1, SEGMENT.check2);
        return true;
    }

private:
    uint16_t aux0{};
};


#endif //WLED_DISABLE_2D
