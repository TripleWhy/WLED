#pragma once

#include "../FX.h"
#include "Effect.h"

/*
* Color wipe function
* LEDs are turned on (color1) in sequence, then turned off (color2) in sequence.
* if (bool rev == true) then LEDs are turned off in reverse order
*/
class ColorWipeEffectBase : public Effect {
private:
    using Self = ColorWipeEffectBase;
    using Base = Effect;

public:
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d1;

    explicit constexpr ColorWipeEffectBase(const EffectInformation& ei, bool rev, bool useRandomColors)
        : Base{ei},
        rev{rev},
        useRandomColors{useRandomColors}
    {
    }

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        uint32_t cycleTime = 750 + (255 - parameters.speed)*150;
        uint32_t perc = strip.now % cycleTime;
        unsigned prog = (perc * 65535) / cycleTime;
        if (prog > 32767) {
            back = true;
            if (rev) {
                prog = 65535 - prog;
            } else {
                prog -= 32767;
            }
            if (step == 0) step = 1;
        } else {
            back = false;
            if (step == 2) step = 3; //trigger color change
        }

        if (useRandomColors) {
            if (parameters.call == 0) {
                colorIndex[0] = hw_random8();
                step = 3;
            }
            if (step == 1) { //if flag set, change to new random color
                colorIndex[1] = get_random_wheel_index(colorIndex[0]);
                step = 2;
            }
            if (step == 3) {
                colorIndex[0] = get_random_wheel_index(colorIndex[1]);
                step = 0;
            }

            col[0] = SEGMENT.color_wheel(colorIndex[0]);
            col[1] = SEGMENT.color_wheel(colorIndex[1]);
        } else {
            col[1] = SEGCOLOR(1);
        }

        ledIndex = (prog * coordinate.width) >> 15;
        rem = (prog * coordinate.width) * 2; //mod 0xFFFF
        rem /= (parameters.intensity +1);
        if (rem > 255) rem = 255;
        return true;
    }

    uint32_t getPixelColorImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        const unsigned i = coordinate.getXAbsolute();
        if (!useRandomColors) {
            col[0] = SEGMENT.color_from_palette(i, true, PALETTE_SOLID_WRAP, 0);
        }

        // Some magic to determine which color to show. I didn't arrive at this expression by logical deduction,
        // it's simply an expression that produces the correct result when looking at a truth table.
        const size_t colorIndex = static_cast<size_t>(((i > ledIndex) == (back || rev)));
        if (i == ledIndex) {
            return color_blend(col[1u - colorIndex], col[colorIndex], uint8_t(rem));
        } else {
            return col[colorIndex];
        }
    }

private:
    uint32_t step{};
    bool rev;
    bool useRandomColors;
    bool back{};
    unsigned ledIndex{};
    uint8_t colorIndex[2]{};
    uint32_t col[2]{};
    uint16_t rem{};
};
