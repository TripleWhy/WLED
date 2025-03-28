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
    static constexpr const uint8_t defaultPaletteId = 0u;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d1;

    explicit constexpr ColorWipeEffectBase(const EffectInformation& ei, bool rev, bool useRandomColors)
        : Base{ei},
        rev{rev},
        useRandomColors{useRandomColors}
    {
    }

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        uint32_t cycleTime = 750 + (255 - SEGMENT.speed)*150;
        uint32_t perc = strip.now % cycleTime;
        unsigned prog = (perc * 65535) / cycleTime;
        if (prog > 32767) {
            back = true;
            if (rev) {
                prog = 65535 - prog;
            } else {
                prog -= 32767;
            }
            if (SEGENV.step == 0) SEGENV.step = 1;
        } else {
            back = false;
            if (SEGENV.step == 2) SEGENV.step = 3; //trigger color change
        }

        if (useRandomColors) {
            if (SEGENV.call == 0) {
                SEGENV.aux0 = hw_random8();
                SEGENV.step = 3;
            }
            if (SEGENV.step == 1) { //if flag set, change to new random color
                SEGENV.aux1 = get_random_wheel_index(SEGENV.aux0);
                SEGENV.step = 2;
            }
            if (SEGENV.step == 3) {
                SEGENV.aux0 = get_random_wheel_index(SEGENV.aux1);
                SEGENV.step = 0;
            }

            col[1] = SEGMENT.color_wheel(SEGENV.aux1);
            col[0] = SEGMENT.color_wheel(SEGENV.aux0);
        } else {
            col[1] = SEGCOLOR(1);
        }

        ledIndex = (prog * coordinate.width) >> 15;
        rem = (prog * coordinate.width) * 2; //mod 0xFFFF
        rem /= (SEGMENT.intensity +1);
        if (rem > 255) rem = 255;
    }

    uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
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
    bool rev;
    bool useRandomColors;
    bool back{};
    unsigned ledIndex{};
    uint32_t col[2]{};
    uint16_t rem{};
};
