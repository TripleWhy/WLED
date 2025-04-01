#pragma once

#include <cmath>
#include "../FX.h"
#include "Effect.h"

/*
 * Gradient run base function
 */
class GradientEffectBase : public Effect {
private:
    using Self = GradientEffectBase;
    using Base = Effect;

public:
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d1;

    explicit constexpr GradientEffectBase(const EffectInformation& ei, bool loading)
        : Base{ei},
          loading{loading}
    {
    }

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        uint16_t counter = strip.now * ((SEGMENT.speed >> 2) + 1);
        pp = (counter * coordinate.width) >> 16;
        if (SEGENV.call == 0)
            pp = 0;
        brd = 1 + loading ? SEGMENT.intensity/2 : SEGMENT.intensity/4;
        //if (brd < 1) brd = 1;
        p1 = pp-coordinate.width;
        p2 = pp+coordinate.width;
    }

    uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        int val; //0 = sec 1 = pri
        const int i = static_cast<int>(coordinate.getXAbsolute());
        if (loading) {
            val = abs(((i>pp) ? p2:pp) - i);
        } else {
            val = min(abs(pp-i),min(abs(p1-i),abs(p2-i)));
        }
        val = (brd > val) ? (val * 255) / brd : 255;
        return color_blend(SEGCOLOR(0), SEGMENT.color_from_palette(i, true, PALETTE_SOLID_WRAP, 1), uint8_t(val));
    }

private:
    const bool loading;

    uint16_t pp{};
    int brd{};
    int p1{};
    int p2{};
};
