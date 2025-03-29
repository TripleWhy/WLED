#pragma once

#include "../FX.h"
#include "Effect.h"
#include "effectUtils.h"

/*
 * Running lights effect with smooth sine transition base.
 * Idea: Make the gap width controllable with a third slider in the future
 */
class RunningLightsEffect : public BaseEffect<RunningLightsEffect> {
private:
    using Self = RunningLightsEffect;
    using Base = BaseEffect<Self>;

public:
    static constexpr const char* const metaData = "Running@!,Width,,,,Rainbow,Dual,Saw;L,!,R;!";
    static constexpr const uint8_t effectId = FX_MODE_RUNNING_LIGHTS;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d1;

    using Base::Base;

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        x_scale = SEGMENT.intensity >> 2;
        counter = (strip.now * SEGMENT.speed) >> 9;
    }

    uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        const bool dual = SEGMENT.check2;
        const bool moving = SEGMENT.check1;
        const bool sawMode = SEGMENT.check3;

        const unsigned i = coordinate.getXAbsolute();
        unsigned a = i*x_scale - counter;
        if (sawMode) {
            a &= 0xFF;
            if (a < 16)
            {
                a = 192 + a*8;
            } else {
                a = map(a,16,255,64,192);
            }
            a = 255 - a;
        }
        unsigned palIdx = moving ? (i+counter)%coordinate.width : i;
        uint8_t s = dual ? sin_gap(a) : sin8_t(a);
        uint32_t ca = color_blend(SEGCOLOR(1), SEGMENT.color_from_palette(palIdx, true, moving, 0), s);
        if (dual) {
            unsigned b = (coordinate.width-1-i)*x_scale - counter;
            uint8_t t = sin_gap(b);
            uint32_t cb = color_blend(SEGCOLOR(1), SEGMENT.color_from_palette(palIdx, true, moving, 2), t);
            ca = color_blend(ca, cb, uint8_t(127));
        }
        return ca;
    }

private:
    unsigned x_scale;
    uint32_t counter;
};
