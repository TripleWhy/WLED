#pragma once

#include "../FX.h"
#include "Effect.h"

class StaticEffect : public BaseEffect<StaticEffect> {
private:
    using Self = StaticEffect;
    using Base = BaseEffect<StaticEffect>;

public:
    using Base::Base;

    static constexpr EffectInformation effectInformation {
        "Solid",
        FX_MODE_STATIC,
        0u,
        &Self::makeEffect,
        &Self::nextFrame,
        &Self::nextRow,
        &Self::getPixelColor,
    };

    constexpr void nextFrameImpl() {
    }

    constexpr void nextRowImpl(unsigned y) {
    }

    uint32_t getPixelColorImpl(unsigned x, unsigned y, const LazyColor& currentColor) {
        return SEGCOLOR(0);
    }
};
