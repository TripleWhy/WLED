#pragma once

#include "../FX.h"
#include "Effect.h"

class StaticEffect : public BaseEffect<StaticEffect> {
private:
    using Self = StaticEffect;
    using Base = BaseEffect<Self>;

public:
    using Base::Base;

    static constexpr EffectInformation effectInformation {
        "Solid",
        FX_MODE_STATIC,
        0u,
        2u,
        &Self::makeEffect,
        &Self::nextFrame,
        &Self::nextRow,
        &Self::getPixelColor,
    };

    constexpr void nextFrameImpl() {
    }

    uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        return SEGCOLOR(0);
    }
};
