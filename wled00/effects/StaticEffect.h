#pragma once

#include "../FX.h"
#include "Effect.h"

class StaticEffect : public BaseEffect<StaticEffect> {
private:
    using Self = StaticEffect;
    using Base = BaseEffect<Self>;

public:
    static constexpr const char metaData[] PROGMEM = "Solid";
    static constexpr const uint8_t effectId = FX_MODE_STATIC;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d0;

    using Base::Base;

    constexpr void nextFrameImpl(const EffectCoordinate& coordinate) {
    }

    uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        return SEGCOLOR(0);
    }
};
