#pragma once

#include "../FX.h"
#include "Effect.h"

class StaticEffect : public BaseEffect<StaticEffect> {
private:
    using Self = StaticEffect;
    using Base = BaseEffect<Self>;

public:
    static constexpr const char* const metaData = "Solid";
    static constexpr const uint8_t effectId = FX_MODE_STATIC;
    static constexpr const uint8_t defaultPaletteId = 0u;
    static constexpr const uint8_t maxDimensions = 2u;

    using Base::Base;

    constexpr void nextFrameImpl() {
    }

    uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        return SEGCOLOR(0);
    }
};
