#pragma once

#include "../FX.h"
#include "Effect.h"
#include "GradientEffectBase.h"

/*
 * Gradient run
 */
class GradientEffect : public BaseEffect<GradientEffect, GradientEffectBase> {
private:
    using Self = GradientEffect;
    using Base = BaseEffect<Self, GradientEffectBase>;

public:
    static constexpr const char metaData[] PROGMEM = "Gradient@!,Spread;!,!;!;;ix=16";
    static constexpr const uint8_t effectId = FX_MODE_GRADIENT;

    explicit constexpr GradientEffect(const EffectInformation& ei)
        : Base{ei, false}
    {
    }

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);
    }
};
