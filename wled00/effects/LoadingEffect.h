#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Gradient run with hard transition
 */
class LoadingEffect : public BaseEffect<LoadingEffect, GradientEffectBase> {
private:
    using Self = LoadingEffect;
    using Base = BaseEffect<Self, GradientEffectBase>;

public:
    static constexpr const char metaData[] PROGMEM = "Loading@!,Fade;!,!;!;;ix=16";
    static constexpr const uint8_t effectId = FX_MODE_LOADING;

    explicit constexpr LoadingEffect(const EffectInformation& ei)
        : Base{ei, true}
    {
    }
};
