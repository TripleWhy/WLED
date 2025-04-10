#pragma once

#include "../FX.h"
#include "Effect.h"
#include "PhasedEffectBase.h"

class PhasedNoiseEffect : public BaseEffect<PhasedNoiseEffect, PhasedEffectBase> {
private:
    using Self = PhasedNoiseEffect;
    using Base = BaseEffect<Self, PhasedEffectBase>;

public:
    static constexpr const char metaData[] PROGMEM = "Phased Noise@!,!;!,!;!";
    static constexpr const uint8_t effectId = FX_MODE_PHASEDNOISE;

    explicit PhasedNoiseEffect(const EffectInformation& ei) : Base{ei, true} {}

private:
};


