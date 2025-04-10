#pragma once

#include "../FX.h"
#include "Effect.h"
#include "PhasedEffectBase.h"

class PhasedEffect : public BaseEffect<PhasedEffect, PhasedEffectBase> {
private:
    using Self = PhasedEffect;
    using Base = BaseEffect<Self, PhasedEffectBase>;

public:
    static constexpr const char metaData[] PROGMEM = "Phased@!,!;!,!;!";
    static constexpr const uint8_t effectId = FX_MODE_PHASED;

    explicit PhasedEffect(const EffectInformation& ei) : Base{ei, false} {}

private:
};


