#pragma once

#include "../FX.h"
#include "Effect.h"
#include "PuddlesEffectBase.h"

class PuddlepeakEffect : public BaseEffect<PuddlepeakEffect, PuddlesEffectBase> {
private:
    using Self = PuddlepeakEffect;
    using Base = BaseEffect<Self, PuddlesEffectBase>;

public:
    static constexpr const char* const metaData = "Puddlepeak@Fade rate,Puddle size,Select bin,Volume (min);!,!;!;1v;c2=0,m12=0,si=0";
    static constexpr const uint8_t effectId = FX_MODE_PUDDLEPEAK;

    explicit PuddlepeakEffect(const EffectInformation& ei) : Base{ei, true} {}

private:
};


