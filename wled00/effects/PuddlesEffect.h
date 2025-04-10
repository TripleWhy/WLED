#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

// Puddles. By Andrew Tuline.
class PuddlesEffect : public BaseEffect<PuddlesEffect, PuddlesEffectBase> {
private:
    using Self = PuddlesEffect;
    using Base = BaseEffect<Self, PuddlesEffectBase>;

public:
    static constexpr const char metaData[] PROGMEM = "Puddles@Fade rate,Puddle size;!,!;!;1v;m12=0,si=0";
    static constexpr const uint8_t effectId = FX_MODE_PUDDLES;

    explicit PuddlesEffect(const EffectInformation& ei) : Base{ei, false} {}

private:
};


