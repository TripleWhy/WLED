#pragma once

#include "../FX.h"
#include "Effect.h"
#include "GravcenterEffectBase.h"

// Gravcenter. By Andrew Tuline.
class GravcenterEffect : public BaseEffect<GravcenterEffect, GravcenterEffectBase> {
private:
    using Self = GravcenterEffect;
    using Base = BaseEffect<Self, GravcenterEffectBase>;

public:
    static constexpr const char* const metaData = "Gravcenter@Rate of fall,Sensitivity;!,!;!;1v;ix=128,m12=2,si=0";
    static constexpr const uint8_t effectId = FX_MODE_GRAVCENTER;

    explicit GravcenterEffect(const EffectInformation& ei) : Base{ei, 0} {}

private:
};


