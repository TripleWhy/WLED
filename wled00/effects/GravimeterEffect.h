#pragma once

#include "../FX.h"
#include "GravcenterEffectBase.h"
#include "Effect.h"

///////////////////////
//   * GRAVIMETER    //
///////////////////////
// Gravmeter. By Andrew Tuline.
class GravimeterEffect : public BaseEffect<GravimeterEffect, GravcenterEffectBase> {
private:
    using Self = GravimeterEffect;
    using Base = BaseEffect<Self, GravcenterEffectBase>;

public:
    static constexpr const char metaData[] PROGMEM = "Gravimeter@Rate of fall,Sensitivity;!,!;!;1v;ix=128,m12=2,si=0";
    static constexpr const uint8_t effectId = FX_MODE_GRAVIMETER;

    explicit GravimeterEffect(const EffectInformation& ei) : Base{ei, 2} {}

private:
};


