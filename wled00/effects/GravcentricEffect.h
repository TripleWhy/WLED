#pragma once

#include "../FX.h"
#include "Effect.h"
#include "GravcenterEffectBase.h"

///////////////////////
//   * GRAVCENTRIC   //
///////////////////////
// Gravcentric. By Andrew Tuline.
class GravcentricEffect : public BaseEffect<GravcentricEffect, GravcenterEffectBase> {
private:
    using Self = GravcentricEffect;
    using Base = BaseEffect<Self, GravcenterEffectBase>;

public:
    static constexpr const char* const metaData = "Gravcentric@Rate of fall,Sensitivity;!,!;!;1v;ix=128,m12=3,si=0";
    static constexpr const uint8_t effectId = FX_MODE_GRAVCENTRIC;

    explicit GravcentricEffect(const EffectInformation& ei) : Base{ei, 1} {}

private:
};


