#pragma once

#include "../FX.h"
#include "Effect.h"
#include "GravcenterEffectBase.h"

///////////////////////
//    ** Gravfreq    //
///////////////////////
// Gravfreq. By Andrew Tuline.
class GravfreqEffect : public BaseEffect<GravfreqEffect, GravcenterEffectBase> {
private:
    using Self = GravfreqEffect;
    using Base = BaseEffect<Self, GravcenterEffectBase>;

public:
    static constexpr const char metaData[] PROGMEM = "Gravfreq@Rate of fall,Sensitivity;!,!;!;1f;ix=128,m12=0,si=0";
    static constexpr const uint8_t effectId = FX_MODE_GRAVFREQ;

    explicit GravfreqEffect(const EffectInformation& ei) : Base{ei, 3} {}

private:
};


