#pragma once

#include "../FX.h"
#include "Effect.h"
#include "TwinklefoxEffectBase.h"

class TwinklecatEffect : public BaseEffect<TwinklecatEffect, TwinklefoxEffectBase> {
private:
    using Self = TwinklecatEffect;
    using Base = BaseEffect<Self, TwinklefoxEffectBase>;

public:
    static constexpr const char* const metaData = "Twinklecat@!,Twinkle rate,,,,Cool;!,!;!";
    static constexpr const uint8_t effectId = FX_MODE_TWINKLECAT;

    explicit TwinklecatEffect(const EffectInformation& ei) : Base{ei, true} {}

private:
};


