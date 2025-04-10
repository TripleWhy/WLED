#pragma once

#include "../FX.h"
#include "Effect.h"
#include "TwinklefoxEffectBase.h"

class TwinklefoxEffect : public BaseEffect<TwinklefoxEffect, TwinklefoxEffectBase> {
private:
    using Self = TwinklefoxEffect;
    using Base = BaseEffect<Self, TwinklefoxEffectBase>;

public:
    static constexpr const char metaData[] PROGMEM = "Twinklefox@!,Twinkle rate,,,,Cool;!,!;!";
    static constexpr const uint8_t effectId = FX_MODE_TWINKLEFOX;

    explicit TwinklefoxEffect(const EffectInformation& ei) : Base{ei, false} {}

private:
};


