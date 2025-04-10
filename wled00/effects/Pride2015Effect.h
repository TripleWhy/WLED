#pragma once

#include "../FX.h"
#include "ColorwavesPrideEffectBase.h"
#include "Effect.h"

// Pride2015
// Animated, ever-changing rainbows.
// by Mark Kriegsman: https://gist.github.com/kriegsman/964de772d64c502760e5
class Pride2015Effect : public BaseEffect<Pride2015Effect, ColorwavesPrideEffectBase> {
private:
    using Self = Pride2015Effect;
    using Base = BaseEffect<Self, ColorwavesPrideEffectBase>;

public:
    static constexpr const char metaData[] PROGMEM = "Pride 2015@!;;";
    static constexpr const uint8_t effectId = FX_MODE_PRIDE_2015;

    explicit Pride2015Effect(const EffectInformation& ei) : Base{ei, true} {}

private:
};
