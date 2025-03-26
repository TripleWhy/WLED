#pragma once

#include "../FX.h"
#include "ColorWipeEffectBase.h"

/*
 * Random color introduced alternating from start and end of strip.
 */
class ColorSweepRandomEffect : public BaseEffect<ColorSweepRandomEffect, ColorWipeEffectBase> {
private:
    using Self = ColorSweepRandomEffect;
    using Base = BaseEffect<Self, ColorWipeEffectBase>;

public:
    static constexpr const char* const metaData = "Sweep Random@!;;!";
    static constexpr const uint8_t effectId = FX_MODE_COLOR_SWEEP_RANDOM;

    explicit constexpr ColorSweepRandomEffect(const EffectInformation& ei)
        : Base{ei, true, true}
    {
    }
};
