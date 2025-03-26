#pragma once

#include "../FX.h"
#include "ColorWipeEffectBase.h"

/*
 * Lights all LEDs one after another.
 */
class ColorWipeRandomEffect : public BaseEffect<ColorWipeRandomEffect, ColorWipeEffectBase> {
private:
    using Self = ColorWipeRandomEffect;
    using Base = BaseEffect<Self, ColorWipeEffectBase>;

public:
    static constexpr const char* const metaData = "Wipe Random@!;;!";
    static constexpr const uint8_t effectId = FX_MODE_COLOR_WIPE_RANDOM;

    explicit constexpr ColorWipeRandomEffect(const EffectInformation& ei)
        : Base{ei, false, true}
    {
    }
};
