#pragma once

#include "../FX.h"
#include "ColorWipeEffectBase.h"

/*
 * Lights all LEDs one after another.
 */
class ColorWipeEffect : public BaseEffect<ColorWipeEffect, ColorWipeEffectBase> {
private:
    using Self = ColorWipeEffect;
    using Base = BaseEffect<Self, ColorWipeEffectBase>;

public:
    static constexpr const char* const metaData = "Wipe@!,!;!,!;!";
    static constexpr const uint8_t effectId = FX_MODE_COLOR_WIPE;

    explicit constexpr ColorWipeEffect(const EffectInformation& ei)
        : Base{ei, false, false}
    {
    }
};
