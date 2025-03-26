#pragma once

#include "../FX.h"
#include "ColorWipeEffectBase.h"

/*
 * Lights all LEDs one after another. Turns off opposite
 */
class ColorSweepEffect : public BaseEffect<ColorSweepEffect, ColorWipeEffectBase> {
private:
    using Self = ColorSweepEffect;
    using Base = BaseEffect<Self, ColorWipeEffectBase>;

public:
    static constexpr const char* const metaData = "Sweep@!,!;!,!;!";
    static constexpr const uint8_t effectId = FX_MODE_COLOR_SWEEP;

    explicit constexpr ColorSweepEffect(const EffectInformation& ei)
        : Base{ei, true, false}
    {
    }
};
