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
    explicit constexpr ColorSweepEffect(const EffectInformation& ei)
        : Base{ei, true, false}
    {
    }

    static constexpr EffectInformation effectInformation {
        "Sweep@!,!;!,!;!",
        FX_MODE_COLOR_SWEEP,
        0u,
        1u,
        &Self::makeEffect,
        &Self::nextFrame,
        &Self::nextRow,
        &Self::getPixelColor,
    };
};
