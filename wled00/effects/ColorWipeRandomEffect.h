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
    explicit constexpr ColorWipeRandomEffect(const EffectInformation& ei)
        : Base{ei, false, true}
    {
    }

    static constexpr EffectInformation effectInformation {
        "Wipe Random@!;;!",
        FX_MODE_COLOR_WIPE_RANDOM,
        0u,
        1u,
        &Self::makeEffect,
        &Self::nextFrame,
        &Self::nextRow,
        &Self::getPixelColor,
    };
};
