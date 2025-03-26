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
    explicit constexpr ColorSweepRandomEffect(const EffectInformation& ei)
        : Base{ei, true, true}
    {
    }

    static constexpr EffectInformation effectInformation {
        "Sweep Random@!;;!",
        FX_MODE_COLOR_SWEEP_RANDOM,
        0u,
        1u,
        &Self::makeEffect,
        &Self::nextFrame,
        &Self::nextRow,
        &Self::getPixelColor,
    };
};
