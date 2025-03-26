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
    explicit constexpr ColorWipeEffect(const EffectInformation& ei)
        : Base{ei, false, false}
    {
    }

    static constexpr EffectInformation effectInformation {
        "Wipe@!,!;!,!;!",
        FX_MODE_COLOR_WIPE,
        0u,
        1u,
        &Self::makeEffect,
        &Self::nextFrame,
        &Self::nextRow,
        &Self::getPixelColor,
    };
};
