#pragma once

#include "../FX.h"
#include "BlinkEffectBase.h"

/*
 * Classic Strobe effect. Cycling through the rainbow.
 */
class StrobeRainbowEffect : public BaseEffect<StrobeRainbowEffect, BlinkEffectBase> {
private:
    using Self = StrobeRainbowEffect;
    using Base = BaseEffect<Self, BlinkEffectBase>;

public:
    explicit constexpr StrobeRainbowEffect(const EffectInformation& ei)
        : Base{ei, true, false}
    {
    }

    static constexpr EffectInformation effectInformation {
        "Strobe Rainbow@!;,!;!;01",
        FX_MODE_STROBE_RAINBOW,
        0u,
        &Self::makeEffect,
        &Self::nextFrame,
        &Self::nextRow,
        &Self::getPixelColor,
    };

    void nextFrameImpl() {
        Base::nextFrameImpl(SEGMENT.color_wheel(SEGENV.call & 0xFF), SEGCOLOR(1));
    }
};
