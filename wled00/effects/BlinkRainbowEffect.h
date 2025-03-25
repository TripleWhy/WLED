#pragma once

#include "../FX.h"
#include "BlinkEffectBase.h"

/*
 * Classic Blink effect. Cycling through the rainbow.
 */
class BlinkRainbowEffect : public BaseEffect<BlinkRainbowEffect, BlinkEffectBase> {
private:
    using Self = BlinkRainbowEffect;
    using Base = BaseEffect<Self, BlinkEffectBase>;

public:
    explicit constexpr BlinkRainbowEffect(const EffectInformation& ei)
        : Base{ei, false, false}
    {
    }

    static constexpr EffectInformation effectInformation {
        "Blink Rainbow@Frequency,Blink duration;!,!;!;01",
        FX_MODE_BLINK_RAINBOW,
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
