#pragma once

#include "../FX.h"
#include "BlinkEffectBase.h"

/*
 * Normal blinking. Intensity sets duty cycle.
 */
class BlinkEffect : public BaseEffect<BlinkEffect, BlinkEffectBase> {
private:
    using Self = BlinkEffect;
    using Base = BaseEffect<Self, BlinkEffectBase>;

public:
    explicit constexpr BlinkEffect(const EffectInformation& ei)
        : Base{ei, false, true}
    {
    }

    static constexpr EffectInformation effectInformation {
        "Blink@!,Duty cycle;!,!;!;01",
        FX_MODE_BLINK,
        0u,
        1u,
        &Self::makeEffect,
        &Self::nextFrame,
        &Self::nextRow,
        &Self::getPixelColor,
    };

    void nextFrameImpl() {
        Base::nextFrameImpl(SEGCOLOR(0), SEGCOLOR(1));
    }
};
