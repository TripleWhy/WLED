#pragma once

#include "../FX.h"
#include "BlinkEffectBase.h"

/*
 * Classic Strobe effect.
 */
class StrobeEffect : public BaseEffect<StrobeEffect, BlinkEffectBase> {
private:
    using Self = StrobeEffect;
    using Base = BaseEffect<Self, BlinkEffectBase>;

public:
    explicit constexpr StrobeEffect(const EffectInformation& ei)
        : Base{ei, true, true}
    {
    }

    static constexpr EffectInformation effectInformation {
        "Strobe@!;!,!;!;01",
        FX_MODE_STROBE,
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
