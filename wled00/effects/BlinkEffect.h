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
    static constexpr const char* const metaData = "Blink@!,Duty cycle;!,!;!;01";
    static constexpr const uint8_t effectId = FX_MODE_BLINK;

    explicit constexpr BlinkEffect(const EffectInformation& ei)
        : Base{ei, false, true}
    {
    }

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(SEGCOLOR(0), SEGCOLOR(1));
    }
};
