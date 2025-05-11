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
    static constexpr const char metaData[] PROGMEM = "Blink Rainbow@Frequency,Blink duration;!,!;!;01";
    static constexpr const uint8_t effectId = FX_MODE_BLINK_RAINBOW;

    explicit constexpr BlinkRainbowEffect(const EffectInformation& ei)
        : Base{ei, false, false}
    {
    }

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        return Base::nextFrameImpl(parameters, SEGMENT.color_wheel(parameters.call & 0xFF), SEGCOLOR(1));
    }
};
