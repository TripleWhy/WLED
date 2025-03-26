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
    static constexpr const char* const metaData = "Blink Rainbow@Frequency,Blink duration;!,!;!;01";
    static constexpr const uint8_t effectId = FX_MODE_BLINK_RAINBOW;

    explicit constexpr BlinkRainbowEffect(const EffectInformation& ei)
        : Base{ei, false, false}
    {
    }

    void nextFrameImpl() {
        Base::nextFrameImpl(SEGMENT.color_wheel(SEGENV.call & 0xFF), SEGCOLOR(1));
    }
};
