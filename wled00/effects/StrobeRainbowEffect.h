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
    static constexpr const char* const metaData = "Strobe Rainbow@!;,!;!;01";
    static constexpr const uint8_t effectId = FX_MODE_STROBE_RAINBOW;

    explicit constexpr StrobeRainbowEffect(const EffectInformation& ei)
        : Base{ei, true, false}
    {
    }

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(SEGMENT.color_wheel(SEGENV.call & 0xFF), SEGCOLOR(1));
    }
};
