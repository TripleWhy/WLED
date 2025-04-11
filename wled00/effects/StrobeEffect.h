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
    static constexpr const char metaData[] PROGMEM = "Strobe@!;!,!;!;01";
    static constexpr const uint8_t effectId = FX_MODE_STROBE;

    explicit constexpr StrobeEffect(const EffectInformation& ei)
        : Base{ei, true, true}
    {
    }

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        return Base::nextFrameImpl(SEGCOLOR(0), SEGCOLOR(1));
    }
};
