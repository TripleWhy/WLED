#pragma once

#include "../FX.h"
#include "Effect.h"

/*
 * Cycles a rainbow over the entire string of LEDs.
 */
class RainbowCycleEffect : public BaseEffect<RainbowCycleEffect> {
private:
    using Self = RainbowCycleEffect;
    using Base = BaseEffect<Self>;

public:
    static constexpr const char metaData[] PROGMEM = "Rainbow@!,Size;;!";
    static constexpr const uint8_t effectId = FX_MODE_RAINBOW_CYCLE;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d1;

    using Base::Base;

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        multiplier = (16 << (parameters.intensity /29));
        counter = (strip.now * ((parameters.speed >> 2) +2)) & 0xFFFF;
        counter = counter >> 8;
        return true;
    }

    uint32_t getPixelColorImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        //intensity/29 = 0 (1/16) 1 (1/8) 2 (1/4) 3 (1/2) 4 (1) 5 (2) 6 (4) 7 (8) 8 (16)
        uint8_t index = (coordinate.getXAbsolute() * multiplier / coordinate.width) + counter;
        return parameters.color_wheel(index);
    }

private:
    unsigned multiplier;
    unsigned counter;
};
