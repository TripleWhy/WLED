#pragma once

#include "../FX.h"
#include "Effect.h"

/*
 * Cycles all LEDs at once through a rainbow.
 */
class RainbowEffect : public BaseEffect<RainbowEffect> {
private:
    using Self = RainbowEffect;
    using Base = BaseEffect<Self>;

public:
    static constexpr const char metaData[] PROGMEM = "Colorloop@!,Saturation;;!;01";
    static constexpr const uint8_t effectId = FX_MODE_RAINBOW;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d0;

    using Base::Base;

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        unsigned counter = (strip.now * ((parameters.speed >> 2) +2)) & 0xFFFF;
        counter = counter >> 8;

        if (parameters.intensity < 128){
          color = color_blend(SEGMENT.color_wheel(counter),WHITE,uint8_t(128-parameters.intensity));
        } else {
          color = SEGMENT.color_wheel(counter);
        }
        return true;
    }

    constexpr uint32_t getPixelColorImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        return color;
    }

private:
    uint32_t color;
};
