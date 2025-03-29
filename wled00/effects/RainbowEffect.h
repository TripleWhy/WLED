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
    static constexpr const char* const metaData = "Colorloop@!,Saturation;;!;01";
    static constexpr const uint8_t effectId = FX_MODE_RAINBOW;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d0;

    using Base::Base;

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        unsigned counter = (strip.now * ((SEGMENT.speed >> 2) +2)) & 0xFFFF;
        counter = counter >> 8;

        if (SEGMENT.intensity < 128){
          color = color_blend(SEGMENT.color_wheel(counter),WHITE,uint8_t(128-SEGMENT.intensity));
        } else {
          color = SEGMENT.color_wheel(counter);
        }
    }

    constexpr uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        return color;
    }

private:
    uint32_t color;
};
