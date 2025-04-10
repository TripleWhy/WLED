#pragma once

#include "../FX.h"
#include "Effect.h"

/*
 * Lights all LEDs up in one random color. Then switches them
 * to the next random color.
 */
class RandomColorEffect : public BaseEffect<RandomColorEffect> {
private:
    using Self = RandomColorEffect;
    using Base = BaseEffect<Self>;

public:
    static constexpr const char metaData[] PROGMEM = "Random Colors@!,Fade time;;!;01";
    static constexpr const uint8_t effectId = FX_MODE_RANDOM_COLOR;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d0;

    using Base::Base;

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        uint32_t cycleTime = 200 + (255 - SEGMENT.speed)*50;
        uint32_t it = strip.now / cycleTime;
        uint32_t rem = strip.now % cycleTime;
        unsigned fadedur = (cycleTime * SEGMENT.intensity) >> 8;

        uint32_t fade = 255;
        if (fadedur) {
            fade = (rem * 255) / fadedur;
            if (fade > 255) fade = 255;
        }

        if (SEGENV.call == 0) {
            colorWheelIndex = hw_random8();
            step = 2;
        }
        if (it != step) //new color
        {
            previousColorWheelIndex = colorWheelIndex;
            colorWheelIndex = get_random_wheel_index(colorWheelIndex);
            step = it;
        }

        color = color_blend(SEGMENT.color_wheel(previousColorWheelIndex), SEGMENT.color_wheel(colorWheelIndex), uint8_t(fade));
    }

    constexpr uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        return color;
    }

private:
    uint32_t step{};
    uint32_t color{};
    uint8_t colorWheelIndex{};
    uint8_t previousColorWheelIndex{};
};
