#pragma once

#include "../FX.h"
#include "Effect.h"

/*
 * Lights every LED in a random color. Changes all LED at the same time
 * to new random colors.
 */
class DynamicEffect : public BaseEffect<DynamicEffect> {
private:
    using Self = DynamicEffect;
    using Base = BaseEffect<Self>;

public:
    static constexpr const char metaData[] PROGMEM = "Dynamic@!,!,,,,Smooth;;!";
    static constexpr const uint8_t effectId = FX_MODE_DYNAMIC;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d1;

    using Base::Base;

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!colorIndexes.resize(coordinate.width)) {
            return false;
        }

        if(parameters.call == 0) {
            for (unsigned i = 0; i < coordinate.width; i++) colorIndexes[i] = hw_random8();
        }

        uint32_t cycleTime = 50 + (255 - parameters.speed)*15;
        uint32_t it = strip.now / cycleTime;
        if (it != step && parameters.speed != 0) //new color
        {
            for (unsigned i = 0; i < coordinate.width; i++) {
                if (hw_random8() <= parameters.intensity) colorIndexes[i] = hw_random8(); // random color index
            }
            step = it;
        }

        return false;
    }

    uint32_t getPixelColorImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        uint32_t color = SEGMENT.color_wheel(colorIndexes[coordinate.getXAbsolute()]);
        if (parameters.check1) {
            color = color_blend(currentColor.getColor(), color, 16);
        }
        return color;
    }

private:
    uint32_t step{};
    SegmentAllocator<uint8_t>::vector colorIndexes;
};
