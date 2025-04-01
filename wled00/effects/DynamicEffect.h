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
    static constexpr const char* const metaData = "Dynamic@!,!,,,,Smooth;;!";
    static constexpr const uint8_t effectId = FX_MODE_DYNAMIC;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d1;

    using Base::Base;

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        colorIndexes.resize(coordinate.width);
        if (colorIndexes.size() != coordinate.width) {
            colorIndexes.clear();
            return;
        }
        colorIndexes.shrink_to_fit();

        if(SEGENV.call == 0) {
            for (unsigned i = 0; i < coordinate.width; i++) colorIndexes[i] = hw_random8();
        }

        uint32_t cycleTime = 50 + (255 - SEGMENT.speed)*15;
        uint32_t it = strip.now / cycleTime;
        if (it != step && SEGMENT.speed != 0) //new color
        {
            for (unsigned i = 0; i < coordinate.width; i++) {
                if (hw_random8() <= SEGMENT.intensity) colorIndexes[i] = hw_random8(); // random color index
            }
            step = it;
        }
    }

    uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        uint32_t color = SEGMENT.color_wheel(colorIndexes[coordinate.getXAbsolute()]);
        if (SEGMENT.check1) {
            color = color_blend(currentColor.getColor(), color, 16);
        }
        return color;
    }

private:
    uint32_t step{};
    std::vector<uint8_t> colorIndexes;
};
