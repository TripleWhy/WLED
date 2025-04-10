#pragma once

#include "../FX.h"
#include "Effect.h"

/*
 * Blinks one LED at a time.
 * Inspired by www.tweaking4all.com/hardware/arduino/adruino-led-strip-effects/
 */
class SparkleEffect : public BaseEffect<SparkleEffect> {
private:
    using Self = SparkleEffect;
    using Base = BaseEffect<Self>;

public:
    static constexpr const char metaData[] PROGMEM = "Sparkle@!,,,,,Move,Overlay;!,!;!;;m12=0,01=0";
    static constexpr const uint8_t effectId = FX_MODE_SPARKLE;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d1;

    using Base::Base;

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        const uint32_t cycleTime = 10 + (255 - SEGMENT.speed)*2;
        it = strip.now / cycleTime;
        if (it != step)
        {
            randomLedIndex = hw_random16(coordinate.width);
            step = it;
        }
    }

    uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        const unsigned i = coordinate.getXAbsolute();
        if (i == randomLedIndex) {
            return SEGCOLOR(0);
        }

        const bool overlay = SEGMENT.check2;
        if (overlay) {
            return currentColor.getColor();
        }

        const bool moving = SEGMENT.check1;
        const unsigned palIdx = moving ? (i+it)%coordinate.width : i;
        return SEGMENT.color_from_palette(palIdx, true, moving, 1);
    }

private:
    uint32_t it;
    uint32_t step{};
    uint16_t randomLedIndex;
};
