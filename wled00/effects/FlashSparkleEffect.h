#pragma once

#include "../FX.h"
#include "Effect.h"

/*
 * Lights all LEDs in the color. Flashes single col 1 pixels randomly. (List name: Sparkle Dark)
 * Inspired by www.tweaking4all.com/hardware/arduino/adruino-led-strip-effects/
 */
class FlashSparkleEffect : public BaseEffect<FlashSparkleEffect> {
private:
    using Self = FlashSparkleEffect;
    using Base = BaseEffect<Self>;

public:
    static constexpr const char metaData[] PROGMEM = "Sparkle Dark@!,!,,,,Move,Overlay;Bg,Fx;!;;m12=0";
    static constexpr const uint8_t effectId = FX_MODE_FLASH_SPARKLE;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d1;

    using Base::Base;

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        const uint32_t cycleTime = 10 + (255 - SEGMENT.speed)*2;
        it = strip.now / cycleTime;

        if (strip.now > flashTimestamp + flashPauseDuration) {
            if(hw_random8((255-SEGMENT.intensity) >> 4) == 0) {
                flashLedIndex = hw_random16(coordinate.width);
            } else {
                flashLedIndex = -1;
            }
            flashTimestamp = strip.now;
            flashPauseDuration = 255-SEGMENT.speed;
        }
        return true;
    }

    uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        const unsigned i = coordinate.getXAbsolute();
        if (((int32_t)i == flashLedIndex) && (flashTimestamp == strip.now)) {
            return SEGCOLOR(1);
        }

        const bool overlay = SEGMENT.check2;
        if (overlay) {
            return currentColor.getColor();
        }

        const bool moving = SEGMENT.check1;
        const unsigned palIdx = moving ? (i+it)%coordinate.width : i;
        return SEGMENT.color_from_palette(palIdx, true, moving, 0);
    }

private:
    uint32_t it;
    uint32_t flashTimestamp{};
    uint16_t flashPauseDuration{};
    int32_t flashLedIndex{};
};
