#pragma once

#include "../FX.h"
#include "Effect.h"

/*
 * Runs a single pixel back and forth.
 */
class ScanEffect : public BaseEffect<ScanEffect> {
private:
    using Self = ScanEffect;
    using Base = BaseEffect<Self>;

public:
    static constexpr const char* const metaData = "Scan@!,Size,,,,,Overlay,Dual;!,!,!;!;1;o1=0";
    static constexpr const uint8_t effectId = FX_MODE_SCAN;
    static constexpr const uint8_t defaultPaletteId = 0u;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d1;

    using Base::Base;

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        uint32_t cycleTime = 750 + (255 - SEGMENT.speed)*150;
        uint32_t perc = strip.now % cycleTime;
        int prog = (perc * 65535) / cycleTime;
        size = 1 + ((SEGMENT.intensity * coordinate.width) >> 9);
        int ledIndex = (prog * ((coordinate.width *2) - size *2)) >> 16;
        led_offset = static_cast<unsigned>(std::abs(ledIndex - (static_cast<int>(coordinate.width) - static_cast<int>(size))));
    }

    uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        const bool dual = SEGMENT.check3;

        const unsigned x = coordinate.getXAbsolute();
        if (led_offset <= x && x < led_offset + size) {
            return SEGMENT.color_from_palette(x, true, PALETTE_SOLID_WRAP, 0);
        }
        if (dual) {
            unsigned x2 = coordinate.width - 1 - x;
            if (led_offset <= x2 && x2 < led_offset + size) {
                return SEGMENT.color_from_palette(x2, true, PALETTE_SOLID_WRAP, (SEGCOLOR(2))? 2:0);
            }
        }
        if (SEGMENT.check2) {
            return currentColor.getColor();
        }
        return SEGCOLOR(1);
    }

private:
    unsigned size;
    unsigned led_offset;
};
