#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * ICU mode
 */
class IcuEffect : public BaseEffect<IcuEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = IcuEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "ICU@!,!,,,,,Overlay;!,!;!";
    static constexpr const uint8_t effectId = FX_MODE_ICU;

    explicit IcuEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        // nextExecutionTimestamp rolls over before strip.now does
        if (strip.now < nextExecutionTimestamp) {
            return true;
        }

        unsigned dest = step & 0xFFFF;
        unsigned space = (SEGMENT.intensity >> 3) +2;

        if (!SEGMENT.check2) buffer.fill(SEGCOLOR(1));

        byte pindex = map(dest, 0, coordinate.width-coordinate.width/space, 0, 255);
        uint32_t col = SEGMENT.color_from_palette(pindex, false, false, 0);

        buffer.setPixelColor(dest, col);
        buffer.setPixelColor(dest + coordinate.width/space, col);

        if(aux0 == dest) { // pause between eye movements
            if(hw_random8(6) == 0) { // blink once in a while
                buffer.setPixelColor(dest, SEGCOLOR(1));
                buffer.setPixelColor(dest + coordinate.width/space, SEGCOLOR(1));
                nextExecutionTimestamp = strip.now + 200;
                return true;
            }
            aux0 = hw_random16(coordinate.width-coordinate.width/space);
            nextExecutionTimestamp = strip.now + 1000 + hw_random16(2000);
            return true;
        }

        if(aux0 > step) {
            step++;
            dest++;
        } else if (aux0 < step) {
            step--;
            dest--;
        }

        buffer.setPixelColor(dest, col);
        buffer.setPixelColor(dest + coordinate.width/space, col);

        nextExecutionTimestamp = strip.now + SPEED_FORMULA_L;
        return true;
    }

private:
    decltype(strip.now) nextExecutionTimestamp{};
    uint32_t step{};
    uint16_t aux0{};
};


