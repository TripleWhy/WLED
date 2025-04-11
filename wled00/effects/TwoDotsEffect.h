#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Two dots running
 */
class TwoDotsEffect : public BaseEffect<TwoDotsEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = TwoDotsEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Two Dots@!,Dot size,,,,,Overlay;1,2,Bg;!";
    static constexpr const uint8_t effectId = FX_MODE_TWO_DOTS;

    explicit TwoDotsEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        unsigned delay = 1 + (FRAMETIME<<3) / coordinate.width;  // longer segments should change faster
        uint32_t it = strip.now / map(SEGMENT.speed, 0, 255, delay<<4, delay);
        unsigned offset = it % coordinate.width;
        unsigned width = ((coordinate.width*(SEGMENT.intensity+1))>>9); //max width is half the strip
        if (!width) width = 1;
        if (!SEGMENT.check2) buffer.fill(SEGCOLOR(2));
        const uint32_t color1 = SEGCOLOR(0);
        const uint32_t color2 = (SEGCOLOR(1) == SEGCOLOR(2)) ? color1 : SEGCOLOR(1);
        for (unsigned i = 0; i < width; i++) {
            unsigned indexR = (offset + i) % coordinate.width;
            unsigned indexB = (offset + i + (coordinate.width>>1)) % coordinate.width;
            buffer.setPixelColor(indexR, color1);
            buffer.setPixelColor(indexB, color2);
        }
        return true;
    }

private:
};


