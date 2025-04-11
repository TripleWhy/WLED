#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Strobe effect with different strobe count and pause, controlled by speed.
 */
class MultiStrobeEffect : public BaseEffect<MultiStrobeEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = MultiStrobeEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Strobe Mega@!,!,,,,Move;!,!;!;01;o1=0";
    static constexpr const uint8_t effectId = FX_MODE_MULTI_STROBE;

    explicit MultiStrobeEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        uint32_t cycleTime = 10 + (255 - SEGMENT.speed)*2;
        uint32_t it = strip.now / cycleTime;
        const bool moving = SEGMENT.check1;
        for (unsigned i = 0; i < coordinate.width; i++) {
            unsigned palIdx = moving ? (i+it)%coordinate.width : i;
            buffer.setPixelColor(i, SEGMENT.color_from_palette(palIdx, true, moving, 1));
        }

        aux0 = 50 + 20*(uint16_t)(255-SEGMENT.speed);
        unsigned count = 2 * ((SEGMENT.intensity / 10) + 1);
        if(aux1 < count) {
            if((aux1 & 1) == 0) {
                buffer.fill(SEGCOLOR(0));
                aux0 = 15;
            } else {
                aux0 = 50;
            }
        }

        if (strip.now - aux0 > step) {
            aux1++;
            if (aux1 > count) aux1 = 0;
            step = strip.now;
        }
        return true;
    }

private:
    uint32_t step{};
    uint16_t aux0{};
    uint16_t aux1{};
};


