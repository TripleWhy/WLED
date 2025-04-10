#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Firing comets from one end. "Lighthouse"
 */
class CometEffect : public BaseEffect<CometEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = CometEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Lighthouse@!,Fade rate;!,!;!";
    static constexpr const uint8_t effectId = FX_MODE_COMET;

    explicit CometEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        unsigned counter = (strip.now * ((SEGMENT.speed >>2) +1)) & 0xFFFF;
        unsigned index = (counter * coordinate.width) >> 16;
        if (SEGENV.call == 0) aux0 = index;

        buffer.fadeOut(SEGMENT.intensity);

        buffer.setPixelColor( index, SEGMENT.color_from_palette(index, true, PALETTE_SOLID_WRAP, 0));
        if (index > aux0) {
            for (unsigned i = aux0; i < index ; i++) {
                 buffer.setPixelColor( i, SEGMENT.color_from_palette(i, true, PALETTE_SOLID_WRAP, 0));
            }
        } else if (index < aux0 && index < 10) {
            for (unsigned i = 0; i < index ; i++) {
                 buffer.setPixelColor( i, SEGMENT.color_from_palette(i, true, PALETTE_SOLID_WRAP, 0));
            }
        }
        aux0 = index++;
    }

private:
    uint16_t aux0{};
};


