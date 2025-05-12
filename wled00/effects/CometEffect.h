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

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        unsigned counter = (strip.now * ((parameters.speed >>2) +1)) & 0xFFFF;
        unsigned index = (counter * coordinate.width) >> 16;
        if (parameters.call == 0) aux0 = index;

        buffer.fade(SEGCOLOR(1), parameters.intensity);

        buffer.setPixelColor( index, parameters.color_from_palette(index, true, PALETTE_SOLID_WRAP, 0));
        if (index > aux0) {
            for (unsigned i = aux0; i < index ; i++) {
                 buffer.setPixelColor( i, parameters.color_from_palette(i, true, PALETTE_SOLID_WRAP, 0));
            }
        } else if (index < aux0 && index < 10) {
            for (unsigned i = 0; i < index ; i++) {
                 buffer.setPixelColor( i, parameters.color_from_palette(i, true, PALETTE_SOLID_WRAP, 0));
            }
        }
        aux0 = index++;
        return true;
    }

private:
    uint16_t aux0{};
};


