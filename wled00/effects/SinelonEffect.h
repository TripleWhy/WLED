#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
* Sinelon stolen from FASTLED examples
*/
class SinelonEffect : public BaseEffect<SinelonEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = SinelonEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Sinelon@!,Trail,,,,Rainbow,Dual;!,!,!;!";
    static constexpr const uint8_t effectId = FX_MODE_SINELON;

    explicit SinelonEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        const bool rainbow = parameters.check1;
        const bool dual    = parameters.check2;
        buffer.fade(SEGCOLOR(1), parameters.intensity);
        unsigned pos = beatsin16_t(parameters.speed/10,0,coordinate.width-1);
        if (parameters.call == 0) aux0 = pos;
        uint32_t color1 = parameters.color_from_palette(pos, true, false, 0);
        uint32_t color2 = SEGCOLOR(2);
        if (rainbow) {
            color1 = parameters.color_wheel((pos & 0x07) * 32);
        }
        buffer.setPixelColor(pos, color1);
        if (dual) {
            if (!color2) color2 = parameters.color_from_palette(pos, true, false, 0);
            if (rainbow) color2 = color1; //rainbow
            buffer.setPixelColor(coordinate.width-1-pos, color2);
        }
        if (aux0 != pos) {
            if (aux0 < pos) {
                for (unsigned i = aux0; i < pos ; i++) {
                    buffer.setPixelColor(i, color1);
                    if (dual) buffer.setPixelColor(coordinate.width-1-i, color2);
                }
            } else {
                for (unsigned i = aux0; i > pos ; i--) {
                    buffer.setPixelColor(i, color1);
                    if (dual) buffer.setPixelColor(coordinate.width-1-i, color2);
                }
            }
            aux0 = pos;
        }
        return true;
    }

private:
    uint16_t aux0{};
};


