#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//////////////////////
//   * MATRIPIX     //
//////////////////////
// Matripix. By Andrew Tuline.
// effect can work on single pixels, we just lose the shifting effect
class MatripixEffect : public BaseEffect<MatripixEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = MatripixEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Matripix@!,Brightness;!,!;!;1v;ix=64,m12=2,si=1";
    static constexpr const uint8_t effectId = FX_MODE_MATRIPIX;

    explicit MatripixEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        um_data_t *um_data = getAudioData();
        int volumeRaw    = *(int16_t*)um_data->u_data[1];

        uint8_t secondHand = micros()/(256-SEGMENT.speed)/500 % 16;
        if(aux0 != secondHand) {
            aux0 = secondHand;

            int pixBri = volumeRaw * SEGMENT.intensity / 64;
            unsigned k = coordinate.width-1;
            // loop will not execute if coordinate.width equals 1
            for (unsigned i = 0; i < k; i++) {
                buffer.setPixelColor(i, buffer.getPixelColor(i+1)); // shift left
            }
            buffer.setPixelColor(k, color_blend(SEGCOLOR(1), SEGMENT.color_from_palette(strip.now, false, PALETTE_SOLID_WRAP, 0), pixBri));
        }
        return true;
    }

private:
    uint16_t aux0{};
};


