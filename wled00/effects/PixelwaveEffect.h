#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//////////////////////
//   * PIXELWAVE    //
//////////////////////
// Pixelwave. By Andrew Tuline.
class PixelwaveEffect : public BaseEffect<PixelwaveEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = PixelwaveEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Pixelwave@!,Sensitivity;!,!;!;1v;ix=64,m12=2,si=0";
    static constexpr const uint8_t effectId = FX_MODE_PIXELWAVE;

    explicit PixelwaveEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        if (SEGENV.call == 0) {
            buffer.fill(BLACK);
        }

        um_data_t *um_data = getAudioData();
        int volumeRaw    = *(int16_t*)um_data->u_data[1];

        uint8_t secondHand = micros()/(256-SEGMENT.speed)/500+1 % 16;
        if (aux0 != secondHand) {
            aux0 = secondHand;

            uint8_t pixBri = volumeRaw * SEGMENT.intensity / 64;

            buffer.setPixelColor(coordinate.width/2, color_blend(SEGCOLOR(1), SEGMENT.color_from_palette(strip.now, false, PALETTE_SOLID_WRAP, 0), pixBri));
            for (unsigned i = coordinate.width - 1; i > coordinate.width/2; i--) buffer.setPixelColor(i, buffer.getPixelColor(i-1)); //move to the left
            for (unsigned i = 0; i < coordinate.width/2; i++)          buffer.setPixelColor(i, buffer.getPixelColor(i+1)); // move to the right
        }
        return true;
    }

private:
    uint16_t aux0{};
};


