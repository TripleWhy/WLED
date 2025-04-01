#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"
#include "effectUtils.h"

//////////////////////
//   ** Noisemove   //
//////////////////////
class NoisemoveEffect : public BaseEffect<NoisemoveEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = NoisemoveEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char* const metaData = "Noisemove@Move speed,Fade rate;!,!;!;01f;m12=0,si=0";
    static constexpr const uint8_t effectId = FX_MODE_NOISEMOVE;

    explicit NoisemoveEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);
                                     // Noisemove.    By: Andrew Tuline
        um_data_t *um_data = getAudioData();
        uint8_t *fftResult = (uint8_t*)um_data->u_data[2];

        int fadeoutDelay = (256 - SEGMENT.speed) / 96;
        if ((fadeoutDelay <= 1 ) || ((SEGENV.call % fadeoutDelay) == 0)) buffer.fadeToBlackBy(4+ SEGMENT.speed/4);

        uint8_t numBins = map(SEGMENT.intensity,0,255,0,16);    // Map slider to fftResult bins.
        for (int i=0; i<numBins; i++) {                         // How many active bins are we using.
            unsigned locn = inoise16(strip.now*SEGMENT.speed+i*50000, strip.now*SEGMENT.speed);   // Get a new pixel location from moving noise.
            // if coordinate.width equals 1 locn will be always 0, hence we set the first pixel only
            locn = map(locn, 7500, 58000, 0, coordinate.width-1);           // Map that to the length of the strand, and ensure we don't go over.
            buffer.setPixelColor(locn, color_blend(SEGCOLOR(1), SEGMENT.color_from_palette(i*64, false, PALETTE_SOLID_WRAP, 0), uint8_t(fftResult[i % 16]*4)));
        }
    }

private:
};


