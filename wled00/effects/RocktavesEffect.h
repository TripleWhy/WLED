#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//////////////////////
//   ** Rocktaves   //
//////////////////////
class RocktavesEffect : public BaseEffect<RocktavesEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = RocktavesEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Rocktaves@;!,!;!;01f;m12=1,si=0";
    static constexpr const uint8_t effectId = FX_MODE_ROCKTAVES;

    explicit RocktavesEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);
                                     // Rocktaves. Same note from each octave is same colour.    By: Andrew Tuline
        um_data_t *um_data = getAudioData();
        float   FFT_MajorPeak = *(float*)  um_data->u_data[4];
        float   my_magnitude  = *(float*)   um_data->u_data[5] / 16.0f;

        buffer.fadeToBlackBy(16);                              // Just in case something doesn't get faded.

        float frTemp = FFT_MajorPeak;
        uint8_t octCount = 0;                                   // Octave counter.
        uint8_t volTemp = 0;

        volTemp = 32.0f + my_magnitude * 1.5f;                  // brightness = volume (overflows are handled in next lines)
        if (my_magnitude < 48) volTemp = 0;                     // We need to squelch out the background noise.
        if (my_magnitude > 144) volTemp = 255;                  // everything above this is full brightness

        while ( frTemp > 249 ) {
            octCount++;                                           // This should go up to 5.
            frTemp = frTemp/2;
        }

        frTemp -= 132.0f;                                       // This should give us a base musical note of C3
        frTemp  = fabsf(frTemp * 2.1f);                         // Fudge factors to compress octave range starting at 0 and going to 255;

        unsigned i = map(beatsin8_t(8+octCount*4, 0, 255, 0, octCount*8), 0, 255, 0, coordinate.width-1);
        i = constrain(i, 0U, coordinate.width-1U);
        buffer.addPixelColor(i, color_blend(SEGCOLOR(1), SEGMENT.color_from_palette((uint8_t)frTemp, false, PALETTE_SOLID_WRAP, 0), volTemp));
    }

private:
};


