#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

///////////////////////
//   * Noisemeter    //
///////////////////////
class NoisemeterEffect : public BaseEffect<NoisemeterEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = NoisemeterEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Noisemeter@Fade rate,Width;!,!;!;1v;ix=128,m12=2,si=0";
    static constexpr const uint8_t effectId = FX_MODE_NOISEMETER;

    explicit NoisemeterEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);
                                    // Noisemeter. By Andrew Tuline.

        um_data_t *um_data = getAudioData();
        float   volumeSmth   = *(float*)  um_data->u_data[0];
        int volumeRaw    = *(int16_t*)um_data->u_data[1];

        //uint8_t fadeRate = map(SEGMENT.speed,0,255,224,255);
        uint8_t fadeRate = map(SEGMENT.speed,0,255,200,254);
        buffer.fadeOut(fadeRate);

        float tmpSound2 = volumeRaw * 2.0 * (float)SEGMENT.intensity / 255.0;
        unsigned maxLen = mapf(tmpSound2, 0, 255, 0, coordinate.width); // map to pixels availeable in current segment              // Still a bit too sensitive.
        if (maxLen < 0) maxLen = 0;
        if (maxLen > coordinate.width) maxLen = coordinate.width;

        for (unsigned i=0; i<maxLen; i++) {                                    // The louder the sound, the wider the soundbar. By Andrew Tuline.
            uint8_t index = inoise8(i*volumeSmth+aux0, aux1+i*volumeSmth);  // Get a value from the noise function. I'm using both x and y axis.
            buffer.setPixelColor(i, SEGMENT.color_from_palette(index, false, PALETTE_SOLID_WRAP, 0));
        }

        aux0+=beatsin8_t(5,0,10);
        aux1+=beatsin8_t(4,0,10);
    }

private:
    uint16_t aux0{};
    uint16_t aux1{};
};


