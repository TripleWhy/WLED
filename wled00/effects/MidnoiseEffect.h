#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//////////////////////
//   * MIDNOISE     //
//////////////////////
// Midnoise. By Andrew Tuline.
class MidnoiseEffect : public BaseEffect<MidnoiseEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = MidnoiseEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Midnoise@Fade rate,Max. length;!,!;!;1v;ix=128,m12=1,si=0";
    static constexpr const uint8_t effectId = FX_MODE_MIDNOISE;

    explicit MidnoiseEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        um_data_t *um_data = getAudioData();
        float   volumeSmth   = *(float*)  um_data->u_data[0];

        buffer.fadeOut(parameters.speed);
        buffer.fadeOut(parameters.speed);

        float tmpSound2 = volumeSmth * (float)parameters.intensity / 256.0;  // Too sensitive.
        tmpSound2 *= (float)parameters.intensity / 128.0;              // Reduce sensitivity/length.

        unsigned maxLen = mapf(tmpSound2, 0, 127, 0, coordinate.width/2);
        if (maxLen >coordinate.width/2) maxLen = coordinate.width/2;

        for (unsigned i=(coordinate.width/2-maxLen); i<(coordinate.width/2+maxLen); i++) {
            uint8_t index = perlin8(i*volumeSmth+xdist, ydist+i*volumeSmth);  // Get a value from the noise function. I'm using both x and y axis.
            buffer.setPixelColor(i, SEGMENT.color_from_palette(index, false, PALETTE_SOLID_WRAP, 0));
        }

        xdist=xdist+beatsin8_t(5,0,10);
        ydist=ydist+beatsin8_t(4,0,10);
        return true;
    }

private:
    uint16_t xdist{};
    uint16_t ydist{};
};


