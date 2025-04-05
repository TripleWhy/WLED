#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//////////////////////
//   * PLASMOID     //
//////////////////////
// Plasmoid. By Andrew Tuline.
class PlasmoidEffect : public BaseEffect<PlasmoidEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = PlasmoidEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char* const metaData = "Plasmoid@Phase,# of pixels;!,!;!;01v;sx=128,ix=128,m12=0,si=0";
    static constexpr const uint8_t effectId = FX_MODE_PLASMOID;

    explicit PlasmoidEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        um_data_t *um_data = getAudioData();
        float   volumeSmth   = *(float*)  um_data->u_data[0];

        buffer.fadeToBlackBy(32);

        thisphase += beatsin8_t(6,-4,4);                          // You can change direction and speed individually.
        thatphase += beatsin8_t(7,-4,4);                          // Two phase values to make a complex pattern. By Andrew Tuline.

        for (unsigned i = 0; i < coordinate.width; i++) {                          // For each of the LED's in the strand, set a brightness based on a wave as follows.
            // updated, similar to "plasma" effect - softhack007
            uint8_t thisbright = cubicwave8(((i*(1 + (3*SEGMENT.speed/32)))+thisphase) & 0xFF)/2;
            thisbright += cos8_t(((i*(97 +(5*SEGMENT.speed/32)))+thatphase) & 0xFF)/2; // Let's munge the brightness a bit and animate it all with the phases.

            uint8_t colorIndex=thisbright;
            if (volumeSmth * SEGMENT.intensity / 64 < thisbright) {
                thisbright = 0;
            }

            buffer.addPixelColor(i, color_blend(SEGCOLOR(1), SEGMENT.color_from_palette(colorIndex, false, PALETTE_SOLID_WRAP, 0), thisbright));
        }
    }

private:
    int16_t thisphase{};
    int16_t thatphase{};
};
