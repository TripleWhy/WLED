#pragma once

#include <array>
#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//////////////////////
//     * PIXELS     //
//////////////////////
// Pixels. By Andrew Tuline.
class PixelsEffect : public BaseEffect<PixelsEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = PixelsEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char* const metaData = "Pixels@Fade rate,# of pixels;!,!;!;1v;m12=0,si=0";
    static constexpr const uint8_t effectId = FX_MODE_PIXELS;

    explicit PixelsEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        um_data_t *um_data;
        if (!UsermodManager::getUMData(&um_data, USERMOD_ID_AUDIOREACTIVE)) {
            um_data = simulateSound(SEGMENT.soundSim);
        }
        float   volumeSmth   = *(float*)  um_data->u_data[0];

        myVals[strip.now%32] = volumeSmth;    // filling values semi randomly

        buffer.fadeOut(64+(SEGMENT.speed>>1));

        for (int i=0; i <SEGMENT.intensity/8; i++) {
            unsigned segLoc = hw_random16(coordinate.width);                    // 16 bit for larger strands of LED's.
            buffer.setPixelColor(segLoc, color_blend(SEGCOLOR(1), SEGMENT.color_from_palette(myVals[i%32]+i*4, false, PALETTE_SOLID_WRAP, 0), uint8_t(volumeSmth)));
        }
    }

private:
    // Used to store a pile of samples because WLED frame rate and WLED sample rate are not synchronized. Frame rate is too low.
    std::array<uint8_t, 32> myVals{};
};


