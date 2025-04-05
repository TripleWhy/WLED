#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/////////////////////////
//    * 2D Waverly     //
/////////////////////////
// By: Stepko, https://editor.soulmatelights.com/gallery/652-wave , modified by Andrew Tuline
class Waverly2dEffect : public BaseEffect<Waverly2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = Waverly2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char* const metaData = "Waverly@Amplification,Sensitivity,,,,,Blur;;!;2v;ix=64,si=0";
    static constexpr const uint8_t effectId = FX_MODE_2DWAVERLY;

    explicit Waverly2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        um_data_t *um_data = getAudioData();
        float   volumeSmth  = *(float*)   um_data->u_data[0];

        buffer.fadeToBlackBy(SEGMENT.speed);

        long t = strip.now / 2;
        for (int i = 0; i < cols; i++) {
            unsigned thisVal = (1 + SEGMENT.intensity/64) * inoise8(i * 45 , t , t)/2;
            // use audio if available
            if (um_data) {
                thisVal /= 32; // reduce intensity of inoise8()
                thisVal *= volumeSmth;
            }
            int thisMax = map(thisVal, 0, 512, 0, rows);

            for (int j = 0; j < thisMax; j++) {
                buffer.addPixelColor(i, j, ColorFromPalette(SEGPALETTE, map(j, 0, thisMax, 250, 0), 255, LINEARBLEND));
                buffer.addPixelColor((cols - 1) - i, (rows - 1) - j, ColorFromPalette(SEGPALETTE, map(j, 0, thisMax, 250, 0), 255, LINEARBLEND));
            }
        }
        if (SEGMENT.check3) buffer.blur(16, cols*rows < 100);
    }

private:
};

#endif // WLED_DISABLE_2D
