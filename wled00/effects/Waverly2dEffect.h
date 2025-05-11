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
    static constexpr const char metaData[] PROGMEM = "Waverly@Amplification,Sensitivity,,,,,Blur;;!;2v;ix=64,si=0";
    static constexpr const uint8_t effectId = FX_MODE_2DWAVERLY;

    explicit Waverly2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        um_data_t *um_data = getAudioData();
        float   volumeSmth  = *(float*)   um_data->u_data[0];

        buffer.fadeToBlackBy(parameters.speed);

        long t = strip.now / 2;
        for (int i = 0; i < cols; i++) {
            unsigned thisVal = (1 + parameters.intensity/64) * perlin8(i * 45 , t , t)/2;
            // use audio if available
            if (um_data) {
                thisVal /= 32; // reduce intensity of perlin8()
                thisVal *= volumeSmth;
            }
            int thisMax = map(thisVal, 0, 512, 0, rows);

            for (int j = 0, jMax = std::min(thisMax, rows); j < jMax; j++) {
                buffer.addPixelColor(i, j, ColorFromPalette(SEGPALETTE, map(j, 0, thisMax, 250, 0), 255, LINEARBLEND));
                buffer.addPixelColor((cols - 1) - i, (rows - 1) - j, ColorFromPalette(SEGPALETTE, map(j, 0, thisMax, 250, 0), 255, LINEARBLEND));
            }
        }
        if (parameters.check3) buffer.blur(16, cols*rows < 100);
        return true;
    }

private:
};

#endif // WLED_DISABLE_2D
