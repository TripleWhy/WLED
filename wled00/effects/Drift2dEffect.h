#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/////////////////////////
//     2D Drift        //
/////////////////////////
// By: Stepko   https://editor.soulmatelights.com/gallery/884-drift , Modified by: Andrew Tuline
class Drift2dEffect : public BaseEffect<Drift2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = Drift2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "Drift@Rotation speed,Blur,,,,Twin,Smear;;!;2;ix=0";
    static constexpr const uint8_t effectId = FX_MODE_2DDRIFT;

    explicit Drift2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        const int colsCenter = (cols>>1) + (cols%2);
        const int rowsCenter = (rows>>1) + (rows%2);

        buffer.fadeToBlackBy(128);
        const float maxDim = MAX(cols, rows)/2;
        unsigned long t = strip.now / (32 - (parameters.speed>>3));
        unsigned long t_20 = t/20; // softhack007: pre-calculating this gives about 10% speedup
        for (float i = 1.0f; i < maxDim; i += 0.25f) {
            float angle = radians(t * (maxDim - i));
            int mySin = sin_t(angle) * i;
            int myCos = cos_t(angle) * i;
            buffer.setPixelColor(colsCenter + mySin, rowsCenter + myCos, ColorFromPalette(SEGPALETTE, (i * 20) + t_20, 255, LINEARBLEND));
            if (parameters.check1) buffer.setPixelColor(colsCenter + myCos, rowsCenter + mySin, ColorFromPalette(SEGPALETTE, (i * 20) + t_20, 255, LINEARBLEND));
        }
        buffer.blur(parameters.intensity>>(3 - parameters.check2), parameters.check2);
        return true;
    }

private:
};


#endif //WLED_DISABLE_2D
