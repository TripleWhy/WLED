#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//////////////////////////////
//     2D Frizzles          //
//////////////////////////////
// By: Stepko https://editor.soulmatelights.com/gallery/640-color-frizzles , Modified by: Andrew Tuline
class Frizzles2dEffect : public BaseEffect<Frizzles2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = Frizzles2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "Frizzles@X frequency,Y frequency,Blur,,,Smear;;!;2";
    static constexpr const uint8_t effectId = FX_MODE_2DFRIZZLES;

    explicit Frizzles2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }


        const int cols = coordinate.width;
        const int rows = coordinate.height;

        buffer.fadeToBlackBy(16 + parameters.check1 * 10);
        for (size_t i = 8; i > 0; i--) {
            buffer.addPixelColor(beatsin8_t(parameters.speed/8 + i, 0, cols - 1),
                                                            beatsin8_t(parameters.intensity/8 - i, 0, rows - 1),
                                                            SEGPALETTE.ColorFromPalette(beatsin8_t(12, 0, 255), 255, LINEARBLEND));
        }
        buffer.blur(parameters.custom1 >> (3 + parameters.check1), parameters.check1);
        return true;
    }

private:
};


#endif //WLED_DISABLE_2D
