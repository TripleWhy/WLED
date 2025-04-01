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
    static constexpr const char* const metaData = "Frizzles@X frequency,Y frequency,Blur,,,Smear;;!;2";
    static constexpr const uint8_t effectId = FX_MODE_2DFRIZZLES;

    explicit Frizzles2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);


        const int cols = coordinate.width;
        const int rows = coordinate.height;

        buffer.fadeToBlackBy(16 + SEGMENT.check1 * 10);
        for (size_t i = 8; i > 0; i--) {
            buffer.addPixelColor(beatsin8_t(SEGMENT.speed/8 + i, 0, cols - 1),
                                                            beatsin8_t(SEGMENT.intensity/8 - i, 0, rows - 1),
                                                            ColorFromPalette(SEGPALETTE, beatsin8_t(12, 0, 255), 255, LINEARBLEND));
        }
        buffer.blur(SEGMENT.custom1 >> (3 + SEGMENT.check1), SEGMENT.check1);
    }

private:
};


#endif //WLED_DISABLE_2D
