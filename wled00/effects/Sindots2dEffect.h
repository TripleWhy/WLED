#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/////////////////////////
//     2D Sindots      //
/////////////////////////
// By: ldirko   https://editor.soulmatelights.com/gallery/597-sin-dots , modified by: Andrew Tuline
class Sindots2dEffect : public BaseEffect<Sindots2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = Sindots2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char* const metaData = "Sindots@!,Dot distance,Fade rate,Blur,,Smear;;!;2;";
    static constexpr const uint8_t effectId = FX_MODE_2DSINDOTS;

    explicit Sindots2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        if (SEGENV.call == 0) {
            buffer.fill(BLACK);
        }

        buffer.fadeToBlackBy((SEGMENT.custom1>>3) + (SEGMENT.check1 * 24));

        byte t1 = strip.now / (257 - SEGMENT.speed); // 20;
        byte t2 = sin8_t(t1) / 4 * 2;
        for (int i = 0; i < 13; i++) {
            int x = sin8_t(t1 + i * SEGMENT.intensity/8)*(cols-1)/255;  // max index now 255x15/255=15!
            int y = sin8_t(t2 + i * SEGMENT.intensity/8)*(rows-1)/255;  // max index now 255x15/255=15!
            buffer.setPixelColor(x, y, ColorFromPalette(SEGPALETTE, i * 255 / 13, 255, LINEARBLEND));
        }
        buffer.blur(SEGMENT.custom2 >> (3 + SEGMENT.check1), SEGMENT.check1);
    }

private:
};


#endif //WLED_DISABLE_2D
