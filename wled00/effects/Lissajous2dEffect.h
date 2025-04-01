#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//////////////////////////////
//     2D Lissajous         //
//////////////////////////////
// By: Andrew Tuline
class Lissajous2dEffect : public BaseEffect<Lissajous2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = Lissajous2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char* const metaData = "Lissajous@X frequency,Fade rate,Blur,,Speed,Smear;!;!;2;c1=0";
    static constexpr const uint8_t effectId = FX_MODE_2DLISSAJOUS;

    explicit Lissajous2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        buffer.fadeToBlackBy(SEGMENT.intensity);
        uint_fast16_t phase = (strip.now * (1 + SEGENV.custom3)) /32;  // allow user to control rotation speed

        //for (int i=0; i < 4*(cols+rows); i ++) {
        for (int i=0; i < 256; i ++) {
            //float xlocn = float(sin8_t(now/4+i*(SEGMENT.speed>>5))) / 255.0f;
            //float ylocn = float(cos8_t(now/4+i*2)) / 255.0f;
            uint_fast8_t xlocn = sin8_t(phase/2 + (i*SEGMENT.speed)/32);
            uint_fast8_t ylocn = cos8_t(phase/2 + i*2);
            xlocn = (cols < 2) ? 1 : (map(2*xlocn, 0,511, 0,2*(cols-1)) +1) /2;    // softhack007: "(2* ..... +1) /2" for proper rounding
            ylocn = (rows < 2) ? 1 : (map(2*ylocn, 0,511, 0,2*(rows-1)) +1) /2;    // "rows > 1" is needed to avoid div/0 in map()
            buffer.setPixelColor((uint8_t)xlocn, (uint8_t)ylocn, SEGMENT.color_from_palette(strip.now/100+i, false, PALETTE_SOLID_WRAP, 0));
        }
        buffer.blur(SEGMENT.custom1 >> (1 + SEGMENT.check1 * 3), SEGMENT.check1);
    }

private:
};


#endif //WLED_DISABLE_2D
