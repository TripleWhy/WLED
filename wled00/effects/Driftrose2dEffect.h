#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

////////////////////////////
//     2D Drift Rose      //
////////////////////////////
//// Drift Rose by stepko (c)2021 [https://editor.soulmatelights.com/gallery/1369-drift-rose-pattern], adapted by Blaz Kristan (AKA blazoncek) improved by @dedehai
class Driftrose2dEffect : public BaseEffect<Driftrose2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = Driftrose2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "Drift Rose@Fade,Blur,,,,Smear;;!;2;pal=11";
    static constexpr const uint8_t effectId = FX_MODE_2DDRIFTROSE;

    explicit Driftrose2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        const float CX = (cols-cols%2)/2.f - .5f;
        const float CY = (rows-rows%2)/2.f - .5f;
        const float L = min(cols, rows) / 2.f;

        buffer.fadeToBlackBy(32+(SEGMENT.speed>>3));
        for (size_t i = 1; i < 37; i++) {
            float angle = radians(i * 10);
            uint32_t x = (CX + (sin_t(angle) * (beatsin8_t(i, 0, L*2)-L))) * 255.f;
            uint32_t y = (CY + (cos_t(angle) * (beatsin8_t(i, 0, L*2)-L))) * 255.f;
            if(SEGMENT.palette == 0) buffer.wuPixel(x, y, CHSV(i * 10, 255, 255));
            else buffer.wuPixel(x, y, ColorFromPalette(SEGPALETTE, i * 10));
        }
        buffer.blur(SEGMENT.intensity >> 4, SEGMENT.check1);
    }

private:
};


