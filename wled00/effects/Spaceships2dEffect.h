#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/////////////////////////
//     2D spaceships   //
/////////////////////////
//// Space ships by stepko (c)05.02.21 [https://editor.soulmatelights.com/gallery/639-space-ships], adapted by Blaz Kristan (AKA blazoncek)
class Spaceships2dEffect : public BaseEffect<Spaceships2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = Spaceships2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "Spaceships@!,Blur,,,,Smear;;!;2";
    static constexpr const uint8_t effectId = FX_MODE_2DSPACESHIPS;

    explicit Spaceships2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        uint32_t tb = strip.now >> 12;  // every ~4s
        if (tb > step) {
            int dir = ++aux0;
            dir  += (int)hw_random8(3)-1;
            if      (dir > 7) aux0 = 0;
            else if (dir < 0) aux0 = 7;
            else              aux0 = dir;
            step = tb + hw_random8(4);
        }

        buffer.fadeToBlackBy(map(SEGMENT.speed, 0, 255, 248, 16));
        buffer.movePixels(aux0, 1);

        for (size_t i = 0; i < 8; i++) {
            int x = beatsin8_t(12 + i, 2, cols - 3);
            int y = beatsin8_t(15 + i, 2, rows - 3);
            uint32_t color = ColorFromPalette(SEGPALETTE, beatsin8_t(12 + i, 0, 255), 255);
            buffer.addPixelColor(x, y, color);
            if (cols > 24 || rows > 24) {
                buffer.addPixelColor(x+1, y, color);
                buffer.addPixelColor(x-1, y, color);
                buffer.addPixelColor(x, y+1, color);
                buffer.addPixelColor(x, y-1, color);
            }
        }
        buffer.blur(SEGMENT.intensity >> 3, SEGMENT.check1);
    }

private:
    uint32_t step{};
    uint16_t aux0{};
};


#endif //WLED_DISABLE_2D
