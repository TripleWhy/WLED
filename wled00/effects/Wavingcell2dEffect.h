#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//Waving Cell
//@Stepko (https://editor.soulmatelights.com/gallery/1704-wavingcells)
// adapted for WLED by @blazoncek, improvements by @dedehai
class Wavingcell2dEffect : public BaseEffect<Wavingcell2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = Wavingcell2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "Waving Cell@!,Blur,Amplitude 1,Amplitude 2,Amplitude 3,,Flow;;!;2;ix=0";
    static constexpr const uint8_t effectId = FX_MODE_2DWAVINGCELL;

    explicit Wavingcell2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        uint32_t t = (strip.now*(SEGMENT.speed + 1))>>3;
        uint32_t aX = SEGMENT.custom1/16 + 9;
        uint32_t aY = SEGMENT.custom2/16 + 1;
        uint32_t aZ = SEGMENT.custom3 + 1;
         for (int x = 0; x < cols; x++) {
            for (int y = 0; y < rows; y++) {
                uint32_t wave = sin8_t((x * aX) + sin8_t((((y<<8) + t) * aY)>>8)) + cos8_t(y * aZ); // bit shifts to increase temporal resolution
                uint8_t colorIndex = wave + (t>>(8-(SEGMENT.check2*3)));
                buffer.setPixelColor(x, y, ColorFromPalette(SEGPALETTE, colorIndex));
            }
        }
        buffer.blur(SEGMENT.intensity);
        return true;
    }

private:
};


#endif //WLED_DISABLE_2D
