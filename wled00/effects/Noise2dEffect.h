#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//////////////////////
//    2D Noise      //
//////////////////////
// By Andrew Tuline
class Noise2dEffect : public BaseEffect<Noise2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = Noise2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "Noise2D@!,Scale;;!;2";
    static constexpr const uint8_t effectId = FX_MODE_2DNOISE;

    explicit Noise2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        const unsigned scale  = SEGMENT.intensity+2;

        for (int y = 0; y < rows; y++) {
            for (int x = 0; x < cols; x++) {
                uint8_t pixelHue8 = inoise8(x * scale, y * scale, strip.now / (16 - SEGMENT.speed/16));
                buffer.setPixelColor(x, y, ColorFromPalette(SEGPALETTE, pixelHue8));
            }
        }
    }

private:
};


#endif //WLED_DISABLE_2D
