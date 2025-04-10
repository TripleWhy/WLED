#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

// Black hole
// By: Stepko https://editor.soulmatelights.com/gallery/1012 , Modified by: Andrew Tuline
class BlackHole2dEffect : public BaseEffect<BlackHole2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = BlackHole2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "Black Hole@Fade rate,Outer Y freq.,Outer X freq.,Inner X freq.,Inner Y freq.,Solid,,Blur;!;!;2;pal=11";
    static constexpr const uint8_t effectId = FX_MODE_2DBLACKHOLE;

    explicit BlackHole2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        const int cols = coordinate.width;
        const int rows = coordinate.height;
        int x, y;
        const bool oneColor = SEGMENT.check1;

        buffer.fadeToBlackBy(16 + (SEGMENT.speed>>3)); // create fading trails
        unsigned long t = strip.now/128;                 // timebase
        // outer stars
        for (size_t i = 0; i < 8; i++) {
            x = beatsin8_t(SEGMENT.custom1>>3,   0, cols - 1, 0, ((i % 2) ? 128 : 0) + t * i);
            y = beatsin8_t(SEGMENT.intensity>>3, 0, rows - 1, 0, ((i % 2) ? 192 : 64) + t * i);
            buffer.addPixelColor(x, y, SEGMENT.color_from_palette(i*32, false, PALETTE_SOLID_WRAP, oneColor?0:255));
        }
        // inner stars
        for (size_t i = 0; i < 4; i++) {
            x = beatsin8_t(SEGMENT.custom2>>3, cols/4, cols - 1 - cols/4, 0, ((i % 2) ? 128 : 0) + t * i);
            y = beatsin8_t(SEGMENT.custom3   , rows/4, rows - 1 - rows/4, 0, ((i % 2) ? 192 : 64) + t * i);
            buffer.addPixelColor(x, y, SEGMENT.color_from_palette(255-i*64, false, PALETTE_SOLID_WRAP, oneColor?0:255));
        }
        // central white dot
        buffer.setPixelColor(cols/2, rows/2, WHITE);
        // blur everything a bit
        if (SEGMENT.check3)
            buffer.blur(16, cols*rows < 100);
    }

private:
};


#endif //WLED_DISABLE_2D
