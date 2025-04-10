#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/////////////////////////
//     2D Pulser       //
/////////////////////////
// By: ldirko   https://editor.soulmatelights.com/gallery/878-pulse-test , modifed by: Andrew Tuline
class Pulser2dEffect : public BaseEffect<Pulser2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = Pulser2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "Pulser@!,Blur;;!;2";
    static constexpr const uint8_t effectId = FX_MODE_2DPULSER;

    explicit Pulser2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        buffer.fadeToBlackBy(8 - (SEGMENT.intensity>>5));
        uint32_t a = strip.now / (18 - SEGMENT.speed / 16);
        int x = (a / 14) % cols;
        int y = map((sin8_t(a * 5) + sin8_t(a * 4) + sin8_t(a * 2)), 0, 765, rows-1, 0);
        buffer.setPixelColor(x, y, ColorFromPalette(SEGPALETTE, map(y, 0, rows-1, 0, 255), 255, LINEARBLEND));

        buffer.blur(SEGMENT.intensity>>4);
    }

private:
};


#endif //WLED_DISABLE_2D
