#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//eight colored dots, weaving in and out of sync with each other
class JuggleEffect : public BaseEffect<JuggleEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = JuggleEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char* const metaData = "Juggle@!,Trail;;!;;sx=64,ix=128";
    static constexpr const uint8_t effectId = FX_MODE_JUGGLE;

    explicit JuggleEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        buffer.fadeToBlackBy(192 - (3*SEGMENT.intensity/4));
        CRGB fastled_col;
        byte dothue = 0;
        for (int i = 0; i < 8; i++) {
            int index = 0 + beatsin88_t((16 + SEGMENT.speed)*(i + 7), 0, coordinate.width -1);
            fastled_col = CRGB(buffer.getPixelColor(index));
            fastled_col |= (SEGMENT.palette==0)?CHSV(dothue, 220, 255):CRGB(ColorFromPalette(SEGPALETTE, dothue, 255));
            buffer.setPixelColor(index, RGBW32(fastled_col.r, fastled_col.g, fastled_col.b, 0));
            dothue += 32;
        }
    }

private:
};


