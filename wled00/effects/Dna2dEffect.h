#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/////////////////////
//      2D DNA     //
/////////////////////
// dna originally by by ldirko at https://pastebin.com/pCkkkzcs. Updated by Preyy. WLED conversion by Andrew Tuline.
class Dna2dEffect : public BaseEffect<Dna2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = Dna2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "DNA@Scroll speed,Blur,,,,Smear;;!;2;ix=0";
    static constexpr const uint8_t effectId = FX_MODE_2DDNA;

    explicit Dna2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        buffer.fadeToBlackBy(64);
        for (int i = 0; i < cols; i++) {
            buffer.setPixelColor(i, beatsin8_t(SEGMENT.speed/8, 0, rows-1, 0, i*4    ), ColorFromPalette(SEGPALETTE, i*5+strip.now/17, beatsin8_t(5, 55, 255, 0, i*10), LINEARBLEND));
            buffer.setPixelColor(i, beatsin8_t(SEGMENT.speed/8, 0, rows-1, 0, i*4+128), ColorFromPalette(SEGPALETTE, i*5+128+strip.now/17, beatsin8_t(5, 55, 255, 0, i*10+128), LINEARBLEND));
        }
        buffer.blur(SEGMENT.intensity / (8 - (SEGMENT.check1 * 2)), SEGMENT.check1);
        return true;
    }

private:
};


#endif //WLED_DISABLE_2D
