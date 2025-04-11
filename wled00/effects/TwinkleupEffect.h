#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

class TwinkleupEffect : public BaseEffect<TwinkleupEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = TwinkleupEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Twinkleup@!,Intensity;!,!;!;;m12=0";
    static constexpr const uint8_t effectId = FX_MODE_TWINKLEUP;

    explicit TwinkleupEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }
                                     // A very short twinkle routine with fade-in and dual controls. By Andrew Tuline.
        unsigned prevSeed = prng.getSeed();      // save seed so we can restore it at the end of the function
        prng.setSeed(535);                       // The randomizer needs to be re-set each time through the loop in order for the same 'random' numbers to be the same each time through.

        for (unsigned i = 0; i < coordinate.width; i++) {
            unsigned ranstart = prng.random8();               // The starting value (aka brightness) for each pixel. Must be consistent each time through the loop for this to work.
            unsigned pixBri = sin8_t(ranstart + 16 * strip.now/(256-SEGMENT.speed));
            if (prng.random8() > SEGMENT.intensity) pixBri = 0;
            buffer.setPixelColor(i, color_blend(SEGCOLOR(1), SEGMENT.color_from_palette(prng.random8()+strip.now/100, false, PALETTE_SOLID_WRAP, 0), pixBri));
        }

        prng.setSeed(prevSeed); // restore original seed so other effects can use "random" PRNG
        return true;
    }

private:
};


