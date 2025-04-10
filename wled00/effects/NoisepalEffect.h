#pragma once

#include <array>
#include "../FX.h"
#include "Effect.h"

// Peaceful noise that's slow and with gradually changing palettes. Does not support WLED palettes or default colours or controls.
// Slow noise palette by Andrew Tuline.
class NoisepalEffect : public BaseEffect<NoisepalEffect> {
private:
    using Self = NoisepalEffect;
    using Base = BaseEffect<Self>;

public:
    static constexpr const char metaData[] PROGMEM = "Noise Pal@!,Scale;;!";
    static constexpr const uint8_t effectId = FX_MODE_NOISEPAL;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d1;

    using Base::Base;

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        scale = 15 + (SEGMENT.intensity >> 2); //default was 30

        unsigned changePaletteMs = 4000 + SEGMENT.speed *10; //between 4 - 6.5sec
        if (strip.now - step > changePaletteMs)
        {
            step = strip.now;

            unsigned baseI = hw_random8();
            palettes[1] = CRGBPalette16(
                CHSV(baseI+hw_random8(64), 255, hw_random8(128,255)),
                CHSV(baseI+128, 255, hw_random8(128,255)),
                CHSV(baseI+hw_random8(92), 192, hw_random8(128,255)),
                CHSV(baseI+hw_random8(92), 255, hw_random8(128,255))
            );
        }

        nblendPaletteTowardPalette(palettes[0], palettes[1], 48);               // Blend towards the target palette over 48 iterations.

        if (SEGMENT.palette > 0)
            palettes[0] = SEGPALETTE;

        // In the original effect, aux0 was 0 in the first frame, here it is incremented one frame earlier.
        aux0 += beatsin8_t(10,1,4);                                        // Moving along the distance. Vary it a bit with a sine wave.
    }

    uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        const unsigned i = coordinate.getXAbsolute();
        const unsigned index = inoise8(i*scale, aux0+i*scale);                // Get a value from the noise function. I'm using both x and y axis.
        return ColorFromPalette(palettes[0], index, 255, LINEARBLEND);  // Use my own palette.
    }

private:
    std::array<CRGBPalette16, 2> palettes{};
    unsigned scale{};
    uint32_t step{};
    uint16_t aux0{};
};


