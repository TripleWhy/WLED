#pragma once

#include "../FX.h"
#include "Effect.h"

/*
 * Effects by Andrew Tuline
 */
// We're making sine waves here. By Andrew Tuline.
class PhasedEffectBase : public Effect {
private:
    using Self = PhasedEffectBase;
    using Base = Effect;

public:
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d1;

    PhasedEffectBase(const EffectInformation& ei, bool moder)
        : Base{ei},
          moder{moder}
    {
    }

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        allfreq = 16;                                          // Base frequency.
        cutOff = (255-parameters.intensity);                      // You can change the number of pixels.  AKA INTENSITY (was 192).
        modVal = 5;//SEGMENT.fft1/8+1;                         // You can change the modulus. AKA FFT1 (was 5).

        index = strip.now/64;                                  // Set color rotation speed
        phase += parameters.speed/32.0;                           // You can change the speed of the wave. AKA SPEED (was .4)
        return true;
    }

    uint32_t getPixelColorImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        unsigned i = coordinate.getXAbsolute();
        if (moder)
            modVal = (perlin8(i*10 + i*10) /16);                     // Let's randomize our mod length with some Perlin noise.
        unsigned val = (i+1) * allfreq;                              // This sets the frequency of the waves. The +1 makes sure that led 0 is used.
        if (modVal == 0)
            modVal = 1;
        val += phase * (i % modVal +1) /2;                           // This sets the varying phase change of the waves. By Andrew Tuline.
        unsigned b = cubicwave8(val);                                // Now we make an 8 bit sinewave.
        b = (b > cutOff) ? (b - cutOff) : 0;                         // A ternary operator to cutoff the light.
        const uint32_t color = color_blend(SEGCOLOR(1), parameters.color_from_palette(index, false, false, 0), uint8_t(b));
        index += 256 / coordinate.width;
        if (coordinate.width > 256)
            index ++;                                                // Correction for segments longer than 256 LEDs
        return color;
    }

private:
    const bool moder;
    float phase{};

    unsigned allfreq{};
    unsigned cutOff{};
    unsigned modVal{};
    unsigned index{};
};
