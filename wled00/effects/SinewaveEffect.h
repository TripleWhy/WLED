#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

// Sine waves that have controllable phase change speed, frequency and cutoff. By Andrew Tuline.
// parameters.speed ->Speed, parameters.intensity -> Frequency (SEGMENT.fft1 -> Color change, SEGMENT.fft2 -> PWM cutoff)
//
// Adjustable sinewave. By Andrew Tuline
class SinewaveEffect : public BaseEffect<SinewaveEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = SinewaveEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Sine@!,Scale;;!";
    static constexpr const uint8_t effectId = FX_MODE_SINEWAVE;

    explicit SinewaveEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        unsigned colorIndex = strip.now /32;//(256 - SEGMENT.fft1);  // Amount of colour change.

        step += parameters.speed/16;                   // Speed of animation.
        unsigned freq = parameters.intensity/4;//SEGMENT.fft2/8;                       // Frequency of the signal.

        for (unsigned i = 0; i < coordinate.width; i++) {                 // For each of the LED's in the strand, set a brightness based on a wave as follows:
            uint8_t pixBri = cubicwave8((i*freq)+step);//qsuba(cubicwave8((i*freq)+step), (255-parameters.intensity)); // qsub sets a minimum value called thiscutoff. If < thiscutoff, then bright = 0. Otherwise, bright = 128 (as defined in qsub)..
            //setPixCol(i, i*colorIndex/255, pixBri);
            buffer.setPixelColor(i, color_blend(SEGCOLOR(1), parameters.color_from_palette(i*colorIndex/255, false, PALETTE_SOLID_WRAP, 0), pixBri));
        }
        return true;
    }

private:
    uint32_t step{};
};


