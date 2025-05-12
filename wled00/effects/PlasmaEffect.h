#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
/ Plasma Effect
/ adapted from https://github.com/atuline/FastLED-Demos/blob/master/plasma/plasma.ino
*/
class PlasmaEffect : public BaseEffect<PlasmaEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = PlasmaEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Plasma@Phase,!;!;!";
    static constexpr const uint8_t effectId = FX_MODE_PLASMA;

    explicit PlasmaEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        // initialize phases on start
        if (parameters.call == 0) {
            aux0 = hw_random8(0,2);  // add a bit of randomness
        }
        unsigned thisPhase = beatsin8_t(6+aux0,-64,64);
        unsigned thatPhase = beatsin8_t(7+aux0,-64,64);

        for (unsigned i = 0; i < coordinate.width; i++) {   // For each of the LED's in the strand, set color &  brightness based on a wave as follows:
            unsigned colorIndex = cubicwave8((i*(2+ 3*(parameters.speed >> 5))+thisPhase) & 0xFF)/2   // factor=23 // Create a wave and add a phase change and add another wave with its own phase change.
                                                                + cos8_t((i*(1+ 2*(parameters.speed >> 5))+thatPhase) & 0xFF)/2;  // factor=15 // Hey, you can even change the frequencies if you wish.
            unsigned thisBright = qsub8(colorIndex, beatsin8_t(7,0, (128 - (parameters.intensity>>1))));
            buffer.setPixelColor(i, parameters.color_from_palette(colorIndex, false, PALETTE_SOLID_WRAP, 0, thisBright));
        }
        return true;
    }

private:
    uint16_t aux0{};
};


