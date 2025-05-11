#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//Glitter with palette background, inspired by https://gist.github.com/kriegsman/062e10f7f07ba8518af6
class GlitterEffect : public BaseEffect<GlitterEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = GlitterEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Glitter@!,!,,,,,Overlay;,,Glitter color;!;;pal=11,m12=0";
    static constexpr const uint8_t effectId = FX_MODE_GLITTER;

    explicit GlitterEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        if (!parameters.check2) { // use "* Color 1" palette for solid background (replacing "Solid glitter")
            unsigned counter = 0;
            if (parameters.speed != 0) {
                // animate palette
                counter = (strip.now * ((parameters.speed >> 3) +1)) & 0xFFFF;
                counter = counter >> 8;
            }
            for (unsigned i = 0; i < coordinate.width; i++) {
                unsigned colorIndex = (i * 255 / coordinate.width) - counter;
                buffer.setPixelColor(i, SEGMENT.color_from_palette(colorIndex, false, true, 255));
            }
        }
        if (parameters.intensity > hw_random8()) buffer.setPixelColor(hw_random16(coordinate.width), SEGCOLOR(2) ? SEGCOLOR(2) : ULTRAWHITE);
        return true;
    }

private:
};


