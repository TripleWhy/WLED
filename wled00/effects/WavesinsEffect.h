#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/////////////////////////
//     Waveins         //
/////////////////////////
// Uses beatsin8() + phase shifting. By: Andrew Tuline
class WavesinsEffect : public BaseEffect<WavesinsEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = WavesinsEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Wavesins@!,Brightness variation,Starting color,Range of colors,Color variation;!;!";
    static constexpr const uint8_t effectId = FX_MODE_WAVESINS;

    explicit WavesinsEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }


        for (unsigned i = 0; i < coordinate.width; i++) {
            uint8_t bri = sin8_t(strip.now/4 + i * parameters.intensity);
            uint8_t index = beatsin8_t(parameters.speed, parameters.custom1, parameters.custom1+parameters.custom2, 0, i * (parameters.custom3<<3)); // custom3 is reduced resolution slider
            //buffer.setPixelColor(i, ColorFromPalette(SEGPALETTE, index, bri, LINEARBLEND));
            buffer.setPixelColor(i, SEGMENT.color_from_palette(index, false, true, 0, bri));
        }
        return true;
    }

private:
};


