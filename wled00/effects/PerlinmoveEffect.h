#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

// WLED-SR effects

/////////////////////////
//     Perlin Move     //
/////////////////////////
// 16 bit perlinmove. Use Perlin Noise instead of sinewaves for movement. By Andrew Tuline.
// Controls are speed, # of pixels, faderate.
class PerlinmoveEffect : public BaseEffect<PerlinmoveEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = PerlinmoveEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Perlin Move@!,# of pixels,Fade rate;!,!;!";
    static constexpr const uint8_t effectId = FX_MODE_PERLINMOVE;

    explicit PerlinmoveEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        buffer.fadeOut(255-parameters.custom1);
        for (int i = 0; i < parameters.intensity/16 + 1; i++) {
            unsigned locn = inoise16(strip.now*128/(260-parameters.speed)+i*15000, strip.now*128/(260-parameters.speed)); // Get a new pixel location from moving noise.
            unsigned pixloc = map(locn, 50*256, 192*256, 0, coordinate.width-1);                                            // Map that to the length of the strand, and ensure we don't go over.
            buffer.setPixelColor(pixloc, SEGMENT.color_from_palette(pixloc%255, false, PALETTE_SOLID_WRAP, 0));
        }
        return true;
    }

private:
};


