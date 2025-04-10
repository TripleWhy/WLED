#pragma once

#include "../FX.h"
#include "Effect.h"
#include "SpotsEffectBase.h"

//Intensity slider sets number of "lights", LEDs per light fade in and out
class SpotsFadeEffect : public BaseEffect<SpotsFadeEffect, SpotsEffectBase> {
private:
    using Self = SpotsFadeEffect;
    using Base = BaseEffect<Self, SpotsEffectBase>;

public:
    static constexpr const char metaData[] PROGMEM = "Spots Fade@Spread,Width,,,,,Overlay;!,!;!";
    static constexpr const uint8_t effectId = FX_MODE_SPOTS_FADE;

    using Base::Base;

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        unsigned counter = strip.now * ((SEGMENT.speed >> 2) +8);
        unsigned t = triwave16(counter);
        unsigned tr = (t >> 1) + (t >> 2);
        Base::nextFrameImpl(coordinate, tr);
    }

private:
};


