#pragma once

#include "../FX.h"
#include "Effect.h"
#include "SpotsEffectBase.h"

//Intensity slider sets number of "lights", speed sets LEDs per light
class SpotsEffect : public BaseEffect<SpotsEffect, SpotsEffectBase> {
private:
    using Self = SpotsEffect;
    using Base = BaseEffect<Self, SpotsEffectBase>;

public:
    static constexpr const char metaData[] PROGMEM = "Spots@Spread,Width,,,,,Overlay;!,!;!";
    static constexpr const uint8_t effectId = FX_MODE_SPOTS;

    using Base::Base;

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        return Base::nextFrameImpl(coordinate, (255 - SEGMENT.speed) << 8);
    }

private:
};


