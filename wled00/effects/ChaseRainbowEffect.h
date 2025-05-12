#pragma once

#include "../FX.h"
#include "ChaseEffectBase.h"
#include "Effect.h"

/*
 * Primary, secondary running on rainbow.
 */
class ChaseRainbowEffect : public BaseEffect<ChaseRainbowEffect, ChaseEffectBase> {
private:
    using Self = ChaseRainbowEffect;
    using Base = BaseEffect<Self, ChaseEffectBase>;

public:
    static constexpr const char metaData[] PROGMEM = "Chase Rainbow@!,Width;!,!;!";
    static constexpr const uint8_t effectId = FX_MODE_CHASE_RAINBOW;

    explicit constexpr ChaseRainbowEffect(const EffectInformation& ei)
        : Base{ei, false, false}
    {
    }

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        unsigned color_sep = 256 / coordinate.width;
        if (color_sep == 0) color_sep = 1;                                           // correction for segments longer than 256 LEDs
        unsigned color_index = parameters.call & 0xFF;
        uint32_t color = parameters.color_wheel(((step * color_sep) + color_index) & 0xFF);

        return Base::nextFrameImpl(parameters, coordinate, color, SEGCOLOR(0), SEGCOLOR(1));
    }
};

