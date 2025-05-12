#pragma once

#include "../FX.h"
#include "ChaseEffectBase.h"
#include "Effect.h"

/*
 * Primary running on rainbow.
 */
class ChaseRainbowWhiteEffect : public BaseEffect<ChaseRainbowWhiteEffect, ChaseEffectBase> {
private:
    using Self = ChaseRainbowWhiteEffect;
    using Base = BaseEffect<Self, ChaseEffectBase>;

public:
    static constexpr const char metaData[] PROGMEM = "Rainbow Runner@!,Size;Bg;!";
    static constexpr const uint8_t effectId = FX_MODE_CHASE_RAINBOW_WHITE;

    explicit constexpr ChaseRainbowWhiteEffect(const EffectInformation& ei)
        : Base{ei, false, false}
    {
    }

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        uint16_t n = step;
        uint16_t m = (step + 1) % coordinate.width;
        uint32_t color2 = parameters.color_wheel(((n * 256 / coordinate.width) + (parameters.call & 0xFF)) & 0xFF);
        uint32_t color3 = parameters.color_wheel(((m * 256 / coordinate.width) + (parameters.call & 0xFF)) & 0xFF);

        return Base::nextFrameImpl(parameters, coordinate, SEGCOLOR(0), color2, color3);
    }
};
