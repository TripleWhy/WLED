#pragma once

#include "../FX.h"
#include "ChaseEffectBase.h"
#include "Effect.h"

/*
 * Bicolor chase, more primary color.
 */
class ChaseColorEffect : public BaseEffect<ChaseColorEffect, ChaseEffectBase> {
private:
    using Self = ChaseColorEffect;
    using Base = BaseEffect<Self, ChaseEffectBase>;

public:
    static constexpr const char* const metaData = "Chase@!,Width;!,!,!;!";
    static constexpr const uint8_t effectId = FX_MODE_CHASE_COLOR;

    explicit constexpr ChaseColorEffect(const EffectInformation& ei)
        : Base{ei, false, true}
    {
    }

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate, SEGCOLOR(1), (SEGCOLOR(2)) ? SEGCOLOR(2) : SEGCOLOR(0), SEGCOLOR(0));
    }
};
