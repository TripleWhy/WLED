#pragma once

#include "../FX.h"
#include "ChaseEffectBase.h"
#include "Effect.h"

/*
 * Primary running followed by random color.
 */
class ChaseRandomEffect : public BaseEffect<ChaseRandomEffect, ChaseEffectBase> {
private:
    using Self = ChaseRandomEffect;
    using Base = BaseEffect<Self, ChaseEffectBase>;

public:
    static constexpr const char* const metaData = "Chase Random@!,Width;!,,!;!";
    static constexpr const uint8_t effectId = FX_MODE_CHASE_RANDOM;

    explicit constexpr ChaseRandomEffect(const EffectInformation& ei)
        : Base{ei, true, false}
    {
    }

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate, SEGCOLOR(1), (SEGCOLOR(2)) ? SEGCOLOR(2) : SEGCOLOR(0), SEGCOLOR(0));
    }
};
