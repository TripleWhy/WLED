#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Dots waving around in a sine/pendulum motion.
 * Little pixel birds flying in a circle. By Aircoookie
 */
class ChunchunEffect : public BaseEffect<ChunchunEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = ChunchunEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Chunchun@!,Gap size;!,!;!";
    static constexpr const uint8_t effectId = FX_MODE_CHUNCHUN;

    explicit ChunchunEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        buffer.fadeOut(254); // add a bit of trail
        unsigned counter = strip.now * (6 + (SEGMENT.speed >> 4));
        unsigned numBirds = 2 + (coordinate.width >> 3);  // 2 + 1/8 of a segment
        unsigned span = (SEGMENT.intensity << 8) / numBirds;

        for (unsigned i = 0; i < numBirds; i++)
        {
            counter -= span;
            unsigned megumin = sin16_t(counter) + 0x8000;
            unsigned bird = uint32_t(megumin * coordinate.width) >> 16;
            bird = constrain(bird, 0U, coordinate.width-1U);
            buffer.setPixelColor(bird, SEGMENT.color_from_palette((i * 255)/ numBirds, false, false, 0)); // no palette wrapping
        }
        return true;
    }

private:
};


