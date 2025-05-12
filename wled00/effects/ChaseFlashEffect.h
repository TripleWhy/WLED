#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Sec flashes running on prim.
 */
class ChaseFlashEffect : public BaseEffect<ChaseFlashEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    static constexpr unsigned FLASH_COUNT = 4;

    using Self = ChaseFlashEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Chase Flash@!,,,,,Animate BG;Bg,Fx;!;;o1=0";
    static constexpr const uint8_t effectId = FX_MODE_CHASE_FLASH;

    explicit ChaseFlashEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        // nextExecutionTimestamp rolls over before strip.now does
        if (strip.now < nextExecutionTimestamp) {
            return true;
        }

        unsigned flash_step = parameters.call % ((FLASH_COUNT * 2) + 1);

        uint32_t cycleTime = 10 + (255 - parameters.speed)*2;
        uint32_t it = strip.now / cycleTime;
        const bool moving = parameters.check1;
        for (unsigned i = 0; i < coordinate.width; i++) {
            unsigned palIdx = moving ? (i+it)%coordinate.width : i;
            buffer.setPixelColor(i, parameters.color_from_palette(palIdx, true, moving, 0));
        }

        unsigned delay = 10 + ((30 * (uint16_t)(255 - parameters.speed)) / coordinate.width);
        if(flash_step < (FLASH_COUNT * 2)) {
            if(flash_step % 2 == 0) {
                unsigned n = step;
                unsigned m = (step + 1) % coordinate.width;
                buffer.setPixelColor( n, SEGCOLOR(1));
                buffer.setPixelColor( m, SEGCOLOR(1));
                delay = 20;
            } else {
                delay = 30;
            }
        } else {
            step = (step + 1) % coordinate.width;
        }
        nextExecutionTimestamp = strip.now + delay;
        return true;
    }

private:
    decltype(strip.now) nextExecutionTimestamp{};
    uint32_t step{};
};


