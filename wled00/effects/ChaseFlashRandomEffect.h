#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Prim flashes running, followed by random color.
 */
class ChaseFlashRandomEffect : public BaseEffect<ChaseFlashRandomEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = ChaseFlashRandomEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char* const metaData = "Chase Flash Rnd@!;!,!;!";
    static constexpr const uint8_t effectId = FX_MODE_CHASE_FLASH_RANDOM;

    explicit ChaseFlashRandomEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        // nextExecutionTimestamp rolls over before strip.now does
        if (strip.now < nextExecutionTimestamp) {
            return;
        }

        unsigned flash_step = SEGENV.call % ((FLASH_COUNT * 2) + 1);

        for (int i = 0; i < aux1; i++) {
            buffer.setPixelColor(i, SEGMENT.color_wheel(aux0));
        }

        unsigned delay = 1 + ((10 * (uint16_t)(255 - SEGMENT.speed)) / coordinate.width);
        if(flash_step < (FLASH_COUNT * 2)) {
            unsigned n = aux1;
            unsigned m = (aux1 + 1) % coordinate.width;
            if(flash_step % 2 == 0) {
                buffer.setPixelColor( n, SEGCOLOR(0));
                buffer.setPixelColor( m, SEGCOLOR(0));
                delay = 20;
            } else {
                buffer.setPixelColor( n, SEGMENT.color_wheel(aux0));
                buffer.setPixelColor( m, SEGCOLOR(1));
                delay = 30;
            }
        } else {
            aux1 = (aux1 + 1) % coordinate.width;

            if (aux1 == 0) {
                aux0 = get_random_wheel_index(aux0);
            }
        }
        nextExecutionTimestamp = strip.now + delay;
    }

private:
    decltype(strip.now) nextExecutionTimestamp{};
    uint16_t aux0{};
    uint16_t aux1{};
};


