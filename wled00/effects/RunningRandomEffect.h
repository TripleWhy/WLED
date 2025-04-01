#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Random colored pixels running. ("Stream")
 */
class RunningRandomEffect : public BaseEffect<RunningRandomEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = RunningRandomEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char* const metaData = "Stream@!,Zone size;;!";
    static constexpr const uint8_t effectId = FX_MODE_RUNNING_RANDOM;

    explicit RunningRandomEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        uint32_t cycleTime = 25 + (3 * (uint32_t)(255 - SEGMENT.speed));
        uint32_t it = strip.now / cycleTime;
        if (SEGENV.call == 0) aux0 = hw_random(); // random seed for PRNG on start

        unsigned zoneSize = ((255-SEGMENT.intensity) >> 4) +1;
        uint16_t PRNG16 = aux0;

        unsigned z = it % zoneSize;
        bool nzone = (!z && it != aux1);
        for (int i=coordinate.width-1; i >= 0; i--) {
            if (nzone || z >= zoneSize) {
                unsigned lastrand = PRNG16 >> 8;
                int16_t diff = 0;
                while (abs(diff) < 42) { // make sure the difference between adjacent colors is big enough
                    PRNG16 = (uint16_t)(PRNG16 * 2053) + 13849; // next zone, next 'random' number
                    diff = (PRNG16 >> 8) - lastrand;
                }
                if (nzone) {
                    aux0 = PRNG16; // save next starting seed
                    nzone = false;
                }
                z = 0;
            }
            buffer.setPixelColor(i, SEGMENT.color_wheel(PRNG16 >> 8));
            z++;
        }

        aux1 = it;
    }

private:
    uint16_t aux0{};
    uint16_t aux1{};
};


