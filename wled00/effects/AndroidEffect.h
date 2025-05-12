#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Android loading circle
 */
class AndroidEffect : public BaseEffect<AndroidEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = AndroidEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Android@!,Width;!,!;!;;m12=1";
    static constexpr const uint8_t effectId = FX_MODE_ANDROID;

    explicit AndroidEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        // nextExecutionTimestamp rolls over before strip.now does
        if (strip.now < nextExecutionTimestamp) {
            return false;
        }
        nextExecutionTimestamp = strip.now + (3 + ((8 * (uint32_t)(255 - parameters.speed)) / coordinate.width));

        for (unsigned i = 0; i < coordinate.width; i++) {
            buffer.setPixelColor(i, parameters.color_from_palette(i, true, PALETTE_SOLID_WRAP, 1));
        }

        if (aux1 > (parameters.intensity*coordinate.width)/255)
        {
            aux0 = 1;
        } else
        {
            if (aux1 < 2) aux0 = 0;
        }

        unsigned a = step & 0xFFFFU;

        if (aux0 == 0)
        {
            if (parameters.call %3 == 1) {a++;}
            else {aux1++;}
        } else
        {
            a++;
            if (parameters.call %3 != 1) aux1--;
        }

        if (a >= coordinate.width) a = 0;

        if (a + aux1 < coordinate.width)
        {
            for (unsigned i = a; i < a+aux1; i++) {
                buffer.setPixelColor(i, SEGCOLOR(0));
            }
        } else
        {
            for (unsigned i = a; i < coordinate.width; i++) {
                buffer.setPixelColor(i, SEGCOLOR(0));
            }
            for (unsigned i = 0; i < aux1 - (coordinate.width -a); i++) {
                buffer.setPixelColor(i, SEGCOLOR(0));
            }
        }
        step = a;
        return true;
    }

private:
    decltype(strip.now) nextExecutionTimestamp{};
    uint32_t step{};
    uint16_t aux0{};
    uint16_t aux1{};
};


