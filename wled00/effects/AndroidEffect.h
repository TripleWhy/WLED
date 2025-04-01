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
    static constexpr const char* const metaData = "Android@!,Width;!,!;!;;m12=1";
    static constexpr const uint8_t effectId = FX_MODE_ANDROID;

    explicit AndroidEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        // nextExecutionTimestamp rolls over before strip.now does
        if (strip.now < nextExecutionTimestamp) {
            return;
        }
        nextExecutionTimestamp = strip.now + (3 + ((8 * (uint32_t)(255 - SEGMENT.speed)) / coordinate.width));

        for (unsigned i = 0; i < coordinate.width; i++) {
            buffer.setPixelColor(i, SEGMENT.color_from_palette(i, true, PALETTE_SOLID_WRAP, 1));
        }

        if (aux1 > (SEGMENT.intensity*coordinate.width)/255)
        {
            aux0 = 1;
        } else
        {
            if (aux1 < 2) aux0 = 0;
        }

        unsigned a = step & 0xFFFFU;

        if (aux0 == 0)
        {
            if (SEGENV.call %3 == 1) {a++;}
            else {aux1++;}
        } else
        {
            a++;
            if (SEGENV.call %3 != 1) aux1--;
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
    }

private:
    decltype(strip.now) nextExecutionTimestamp{};
    uint32_t step{};
    uint16_t aux0{};
    uint16_t aux1{};
};


