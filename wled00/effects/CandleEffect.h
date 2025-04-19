#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//values close to 100 produce 5Hz flicker, which looks very candle-y
//Inspired by https://github.com/avanhanegem/ArduinoCandleEffectNeoPixel
//and https://cpldcpu.wordpress.com/2016/01/05/reverse-engineering-a-real-candle/

class CandleEffect : public BaseEffect<CandleEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = CandleEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Candle@!,!,,,,,,Multi;!,!;!;01;sx=96,ix=224,pal=0";
    static constexpr const uint8_t effectId = FX_MODE_CANDLE;

    explicit CandleEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        bool multi = SEGMENT.check3 && coordinate.width;
        if (multi) {
            const unsigned dataSize = max(1, (int)coordinate.width -1) *3; //max. 1365 pixels (ESP8266)
            if (!data.resize(dataSize)) {
                multi = false;
            }
        }

        //max. flicker range controlled by intensity
        unsigned valrange = SEGMENT.intensity;
        unsigned rndval = valrange >> 1; //max 127

        //step (how much to move closer to target per frame) coarsely set by speed
        unsigned speedFactor = 4;
        if (SEGMENT.speed > 252) { //epilepsy
            speedFactor = 1;
        } else if (SEGMENT.speed > 99) { //regular candle (mode called every ~25 ms, so 4 frames to have a new target every 100ms)
            speedFactor = 2;
        } else if (SEGMENT.speed > 49) { //slower fade
            speedFactor = 3;
        } //else 4 (slowest)

        unsigned numCandles = (multi) ? coordinate.width : 1;

        for (unsigned i = 0; i < numCandles; i++)
        {
            unsigned d = 0; //data location

            unsigned s = aux0, s_target = aux1, fadeStep = step;
            if (i > 0) {
                d = (i-1) *3;
                s = data[d]; s_target = data[d+1]; fadeStep = data[d+2];
            }
            if (fadeStep == 0) { //init vals
                s = 128; s_target = 130 + hw_random8(4); fadeStep = 1;
            }

            bool newTarget = false;
            if (s_target > s) { //fade up
                s = qadd8(s, fadeStep);
                if (s >= s_target) newTarget = true;
            } else {
                s = qsub8(s, fadeStep);
                if (s <= s_target) newTarget = true;
            }

            if (newTarget) {
                s_target = hw_random8(rndval) + hw_random8(rndval); //between 0 and rndval*2 -2 = 252
                if (s_target < (rndval >> 1)) s_target = (rndval >> 1) + hw_random8(rndval);
                unsigned offset = (255 - valrange);
                s_target += offset;

                unsigned dif = (s_target > s) ? s_target - s : s - s_target;

                fadeStep = dif >> speedFactor;
                if (fadeStep == 0) fadeStep = 1;
            }

            if (i > 0) {
                buffer.setPixelColor(i, color_blend(SEGCOLOR(1), SEGMENT.color_from_palette(i, true, PALETTE_SOLID_WRAP, 0), uint8_t(s)));

                data[d] = s; data[d+1] = s_target; data[d+2] = fadeStep;
            } else {
                for (unsigned j = 0; j < coordinate.width; j++) {
                    buffer.setPixelColor(j, color_blend(SEGCOLOR(1), SEGMENT.color_from_palette(j, true, PALETTE_SOLID_WRAP, 0), uint8_t(s)));
                }

                aux0 = s; aux1 = s_target; step = fadeStep;
            }
        }

        return true;
    }

private:
    SegmentAllocator<byte>::vector data{};
    uint32_t step{};
    uint16_t aux0{};
    uint16_t aux1{};
};


