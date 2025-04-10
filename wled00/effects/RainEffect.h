#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "FireworksEffect.h"

//Twinkling LEDs running. Inspired by https://github.com/kitesurfer1404/WS2812FX/blob/master/src/custom/Rain.h
class RainEffect : public BaseEffect<RainEffect, FireworksEffect> {
private:
    using Self = RainEffect;
    using Base = BaseEffect<RainEffect, FireworksEffect>;

public:
    static constexpr const char metaData[] PROGMEM = "Rain@!,Spawning rate;!,!;!;12;ix=128,pal=0";
    static constexpr const uint8_t effectId = FX_MODE_RAIN;

    // explicit RainEffect(const EffectInformation& ei) : Base{ei, false} {}
    using Base::Base;

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        const unsigned width  = coordinate.width;
        const unsigned height = coordinate.height;
        step += FRAMETIME;
        if (SEGENV.call && step > SPEED_FORMULA_L) {
            step = 1;
            if (SEGMENT.is2D()) {
                buffer.movePixelsY(-1, true);  // move all pixels down
                sparkIndex = (sparkIndex % width) + (sparkIndex / width + 1) * width;
                oldSparkIndex = (oldSparkIndex % width) + (oldSparkIndex / width + 1) * width;
            } else {
                //shift all leds left
                buffer.movePixelsX(-1, true);  // move all pixels left
                sparkIndex++;  // increase spark index
                oldSparkIndex++;
            }
            if (sparkIndex == 0) sparkIndex = UINT16_MAX; // reset previous spark position
            if (oldSparkIndex == 0) sparkIndex = UINT16_MAX; // reset previous spark position
            if (sparkIndex >= width*height) sparkIndex = 0;     // ignore
            if (oldSparkIndex >= width*height) oldSparkIndex = 0;
        }

        Base::nextFrameImpl(coordinate);
    }
};

#endif //WLED_DISABLE_2D
