#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Running random pixels ("Stream 2")
 * Custom mode by Keith Lord: https://github.com/kitesurfer1404/WS2812FX/blob/master/src/custom/RandomChase.h
 */
//TODO This effect can be improved in different ways:
// 1. If we reverse the flow direction, it doesn't need to be buffered.
// 2. If we keep the buffer, it doesn't need to re-generate all the clors reach frame, instead it can just user buffer.moveX().
//    That would remove the need to mess with the rng.
class RandomChaseEffect : public BaseEffect<RandomChaseEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = RandomChaseEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char* const metaData = "Stream 2@!;;";
    static constexpr const uint8_t effectId = FX_MODE_RANDOM_CHASE;

    explicit RandomChaseEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        if (SEGENV.call == 0) {
            step = RGBW32(random8(), random8(), random8(), 0);
            aux0 = random16();
        }
        unsigned prevSeed = random16_get_seed(); // save seed so we can restore it at the end of the function
        uint32_t cycleTime = 25 + (3 * (uint32_t)(255 - SEGMENT.speed));
        uint32_t it = strip.now / cycleTime;
        uint32_t color = step;
        random16_set_seed(aux0);

        const int width = static_cast<int>(coordinate.width);
        for (int i = width -1; i >= 0; i--) {
            uint8_t r = random8(6) != 0 ? (color >> 16 & 0xFF) : random8();
            uint8_t g = random8(6) != 0 ? (color >> 8  & 0xFF) : random8();
            uint8_t b = random8(6) != 0 ? (color       & 0xFF) : random8();
            color = RGBW32(r, g, b, 0);
            buffer.setPixelColor(i, color);
            if (i == width -1 && aux1 != (it & 0xFFFFU)) { //new first color in next frame
                step = color;
                aux0 = random16_get_seed();
            }
        }

        aux1 = it & 0xFFFF;

        random16_set_seed(prevSeed); // restore original seed so other effects can use "random" PRNG
    }

private:
    uint32_t step{};
    uint16_t aux0{};
    uint16_t aux1{};
};


