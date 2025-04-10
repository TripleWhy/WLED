#pragma once

#include <array>
#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
/  Oscillating bars of color, updated with standard framerate
*/
class OscillateEffect : public BaseEffect<OscillateEffect> {
private:
    struct Oscillator {
        uint16_t pos;
        uint8_t  size;
        int8_t   dir;
        uint8_t  speed;
    };

private:
    using Self = OscillateEffect;
    using Base = BaseEffect<Self>;
    static constexpr unsigned numOscillators = 3;

public:
    static constexpr const char metaData[] PROGMEM = "Oscillate";
    static constexpr const uint8_t effectId = FX_MODE_OSCILLATE;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d1;

    using Base::Base;

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        if (SEGENV.call == 0)
        {
            oscillators[0] = {(uint16_t)(coordinate.width/4),   (uint8_t)(coordinate.width/8),  1, 1};
            oscillators[1] = {(uint16_t)(coordinate.width/4*3), (uint8_t)(coordinate.width/8),  1, 2};
            oscillators[2] = {(uint16_t)(coordinate.width/4*2), (uint8_t)(coordinate.width/8), -1, 1};
        }

        uint32_t cycleTime = 20 + (2 * (uint32_t)(255 - SEGMENT.speed));
        uint32_t it = strip.now / cycleTime;

        for (unsigned i = 0; i < numOscillators; i++) {
            // if the counter has increased, move the oscillator by the random step
            if (it != step) oscillators[i].pos += oscillators[i].dir * oscillators[i].speed;
            oscillators[i].size = coordinate.width/(3+SEGMENT.intensity/8);
            if((oscillators[i].dir == -1) && (oscillators[i].pos > coordinate.width << 1)) { // use integer overflow
                oscillators[i].pos = 0;
                oscillators[i].dir = 1;
                // make bigger steps for faster speeds
                oscillators[i].speed = SEGMENT.speed > 100 ? hw_random8(2, 4):hw_random8(1, 3);
            }
            if((oscillators[i].dir == 1) && (oscillators[i].pos >= (coordinate.width - 1))) {
                oscillators[i].pos = coordinate.width - 1;
                oscillators[i].dir = -1;
                oscillators[i].speed = SEGMENT.speed > 100 ? hw_random8(2, 4):hw_random8(1, 3);
            }
        }

        step = it;
    }

    uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        const unsigned i = coordinate.getXAbsolute();
        uint32_t color = BLACK;
        for (unsigned j = 0; j < numOscillators; j++) {
            if((int)i >= (int)oscillators[j].pos - oscillators[j].size && i <= oscillators[j].pos + oscillators[j].size) {
                color = (color == BLACK) ? SEGCOLOR(j) : color_blend(color, SEGCOLOR(j), uint8_t(128));
            }
        }
        return color;
    }

private:
    std::array<Oscillator, numOscillators> oscillators{};
    uint32_t step{};
};


