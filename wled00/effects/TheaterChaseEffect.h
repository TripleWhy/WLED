#pragma once

#include "../FX.h"
#include "Effect.h"

/*
 * Alternating pixels running function / Theatre-style crawling lights.
 * Inspired by the Adafruit examples.
 */
class TheaterChaseEffect : public BaseEffect<TheaterChaseEffect> {
private:
    using Self = TheaterChaseEffect;
    using Base = BaseEffect<Self>;

public:
    static constexpr const char metaData[] PROGMEM = "Theater@!,Gap size,,,,Rainbow,,Theater;!,!;!;;o1=0,o3=1";
    static constexpr const uint8_t effectId = FX_MODE_THEATER_CHASE;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d1;

    using Base::Base;

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        const bool animate = parameters.check1;
        const bool theatre = parameters.check3;
        width = (theatre ? 3 : 1) + (parameters.intensity >> 4);  // window
        uint32_t cycleTime = 50 + (255 - parameters.speed);
        uint32_t it = strip.now / cycleTime;

        c2 = SEGCOLOR(1);
        if (animate) {
            c1 = parameters.color_wheel(step); // sets moving palette and rainbow for default
        }

        counter = nextCounter;
        if (it != step) {
            nextCounter = (nextCounter +1) % (theatre ? width : (width<<1));
            step = it;
        }
        return true;
    }

    uint32_t getPixelColorImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        const bool animate = parameters.check1;
        const bool theatre = parameters.check3;
        if (!animate) {
            c1 = parameters.color_from_palette(coordinate.getXAbsolute(), true, false, 0);
        }
        if (theatre) {
            if ((coordinate.getXAbsolute() % width) == counter) {
                return c1;
            }
        } else {
            int pos = (coordinate.getXAbsolute() % (width<<1));
            if ((pos < counter-width) || ((pos >= counter) && (pos < counter+width))) {
                return c1;
            }
        }
        return c2;
    }

private:
    uint16_t counter;
    uint16_t nextCounter{};
    uint32_t step{};
    uint32_t c1;
    uint32_t c2;
    int width;
};
