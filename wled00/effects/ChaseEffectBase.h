#pragma once

#include "../FX.h"
#include "Effect.h"

/*
 * color chase function.
 * color1 = background color
 * color2 and color3 = colors of two adjacent leds
 */
class ChaseEffectBase : public Effect {
private:
    using Self = ChaseEffectBase;
    using Base = Effect;

public:
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d1;

    explicit constexpr ChaseEffectBase(const EffectInformation& ei, bool chase_random, bool do_palette)
        : Base{ei},
          chase_random{chase_random},
          do_palette{do_palette}
    {
    }

    bool nextFrameImpl(const EffectCoordinate& coordinate, uint32_t c1, uint32_t c2, uint32_t c3) {
        color1 = c1;
        color2 = c2;
        color3 = c3;

        uint16_t counter = strip.now * ((SEGMENT.speed >> 2) + 1);
        uint16_t a = (counter * coordinate.width) >> 16;

        if (chase_random) {
            if (a < step) //we hit the start again, choose new color for Chase random
            {
                previousRandomColorIndex = randomColorIndex;
                randomColorIndex = get_random_wheel_index(randomColorIndex);
            }
            color1 = SEGMENT.color_wheel(randomColorIndex);
        }
        step = a;

        // Use intensity setting to vary chase up to 1/2 string length
        unsigned size = 1 + ((SEGMENT.intensity * coordinate.width) >> 10);

        uint16_t b = a + size; //"trail" of chase, filled with color1
        if (b > coordinate.width) b -= coordinate.width;
        uint16_t c = b + size;
        if (c > coordinate.width) c -= coordinate.width;
        return true;
    }

    uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        const unsigned i = coordinate.getXAbsolute();

        //fill between points a and b with color2
        if (a < b)
        {
            if (a <= i && i < b)
                return color2;
        } else {
            if (a <= i && i < coordinate.width) //fill until end
                return color2;
            if (i < b) //fill from start until b
                return color2;
        }

        //fill between points b and c with color2
        if (b < c)
        {
            if (b <= i && i < c)
                return color3;
        } else {
            if (b <= i && i < coordinate.width) //fill until end
                return color3;
            if (i < c) //fill from start until c
                return color3;
        }

        //background

        //if random, fill old background between a and end
        if (chase_random)
        {
            color1 = SEGMENT.color_wheel(previousRandomColorIndex);
            if (i >= a)
                return color1;
        }
        if (do_palette)
        {
            return SEGMENT.color_from_palette(i, true, PALETTE_SOLID_WRAP, 1);
        }
        return color1;
    }

protected:
    uint32_t step{};

private:
    const bool chase_random;
    const bool do_palette;

    uint16_t a{};
    uint16_t b{};
    uint16_t c{};
    uint32_t color1{};
    uint32_t color2{};
    uint32_t color3{};

    uint16_t randomColorIndex{};
    uint16_t previousRandomColorIndex{};
};
