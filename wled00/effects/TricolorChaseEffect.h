#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Tricolor chase function
 */
class TricolorChaseEffect : public BaseEffect<TricolorChaseEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = TricolorChaseEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Chase 3@!,Size;1,2,3;!";
    static constexpr const uint8_t effectId = FX_MODE_TRICOLOR_CHASE;

    explicit TricolorChaseEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        uint32_t cycleTime = 50 + ((255 - parameters.speed)<<1);
        uint32_t it = strip.now / cycleTime;  // iterator
        unsigned width = (1 + (parameters.intensity>>4)); // value of 1-16 for each colour
        unsigned index = it % (width*3);

        for (unsigned i = 0; i < coordinate.width; i++, index++) {
            if (index > (width*3)-1) index = 0;

            uint32_t color = SEGCOLOR(2);
            if (index > (width<<1)-1) color = parameters.color_from_palette(i, true, PALETTE_SOLID_WRAP, 1);
            else if (index > width-1) color = SEGCOLOR(0);

            buffer.setPixelColor(coordinate.width - i -1, color);
        }
        return true;
    }

private:
};


