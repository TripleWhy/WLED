#pragma once

#include "../FX.h"
#include "Effect.h"

/*
 * Blink/strobe function
 * Alternate between color1 and color2
 * if(strobe == true) then create a strobe effect
 */
class BlinkEffectBase : public Effect {
private:
    using Self = BlinkEffectBase;
    using Base = Effect;

public:
    explicit constexpr BlinkEffectBase(const EffectInformation& ei, bool strobe, bool do_palette)
        : Base(ei),
          strobe(strobe),
          do_palette(do_palette)
    {
    }

    void nextFrameImpl(uint32_t color1, uint32_t color2) {
        uint32_t cycleTime = (255 - SEGMENT.speed)*20;
        uint32_t onTime = FRAMETIME;
        if (!strobe) onTime += ((cycleTime * SEGMENT.intensity) >> 8);
        cycleTime += FRAMETIME*2;
        uint32_t it = strip.now / cycleTime;
        uint32_t rem = strip.now % cycleTime;

        bool on = false;
        if (it != SEGENV.step //new iteration, force on state for one frame, even if set time is too brief
            || rem <= onTime) {
            on = true;
        }

        SEGENV.step = it; //save previous iteration

        color = on ? color1 : color2;
        usePalette = (do_palette && (color == color1));
    }

    uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        if (usePalette) {
            return SEGMENT.color_from_palette(coordinate.getLinearIndex(), true, PALETTE_SOLID_WRAP, 0);
        } else {
            return color;
        }
    }

private:
    bool strobe;
    bool do_palette;
    bool usePalette{false}; // like, actually this time
    uint32_t color{0u};
};
