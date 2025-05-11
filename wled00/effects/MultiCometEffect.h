#pragma once

#include <array>
#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Creates random comets
 * Custom mode by Keith Lord: https://github.com/kitesurfer1404/WS2812FX/blob/master/src/custom/MultiComet.h
 */
class MultiCometEffect : public BaseEffect<MultiCometEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    static constexpr unsigned MAX_COMETS = 8;

    using Self = MultiCometEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Multi Comet@!,Fade;!,!;!;1";
    static constexpr const uint8_t effectId = FX_MODE_MULTI_COMET;

    explicit MultiCometEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        uint32_t cycleTime = 10 + (uint32_t)(255 - parameters.speed);
        uint32_t it = strip.now / cycleTime;
        if (step == it)
            return true;

        buffer.fadeOut(parameters.intensity/2 + 128);

        for (unsigned i=0; i < MAX_COMETS; i++) {
            if(comets[i] < coordinate.width) {
                unsigned index = comets[i];
                if (SEGCOLOR(2) != 0)
                {
                    buffer.setPixelColor(index, i % 2 ? SEGMENT.color_from_palette(index, true, PALETTE_SOLID_WRAP, 0) : SEGCOLOR(2));
                } else
                {
                    buffer.setPixelColor(index, SEGMENT.color_from_palette(index, true, PALETTE_SOLID_WRAP, 0));
                }
                comets[i]++;
            } else {
                if(!hw_random16(coordinate.width)) {
                    comets[i] = 0;
                }
            }
        }

        step = it;
        return true;
    }

private:
    std::array<uint16_t, MAX_COMETS> comets{};
    uint32_t step{};
};

