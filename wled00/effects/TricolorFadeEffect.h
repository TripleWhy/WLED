#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Fades between 3 colors
 * Custom mode by Keith Lord: https://github.com/kitesurfer1404/WS2812FX/blob/master/src/custom/TriFade.h
 * Modified by Aircoookie
 */
class TricolorFadeEffect : public BaseEffect<TricolorFadeEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = TricolorFadeEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Tri Fade@!;1,2,3;!";
    static constexpr const uint8_t effectId = FX_MODE_TRICOLOR_FADE;

    explicit TricolorFadeEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        unsigned counter = strip.now * ((parameters.speed >> 3) +1);
        uint16_t prog = (counter * 768) >> 16;

        uint32_t color1 = 0, color2 = 0;
        unsigned stage = 0;

        if(prog < 256) {
            color1 = SEGCOLOR(0);
            color2 = SEGCOLOR(1);
            stage = 0;
        } else if(prog < 512) {
            color1 = SEGCOLOR(1);
            color2 = SEGCOLOR(2);
            stage = 1;
        } else {
            color1 = SEGCOLOR(2);
            color2 = SEGCOLOR(0);
            stage = 2;
        }

        byte stp = prog; // % 256
        for (unsigned i = 0; i < coordinate.width; i++) {
            uint32_t color;
            if (stage == 2) {
                color = color_blend(SEGMENT.color_from_palette(i, true, PALETTE_SOLID_WRAP, 2), color2, stp);
            } else if (stage == 1) {
                color = color_blend(color1, SEGMENT.color_from_palette(i, true, PALETTE_SOLID_WRAP, 2), stp);
            } else {
                color = color_blend(color1, color2, stp);
            }
            buffer.setPixelColor(i, color);
        }
        return true;
    }

private:
};


