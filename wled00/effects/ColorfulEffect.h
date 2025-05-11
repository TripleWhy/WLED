#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Red - Amber - Green - Blue lights running
 */
class ColorfulEffect : public BaseEffect<ColorfulEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = ColorfulEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Colorful@!,Saturation;1,2,3;!";
    static constexpr const uint8_t effectId = FX_MODE_COLORFUL;

    explicit ColorfulEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        unsigned numColors = 4; //3, 4, or 5
        uint32_t cols[9]{0x00FF0000,0x00EEBB00,0x0000EE00,0x000077CC};
        if (parameters.intensity > 160 || SEGMENT.palette) { //palette or color
            if (!SEGMENT.palette) {
                numColors = 3;
                for (size_t i = 0; i < 3; i++) cols[i] = SEGCOLOR(i);
            } else {
                unsigned fac = 80;
                if (SEGMENT.palette == 52) {numColors = 5; fac = 61;} //C9 2 has 5 colors
                for (size_t i = 0; i < numColors; i++) {
                    cols[i] = SEGMENT.color_from_palette(i*fac, false, true, 255);
                }
            }
        } else if (parameters.intensity < 80) //pastel (easter) colors
        {
            cols[0] = 0x00FF8040;
            cols[1] = 0x00E5D241;
            cols[2] = 0x0077FF77;
            cols[3] = 0x0077F0F0;
        }
        for (size_t i = numColors; i < numColors*2 -1U; i++) cols[i] = cols[i-numColors];

        uint32_t cycleTime = 50 + (8 * (uint32_t)(255 - parameters.speed));
        uint32_t it = strip.now / cycleTime;
        if (it != step)
        {
            if (parameters.speed > 0) aux0++;
            if (aux0 >= numColors) aux0 = 0;
            step = it;
        }

        for (unsigned i = 0; i < coordinate.width; i+= numColors)
        {
            for (unsigned j = 0; j < numColors; j++) buffer.setPixelColor(i + j, cols[aux0 + j]);
        }
        return true;
    }

private:
    uint32_t step{};
    uint16_t aux0{};
};


