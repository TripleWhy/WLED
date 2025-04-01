#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Custom mode by Aircoookie. Color Wipe, but with 3 colors
 */
class TricolorWipeEffect : public BaseEffect<TricolorWipeEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = TricolorWipeEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char* const metaData = "Tri Wipe@!;1,2,3;!";
    static constexpr const uint8_t effectId = FX_MODE_TRICOLOR_WIPE;

    explicit TricolorWipeEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        uint32_t cycleTime = 1000 + (255 - SEGMENT.speed)*200;
        uint32_t perc = strip.now % cycleTime;
        unsigned prog = (perc * 65535) / cycleTime;
        unsigned ledIndex = (prog * coordinate.width * 3) >> 16;
        unsigned ledOffset = ledIndex;

        for (unsigned i = 0; i < coordinate.width; i++)
        {
            buffer.setPixelColor(i, SEGMENT.color_from_palette(i, true, PALETTE_SOLID_WRAP, 2));
        }

        if(ledIndex < coordinate.width) { //wipe from 0 to 1
            for (unsigned i = 0; i < coordinate.width; i++)
            {
                buffer.setPixelColor(i, (i > ledOffset)? SEGCOLOR(0) : SEGCOLOR(1));
            }
        } else if (ledIndex < coordinate.width*2) { //wipe from 1 to 2
            ledOffset = ledIndex - coordinate.width;
            for (unsigned i = ledOffset +1; i < coordinate.width; i++)
            {
                buffer.setPixelColor(i, SEGCOLOR(1));
            }
        } else //wipe from 2 to 0
        {
            ledOffset = ledIndex - coordinate.width*2;
            for (unsigned i = 0; i <= ledOffset; i++)
            {
                buffer.setPixelColor(i, SEGCOLOR(0));
            }
        }
    }

private:
};


