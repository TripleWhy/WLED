#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//////////////////////////
//     2D Firenoise     //
//////////////////////////
// firenoise2d. By Andrew Tuline. Yet another short routine.
class Firenoise2dEffect : public BaseEffect<Firenoise2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = Firenoise2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "Firenoise@X scale,Y scale,,,,Palette;;!;2;pal=66";
    static constexpr const uint8_t effectId = FX_MODE_2DFIRENOISE;

    explicit Firenoise2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        if (SEGENV.call == 0) {
            buffer.fill(BLACK);
        }

        unsigned xscale = SEGMENT.intensity*4;
        unsigned yscale = SEGMENT.speed*8;
        unsigned indexx = 0;

        CRGBPalette16 pal = SEGMENT.check1 ? SEGPALETTE : SEGMENT.loadPalette(pal, 35);  
        for (int j=0; j < cols; j++) {
            for (int i=0; i < rows; i++) {
                indexx = inoise8(j*yscale*rows/255, i*xscale+strip.now/4);                                               // We're moving along our Perlin map.
                buffer.setPixelColor(j, i, ColorFromPalette(pal, min(i*indexx/11, 225U), i*255/rows, LINEARBLEND));   // With that value, look up the 8 bit colour palette value and assign it to the current LED.    
            } // for i
        } // for j
        return true;
    }

private:
};


#endif //WLED_DISABLE_2D
