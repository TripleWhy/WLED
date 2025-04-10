#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

class TriStaticPatternEffect : public BaseEffect<TriStaticPatternEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = TriStaticPatternEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Solid Pattern Tri@,Size;1,2,3;;;pal=0";
    static constexpr const uint8_t effectId = FX_MODE_TRI_STATIC_PATTERN;

    explicit TriStaticPatternEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        unsigned segSize = (SEGMENT.intensity >> 5) +1;
        unsigned currSeg = 0;
        unsigned currSegCount = 0;

        for (unsigned i = 0; i < coordinate.width; i++) {
            if ( currSeg % 3 == 0 ) {
                buffer.setPixelColor(i, SEGCOLOR(0));
            } else if( currSeg % 3 == 1) {
                buffer.setPixelColor(i, SEGCOLOR(1));
            } else {
                buffer.setPixelColor(i, SEGCOLOR(2));
            }
            currSegCount += 1;
            if (currSegCount >= segSize) {
                currSeg +=1;
                currSegCount = 0;
            }
        }
    }

private:
};


