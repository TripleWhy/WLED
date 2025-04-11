#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//Speed slider sets amount of LEDs lit, intensity sets unlit
class StaticPatternEffect : public BaseEffect<StaticPatternEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = StaticPatternEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Solid Pattern@Fg size,Bg size;Fg,!;!;;pal=0";
    static constexpr const uint8_t effectId = FX_MODE_STATIC_PATTERN;

    explicit StaticPatternEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        unsigned lit = 1 + SEGMENT.speed;
        unsigned unlit = 1 + SEGMENT.intensity;
        bool drawingLit = true;
        unsigned cnt = 0;

        for (unsigned i = 0; i < coordinate.width; i++) {
            buffer.setPixelColor(i, (drawingLit) ? SEGMENT.color_from_palette(i, true, PALETTE_SOLID_WRAP, 0) : SEGCOLOR(1));
            cnt++;
            if (cnt >= ((drawingLit) ? lit : unlit)) {
                cnt = 0;
                drawingLit = !drawingLit;
            }
        }
        return true;
    }

private:
};


