#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

// colored stripes pulsing at a defined Beats-Per-Minute (BPM)
class BpmEffect : public BaseEffect<BpmEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = BpmEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Bpm@!;!;!;;sx=64";
    static constexpr const uint8_t effectId = FX_MODE_BPM;

    explicit BpmEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        uint32_t stp = (strip.now / 20) & 0xFF;
        uint8_t beat = beatsin8_t(SEGMENT.speed, 64, 255);
        for (unsigned i = 0; i < coordinate.width; i++) {
            buffer.setPixelColor(i, SEGMENT.color_from_palette(stp + (i * 2), false, PALETTE_SOLID_WRAP, 0, beat - stp + (i * 10)));
        }
        return true;
    }

private:
};


