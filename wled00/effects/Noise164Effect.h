#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//https://github.com/aykevl/ledstrip-spark/blob/master/ledstrip.ino
class Noise164Effect : public BaseEffect<Noise164Effect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = Noise164Effect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Noise 4@!;!;!;;pal=26";
    static constexpr const uint8_t effectId = FX_MODE_NOISE16_4;

    explicit Noise164Effect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        uint32_t stp = (strip.now * SEGMENT.speed) >> 7;
        for (unsigned i = 0; i < coordinate.width; i++) {
            int index = inoise16(uint32_t(i) << 12, stp);
            buffer.setPixelColor(i, SEGMENT.color_from_palette(index, false, PALETTE_SOLID_WRAP, 0));
        }
    }

private:
};


