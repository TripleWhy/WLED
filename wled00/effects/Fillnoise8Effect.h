#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

class Fillnoise8Effect : public BaseEffect<Fillnoise8Effect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = Fillnoise8Effect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Fill Noise@!;!;!";
    static constexpr const uint8_t effectId = FX_MODE_FILLNOISE8;

    explicit Fillnoise8Effect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        if (SEGENV.call == 0) step = hw_random();
        for (unsigned i = 0; i < coordinate.width; i++) {
            unsigned index = perlin8(i * coordinate.width, step + i * coordinate.width);
            buffer.setPixelColor(i, SEGMENT.color_from_palette(index, false, PALETTE_SOLID_WRAP, 0));
        }
        step += beatsin8_t(SEGMENT.speed, 1, 6); //10,1,4
        return true;
    }

private:
    uint32_t step{};
};


