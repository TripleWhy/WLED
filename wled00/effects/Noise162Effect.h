#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

class Noise162Effect : public BaseEffect<Noise162Effect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = Noise162Effect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Noise 2@!;!;!;;pal=43";
    static constexpr const uint8_t effectId = FX_MODE_NOISE16_2;

    explicit Noise162Effect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        unsigned scale = 1000;                                        // the "zoom factor" for the noise
        step += (1 + (SEGMENT.speed >> 1));

        for (unsigned i = 0; i < coordinate.width; i++) {
            unsigned shift_x = step >> 6;                        // x as a function of time
            uint32_t real_x = (i + shift_x) * scale;                    // calculate the coordinates within the noise field
            unsigned noise = perlin16(real_x, 0, 4223) >> 8;            // get the noise data and scale it down
            unsigned index = sin8_t(noise * 3);                           // map led color based on noise data

            buffer.setPixelColor(i, SEGMENT.color_from_palette(index, false, PALETTE_SOLID_WRAP, 0, noise));
        }
        return true;
    }

private:
    uint32_t step{};
};


