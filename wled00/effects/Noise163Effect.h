#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

class Noise163Effect : public BaseEffect<Noise163Effect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = Noise163Effect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Noise 3@!;!;!;;pal=35";
    static constexpr const uint8_t effectId = FX_MODE_NOISE16_3;

    explicit Noise163Effect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        unsigned scale = 800;                                       // the "zoom factor" for the noise
        step += (1 + SEGMENT.speed);

        for (unsigned i = 0; i < coordinate.width; i++) {
            unsigned shift_x = 4223;                                  // no movement along x and y
            unsigned shift_y = 1234;
            uint32_t real_x = (i + shift_x) * scale;                  // calculate the coordinates within the noise field
            uint32_t real_y = (i + shift_y) * scale;                  // based on the precalculated positions
            uint32_t real_z = step*8;
            unsigned noise = perlin16(real_x, real_y, real_z) >> 8;   // get the noise data and scale it down
            unsigned index = sin8_t(noise * 3);                         // map led color based on noise data

            buffer.setPixelColor(i, SEGMENT.color_from_palette(index, false, PALETTE_SOLID_WRAP, 0, noise));
        }
        return true;
    }

private:
    uint32_t step{};
};


