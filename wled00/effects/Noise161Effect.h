#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

class Noise161Effect : public BaseEffect<Noise161Effect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = Noise161Effect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Noise 1@!;!;!;;pal=20";
    static constexpr const uint8_t effectId = FX_MODE_NOISE16_1;

    explicit Noise161Effect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        unsigned scale = 320;                                       // the "zoom factor" for the noise
        step += (1 + SEGMENT.speed/16);

        for (unsigned i = 0; i < coordinate.width; i++) {
            unsigned shift_x = beatsin8_t(11);                          // the x position of the noise field swings @ 17 bpm
            unsigned shift_y = step/42;                        // the y position becomes slowly incremented
            unsigned real_x = (i + shift_x) * scale;                  // the x position of the noise field swings @ 17 bpm
            unsigned real_y = (i + shift_y) * scale;                  // the y position becomes slowly incremented
            uint32_t real_z = step;                            // the z position becomes quickly incremented
            unsigned noise = perlin16(real_x, real_y, real_z) >> 8;   // get the noise data and scale it down
            unsigned index = sin8_t(noise * 3);                         // map LED color based on noise data

            buffer.setPixelColor(i, SEGMENT.color_from_palette(index, false, PALETTE_SOLID_WRAP, 0));
        }
        return true;
    }

private:
    uint32_t step{};
};


