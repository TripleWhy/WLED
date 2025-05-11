#pragma once

#include "../FX.h"
#include "Effect.h"

/*
  Imitates a washing machine, rotating same waves forward, then pause, then backward.
  By Stefan Seegel
*/
class WashingMachineEffect : public BaseEffect<WashingMachineEffect> {
private:
    using Self = WashingMachineEffect;
    using Base = BaseEffect<Self>;

public:
    static constexpr const char metaData[] PROGMEM = "Washing Machine@!,!;;!";
    static constexpr const uint8_t effectId = FX_MODE_WASHING_MACHINE;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d1;

    using Base::Base;

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        speed = tristate_square8(strip.now >> 7, 90, 15);
        step += (speed * 2048) / (512 - parameters.speed);
        return true;
    }

    uint32_t getPixelColorImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        const unsigned i = coordinate.getXAbsolute();
        uint8_t col = sin8_t(((parameters.intensity / 25 + 1) * 255 * i / coordinate.width) + (step >> 7));
        return SEGMENT.color_from_palette(col, false, PALETTE_SOLID_WRAP, 3);
    }

private:
    int speed{};
    uint32_t step{};
};


