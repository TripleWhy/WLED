#pragma once

#include "../FX.h"
#include "Effect.h"

/*
 * Does the "standby-breathing" of well known i-Devices.
 */
class BreathEffect : public BaseEffect<BreathEffect> {
private:
    using Self = BreathEffect;
    using Base = BaseEffect<Self>;

public:
    static constexpr const char metaData[] PROGMEM = "Breathe@!;!,!;!;01";
    static constexpr const uint8_t effectId = FX_MODE_BREATH;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d1;

    using Base::Base;

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        unsigned var = 0;
        unsigned counter = (strip.now * ((SEGMENT.speed >> 3) +10)) & 0xFFFFU;
        counter = (counter >> 2) + (counter >> 4); //0-16384 + 0-2048
        if (counter < 16384) {
          if (counter > 8192) counter = 8192 - (counter - 8192);
          var = sin16_t(counter) / 103; //close to parabolic in range 0-8192, max val. 23170
        }

        lum = 30 + var;
        return true;
    }

    uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        return color_blend(SEGCOLOR(1), SEGMENT.color_from_palette(coordinate.getXAbsolute(), true, PALETTE_SOLID_WRAP, 0), lum);
    }

private:
    uint8_t lum;
};
