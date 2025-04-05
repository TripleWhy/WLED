#pragma once

#include "../FX.h"
#include "Effect.h"

/*
 * Fades the LEDs between two colors
 */
class FadeEffect : public BaseEffect<FadeEffect> {
private:
    using Self = FadeEffect;
    using Base = BaseEffect<Self>;

public:
    static constexpr const char* const metaData = "Fade@!;!,!;!;01";
    static constexpr const uint8_t effectId = FX_MODE_FADE;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d1;

    using Base::Base;

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        unsigned counter = (strip.now * ((SEGMENT.speed >> 3) +10));
        lum = triwave16(counter) >> 8;
    }

    uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        return color_blend(SEGCOLOR(1), SEGMENT.color_from_palette(coordinate.getXAbsolute(), true, PALETTE_SOLID_WRAP, 0), lum);
    }

private:
    uint8_t lum;
};
