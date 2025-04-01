#pragma once

#include "../FX.h"
#include "ColorwavesPrideEffectBase.h"
#include "Effect.h"

// ColorWavesWithPalettes by Mark Kriegsman: https://gist.github.com/kriegsman/8281905786e8b2632aeb
// This function draws color waves with an ever-changing,
// widely-varying set of parameters, using a color palette.
class ColorwavesEffect : public BaseEffect<ColorwavesEffect, ColorwavesPrideEffectBase> {
private:
    using Self = ColorwavesEffect;
    using Base = BaseEffect<Self, ColorwavesPrideEffectBase>;

public:
    static constexpr const char* const metaData = "Colorwaves@!,Hue;!;!;;pal=26";
    static constexpr const uint8_t effectId = FX_MODE_COLORWAVES;

    explicit ColorwavesEffect(const EffectInformation& ei) : Base{ei, false} {}

private:
};
