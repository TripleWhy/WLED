#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//////////////////////////////
//     2D Squared Swirl     //
//////////////////////////////
// custom3 affects the blur amount.
// By: Mark Kriegsman. https://gist.github.com/kriegsman/368b316c55221134b160
// Modifed by: Andrew Tuline
class Squaredswirl2dEffect : public BaseEffect<Squaredswirl2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = Squaredswirl2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char* const metaData = "Squared Swirl@,Fade,,,Blur;;!;2";
    static constexpr const uint8_t effectId = FX_MODE_2DSQUAREDSWIRL;

    explicit Squaredswirl2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        const uint8_t kBorderWidth = 2;

        buffer.fadeToBlackBy(1 + SEGMENT.intensity / 5);
        buffer.blur(SEGMENT.custom3>>1);

        // Use two out-of-sync sine waves
        int i = beatsin8_t(19, kBorderWidth, cols-kBorderWidth);
        int j = beatsin8_t(22, kBorderWidth, cols-kBorderWidth);
        int k = beatsin8_t(17, kBorderWidth, cols-kBorderWidth);
        int m = beatsin8_t(18, kBorderWidth, rows-kBorderWidth);
        int n = beatsin8_t(15, kBorderWidth, rows-kBorderWidth);
        int p = beatsin8_t(20, kBorderWidth, rows-kBorderWidth);

        buffer.addPixelColor(i, m, ColorFromPalette(SEGPALETTE, strip.now/29, 255, LINEARBLEND));
        buffer.addPixelColor(j, n, ColorFromPalette(SEGPALETTE, strip.now/41, 255, LINEARBLEND));
        buffer.addPixelColor(k, p, ColorFromPalette(SEGPALETTE, strip.now/73, 255, LINEARBLEND));
    }

private:
};


#endif //WLED_DISABLE_2D
