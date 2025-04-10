#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/////////////////////////
//     2D Tartan       //
/////////////////////////
// By: Elliott Kember  https://editor.soulmatelights.com/gallery/3-tartan , Modified by: Andrew Tuline
class Tartan2dEffect : public BaseEffect<Tartan2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = Tartan2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "Tartan@X scale,Y scale,,,Sharpness;;!;2";
    static constexpr const uint8_t effectId = FX_MODE_2DTARTAN;

    explicit Tartan2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        if (SEGENV.call == 0) {
            buffer.fill(BLACK);
        }

        uint8_t hue, bri;
        size_t intensity;
        int offsetX = beatsin16_t(3, -360, 360);
        int offsetY = beatsin16_t(2, -360, 360);
        int sharpness = SEGMENT.custom3 / 8; // 0-3

        for (int x = 0; x < cols; x++) {
            for (int y = 0; y < rows; y++) {
                hue = x * beatsin16_t(10, 1, 10) + offsetY;
                intensity = bri = sin8_t(x * SEGMENT.speed/2 + offsetX);
                for (int i=0; i<sharpness; i++) intensity *= bri;
                intensity >>= 8*sharpness;
                buffer.setPixelColor(x, y, ColorFromPalette(SEGPALETTE, hue, intensity, LINEARBLEND));
                hue = y * 3 + offsetX;
                intensity = bri = sin8_t(y * SEGMENT.intensity/2 + offsetY);
                for (int i=0; i<sharpness; i++) intensity *= bri;
                intensity >>= 8*sharpness;
                buffer.addPixelColor(x, y, ColorFromPalette(SEGPALETTE, hue, intensity, LINEARBLEND));
            }
        }
    }

private:
};


#endif //WLED_DISABLE_2D
