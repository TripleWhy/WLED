#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

////////////////////////////////
//  2D Polar Lights           //
////////////////////////////////
// By: Kostyantyn Matviyevskyy  https://editor.soulmatelights.com/gallery/762-polar-lights , Modified by: Andrew Tuline & @dedehai (palette support)

class PolarLights2dEffect : public BaseEffect<PolarLights2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = PolarLights2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "Polar Lights@!,Scale,,,,Flip Palette;;!;2;pal=71";
    static constexpr const uint8_t effectId = FX_MODE_2DPOLARLIGHTS;

    explicit PolarLights2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        if (SEGENV.call == 0) {
            buffer.fill(BLACK);
            step = 0;
        }

        float adjustHeight = (float)map(rows, 8, 32, 28, 12); // maybe use mapf() ???
        unsigned adjScale = map(cols, 8, 64, 310, 63);
        unsigned _scale = map(SEGMENT.intensity, 0, 255, 30, adjScale);
        int _speed = map(SEGMENT.speed, 0, 255, 128, 16);

        for (int x = 0; x < cols; x++) {
            for (int y = 0; y < rows; y++) {
                step++;
                uint8_t palindex = qsub8(inoise8((step%2) + x * _scale, y * 16 + step % 16, step / _speed), fabsf((float)rows / 2.0f - (float)y) * adjustHeight);
                uint8_t palbrightness = palindex;
                if(SEGMENT.check1) palindex = 255 - palindex; //flip palette
                buffer.setPixelColor(x, y, SEGMENT.color_from_palette(palindex, false, false, 255, palbrightness));
            }
        }
        return true;
    }

private:
    uint32_t step{};
};


#endif //WLED_DISABLE_2D
