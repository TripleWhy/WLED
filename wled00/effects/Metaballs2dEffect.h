#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/////////////////////////
//     2D Metaballs    //
/////////////////////////
// Metaballs by Stefan Petrick. Cannot have one of the dimensions be 2 or less. Adapted by Andrew Tuline.
class Metaballs2dEffect : public BaseEffect<Metaballs2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = Metaballs2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char* const metaData = "Metaballs@!;;!;2";
    static constexpr const uint8_t effectId = FX_MODE_2DMETABALLS;

    explicit Metaballs2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        float speed = 0.25f * (1+(SEGMENT.speed>>6));

        // get some 2 random moving points
        int x2 = map(inoise8(strip.now * speed, 25355, 685), 0, 255, 0, cols-1);
        int y2 = map(inoise8(strip.now * speed, 355, 11685), 0, 255, 0, rows-1);

        int x3 = map(inoise8(strip.now * speed, 55355, 6685), 0, 255, 0, cols-1);
        int y3 = map(inoise8(strip.now * speed, 25355, 22685), 0, 255, 0, rows-1);

        // and one Lissajou function
        int x1 = beatsin8_t(23 * speed, 0, cols-1);
        int y1 = beatsin8_t(28 * speed, 0, rows-1);

        for (int y = 0; y < rows; y++) {
            for (int x = 0; x < cols; x++) {
                // calculate distances of the 3 points from actual pixel
                // and add them together with weightening
                unsigned dx = abs(x - x1);
                unsigned dy = abs(y - y1);
                unsigned dist = 2 * sqrt32_bw((dx * dx) + (dy * dy));

                dx = abs(x - x2);
                dy = abs(y - y2);
                dist += sqrt32_bw((dx * dx) + (dy * dy));

                dx = abs(x - x3);
                dy = abs(y - y3);
                dist += sqrt32_bw((dx * dx) + (dy * dy));

                // inverse result
                int color = dist ? 1000 / dist : 255;

                // map color between thresholds
                if (color > 0 and color < 60) {
                    buffer.setPixelColor(x, y, SEGMENT.color_from_palette(map(color * 9, 9, 531, 0, 255), false, PALETTE_SOLID_WRAP, 0));
                } else {
                    buffer.setPixelColor(x, y, SEGMENT.color_from_palette(0, false, PALETTE_SOLID_WRAP, 0));
                }
                // show the 3 points, too
                buffer.setPixelColor(x1, y1, WHITE);
                buffer.setPixelColor(x2, y2, WHITE);
                buffer.setPixelColor(x3, y3, WHITE);
            }
        }
    }

private:
};


#endif //WLED_DISABLE_2D
