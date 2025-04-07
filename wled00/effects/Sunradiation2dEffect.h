#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//////////////////////////////
//     2D Sun Radiation     //
//////////////////////////////
// By: ldirko https://editor.soulmatelights.com/gallery/599-sun-radiation  , modified by: Andrew Tuline
class Sunradiation2dEffect : public BaseEffect<Sunradiation2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = Sunradiation2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char* const metaData = "Sun Radiation@Variance,Brightness;;;2";
    static constexpr const uint8_t effectId = FX_MODE_2DSUNRADIATION;

    explicit Sunradiation2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        if (!resizeVector(bump, (cols+2)*(rows+2))) {
            return;
        }

        if (SEGENV.call == 0) {
            buffer.fill(BLACK);
        }

        unsigned long t = strip.now / 4;
        unsigned index = 0;
        uint8_t someVal = SEGMENT.speed/4;             // Was 25.
        for (int j = 0; j < (rows + 2); j++) {
            for (int i = 0; i < (cols + 2); i++) {
                byte col = (inoise8_raw(i * someVal, j * someVal, t)) / 2;
                bump[index++] = col;
            }
        }

        int yindex = cols + 3;
        int vly = -(rows / 2 + 1);
        for (int y = 0; y < rows; y++) {
            ++vly;
            int vlx = -(cols / 2 + 1);
            for (int x = 0; x < cols; x++) {
                ++vlx;
                int nx = bump[x + yindex + 1] - bump[x + yindex - 1];
                int ny = bump[x + yindex + (cols + 2)] - bump[x + yindex - (cols + 2)];
                unsigned difx = abs8(vlx * 7 - nx);
                unsigned dify = abs8(vly * 7 - ny);
                int temp = difx * difx + dify * dify;
                int col = 255 - temp / 8; //8 its a size of effect
                if (col < 0) col = 0;
                const CRGB heatColor = HeatColor(col / (3.0f-(float)(SEGMENT.intensity)/128.f));
                buffer.setPixelColor(x, y, RGBW32(heatColor.r, heatColor.g, heatColor.b, 0));
            }
            yindex += (cols + 2);
        }
    }

private:
    SegmentAllocator<byte>::vector bump{};
};


#endif //WLED_DISABLE_2D
