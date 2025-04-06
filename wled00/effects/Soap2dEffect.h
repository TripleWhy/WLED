#pragma once
#ifndef WLED_DISABLE_2D

#include <array>
#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//Soap
//@Stepko
//Idea from https://www.youtube.com/watch?v=DiHBgITrZck&ab_channel=StefanPetrick
// adapted for WLED by @blazoncek, optimization by @dedehai
class Soap2dEffect : public BaseEffect<Soap2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    struct NoisePixel {
        uint8_t noise3d;
        CRGB    pixel;
    };

private:
    using Self = Soap2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char* const metaData = "Soap@!,Smoothness,Density;;!;2;pal=11";
    static constexpr const uint8_t effectId = FX_MODE_2DSOAP;

    explicit Soap2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        if (!resizeVector(noisePixels, coordinate.width * coordinate.height)) {
            return;
        }

        const uint32_t scale32_x = 160000U/cols;
        const uint32_t scale32_y = 160000U/rows;
        const uint32_t mov = MIN(cols,rows)*(SEGMENT.speed+2)/2;
        const uint8_t  smoothness = MIN(250,SEGMENT.intensity); // limit as >250 produces very little changes

        if (SEGENV.call == 0) for (int i = 0; i < 3; i++) noisecoord[i] = hw_random(); // init
        else                  for (int i = 0; i < 3; i++) noisecoord[i] += mov;

        for (int i = 0; i < cols; i++) {
            int32_t ioffset = scale32_x * (i - cols / 2);
            for (int j = 0; j < rows; j++) {
                int32_t joffset = scale32_y * (j - rows / 2);
                uint8_t data = inoise16(noisecoord[0] + ioffset, noisecoord[1] + joffset, noisecoord[2]) >> 8;
                noisePixels[XY(coordinate,i,j)].noise3d = scale8(noisePixels[XY(coordinate,i,j)].noise3d, smoothness) + scale8(data, 255 - smoothness);
            }
        }
        // init also if dimensions changed
        if (SEGENV.call == 0 || aux0 != cols || aux1 != rows) {
            aux0 = cols;
            aux1 = rows;
            for (int i = 0; i < cols; i++) {
                for (int j = 0; j < rows; j++) {
                    buffer.setPixelColor(i, j, ColorFromPalette(SEGPALETTE,~noisePixels[XY(coordinate,i,j)].noise3d*3));
                }
            }
        }

        soapPixels(coordinate, true ); // rows
        soapPixels(coordinate, false); // cols
    }

private:
    static inline constexpr int XY (const EffectCoordinate& coordinate, int x, int y) {
            return x + y * coordinate.width;
    };

    void soapPixels(const EffectCoordinate& coordinate, bool isRow) {
        const int  cols = coordinate.width;
        const int  rows = coordinate.height;
        const int  tRC  = isRow ? rows : cols; // transpose if isRow
        const int  tCR  = isRow ? cols : rows; // transpose if isRow
        const int  amplitude = max(1, (tCR - 8) >> 3) * (1 + (SEGMENT.custom1 >> 5));
        const int  shift = 0; //(128 - SEGMENT.custom2)*2;

        CRGB ledsbuff[tCR];

        for (int i = 0; i < tRC; i++) {
            int amount   = ((int)noisePixels[isRow ? i*cols : i].noise3d - 128) * amplitude + shift; // use first row/column: XY(0,i)/XY(i,0)
            int delta    = abs(amount) >> 8;
            int fraction = abs(amount) & 255;
            for (int j = 0; j < tCR; j++) {
                int zD, zF;
                if (amount < 0) {
                    zD = j - delta;
                    zF = zD - 1;
                } else {
                    zD = j + delta;
                    zF = zD + 1;
                }
                int yA = abs(zD)%tCR;
                int yB = abs(zF)%tCR;
                int xA = i;
                int xB = i;
                if (isRow) {
                    std::swap(xA,yA);
                    std::swap(xB,yB);
                }
                const int indxA = XY(coordinate,xA,yA);
                const int indxB = XY(coordinate,xB,yB);
                CRGB PixelA;
                CRGB PixelB;
                if ((zD >= 0) && (zD < tCR)) PixelA = noisePixels[indxA].pixel;
                else                         PixelA = ColorFromPalette(SEGPALETTE, ~noisePixels[indxA].noise3d*3);
                if ((zF >= 0) && (zF < tCR)) PixelB = noisePixels[indxB].pixel;
                else                         PixelB = ColorFromPalette(SEGPALETTE, ~noisePixels[indxB].noise3d*3);
                ledsbuff[j] = (PixelA.nscale8(ease8InOutApprox(255 - fraction))) + (PixelB.nscale8(ease8InOutApprox(fraction)));
            }
            for (int j = 0; j < tCR; j++) {
                CRGB c = ledsbuff[j];
                if (isRow)
                    std::swap(j,i);
                SEGMENT.setPixelColorXY(i, j, noisePixels[XY(coordinate,i,j)].pixel = c);
                if (isRow)
                    std::swap(j,i);
            }
        }
    }

private:
    std::vector<NoisePixel> noisePixels{};
    std::array<uint32_t, 3> noisecoord{};
    uint16_t aux0{};
    uint16_t aux1{};
};


#endif //WLED_DISABLE_2D
