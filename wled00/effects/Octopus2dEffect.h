#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//Idea from https://www.youtube.com/watch?v=HsA-6KIbgto&ab_channel=GreatScott%21
//Octopus (https://editor.soulmatelights.com/gallery/671-octopus)
//Stepko and Sutaburosu
// adapted for WLED by @blazoncek
class Octopus2dEffect : public BaseEffect<Octopus2dEffect> {
private:
    struct Map {
        uint8_t angle{};
        uint8_t radius{};
    };

private:
    using Self = Octopus2dEffect;
    using Base = BaseEffect<Self>;

public:
    static constexpr const char* const metaData = "Octopus@!,,Offset X,Offset Y,Legs,fasttan;;!;2;";
    static constexpr const uint8_t effectId = FX_MODE_2DOCTOPUS;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d2;

    using Base::Base;

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        const int cols = coordinate.width;
        const int rows = coordinate.height;
        const uint8_t mapp = 180 / MAX(cols,rows);

        if (!resizeVector(rMap, coordinate.width * coordinate.height)) {
            return;
        }

        // re-init if SEGMENT dimensions or offset changed
        if (SEGENV.call == 0 || aux0 != cols || aux1 != rows || SEGMENT.custom1 != offsX || SEGMENT.custom2 != offsY) {
            step = 0; // t
            aux0 = cols;
            aux1 = rows;
            offsX = SEGMENT.custom1;
            offsY = SEGMENT.custom2;
            const int C_X = (cols / 2) + ((SEGMENT.custom1 - 128)*cols)/255;
            const int C_Y = (rows / 2) + ((SEGMENT.custom2 - 128)*rows)/255;
            for (int x = 0; x < cols; x++) {
                for (int y = 0; y < rows; y++) {
                    int dx = (x - C_X);
                    int dy = (y - C_Y);
                    const int mapIndex = XY(x, y, cols, rows);
                    rMap[mapIndex].angle  = int(40.7436f * atan2_t(dy, dx));  // avoid 128*atan2()/PI
                    rMap[mapIndex].radius = sqrtf(dx * dx + dy * dy) * mapp; //thanks Sutaburosu
                }
            }
        }

        step += SEGMENT.speed / 32 + 1;  // 1-4 range
    }

    uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        const int mapIndex = XY(coordinate);
        byte angle = rMap[mapIndex].angle;
        byte radius = rMap[mapIndex].radius;
        //CRGB c = CHSV(step / 2 - radius, 255, sin8_t(sin8_t((angle * 4 - radius) / 4 + step) + radius - step * 2 + angle * (SEGMENT.custom3/3+1)));
        unsigned intensity = sin8_t(sin8_t((angle * 4 - radius) / 4 + step/2) + radius - step + angle * (SEGMENT.custom3/4+1));
        intensity = map((intensity*intensity) & 0xFFFF, 0, 65535, 0, 255); // add a bit of non-linearity for cleaner display
        return ColorFromPalette(SEGPALETTE, step / 2 - radius, intensity);
    }

private:
    static inline constexpr int XY(int x, int y, int cols, int rows) {
        return (x%cols) + (y%rows) * cols;
    };
    static inline constexpr int XY(const EffectCoordinate& coordinate) {
        return XY(coordinate.getXAbsolute(), coordinate.getYAbsolute(), coordinate.width, coordinate.height);
    };

private:
    std::vector<Map> rMap{};
    uint8_t offsX{};
    uint8_t offsY{};

    uint32_t step{};
    uint16_t aux0{};
    uint16_t aux1{};
};

#endif //WLED_DISABLE_2D
