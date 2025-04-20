#pragma once
#ifndef WLED_DISABLE_2D

#include <array>
#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/////////////////////////
//     2D Crazy Bees   //
/////////////////////////
//// Crazy bees by stepko (c)12.02.21 [https://editor.soulmatelights.com/gallery/651-crazy-bees], adapted by Blaz Kristan (AKA blazoncek), improved by @dedehai
class Crazybees2dEffect : public BaseEffect<Crazybees2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    static constexpr int MAX_BEES = 5;

    struct Bee {
        uint8_t posX{};
        uint8_t posY{};
        uint8_t aimX{};
        uint8_t aimY{};
        uint8_t hue;
        int8_t deltaX{};
        int8_t deltaY{};
        int8_t signX{};
        int8_t signY{};
        int8_t error{};
        void aimed(uint16_t w, uint16_t h) {
            //prng.setSeed(millis());
            aimX   = prng.random8(1u, w-1);
            aimY   = prng.random8(1u, h-1);
            hue    = prng.random8();
            deltaX = abs(aimX - posX);
            deltaY = abs(aimY - posY);
            signX  = posX < aimX ? 1 : -1;
            signY  = posY < aimY ? 1 : -1;
            error  = deltaX - deltaY;
        };
    };

private:
    using Self = Crazybees2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "Crazy Bees@!,Blur,,,,Smear;;!;2;pal=11,ix=0";
    static constexpr const uint8_t effectId = FX_MODE_2DCRAZYBEES;

    explicit Crazybees2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if ((coordinate.width < 3) || (coordinate.height < 3)) {
            return false;
        }
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        const byte n = MIN(MAX_BEES, (rows * cols) / 256 + 1);

        if (SEGENV.call == 0) {
            prng.setSeed(strip.now);
            for (size_t i = 0; i < n; i++) {
                bee[i].posX = prng.random8(0, cols);
                bee[i].posY = prng.random8(0, rows);
                bee[i].aimed(cols, rows);
            }
        }

        if (strip.now > step) {
            step = strip.now + (FRAMETIME * 16 / ((SEGMENT.speed>>4)+1));
            buffer.fadeToBlackBy(32 + ((SEGMENT.check1*SEGMENT.intensity) / 25));
            buffer.blur(SEGMENT.intensity / (2 + SEGMENT.check1 * 9), SEGMENT.check1);
            for (size_t i = 0; i < n; i++) {
                uint32_t flowerCcolor = SEGMENT.color_from_palette(bee[i].hue, false, true, 255);
                buffer.addPixelColor(bee[i].aimX + 1, bee[i].aimY, flowerCcolor);
                buffer.addPixelColor(bee[i].aimX, bee[i].aimY + 1, flowerCcolor);
                buffer.addPixelColor(bee[i].aimX - 1, bee[i].aimY, flowerCcolor);
                buffer.addPixelColor(bee[i].aimX, bee[i].aimY - 1, flowerCcolor);
                if (bee[i].posX != bee[i].aimX || bee[i].posY != bee[i].aimY) {
                    const CRGB rgb(CHSV(bee[i].hue, 60, 255));
                    buffer.setPixelColor(bee[i].posX, bee[i].posY, RGBW32(rgb.r, rgb.g, rgb.b, 0));
                    int error2 = bee[i].error * 2;
                    if (error2 > -bee[i].deltaY) {
                        bee[i].error -= bee[i].deltaY;
                        bee[i].posX += bee[i].signX;
                    }
                    if (error2 < bee[i].deltaX) {
                        bee[i].error += bee[i].deltaX;
                        bee[i].posY += bee[i].signY;
                    }
                } else {
                    bee[i].aimed(cols, rows);
                }
            }
        }
        return true;
    }

private:
    std::array<Bee, MAX_BEES> bee{};
    uint32_t step{};
};

#endif //WLED_DISABLE_2D
