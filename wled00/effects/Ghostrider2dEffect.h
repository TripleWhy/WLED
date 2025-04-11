#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/////////////////////////
//     2D Ghost Rider  //
/////////////////////////
//// Ghost Rider by stepko (c)2021 [https://editor.soulmatelights.com/gallery/716-ghost-rider], adapted by Blaz Kristan (AKA blazoncek)
#define LIGHTERS_AM 64  // max lighters (adequate for 32x32 matrix)
class Ghostrider2dEffect : public BaseEffect<Ghostrider2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = Ghostrider2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "Ghost Rider@Fade rate,Blur;;!;2";
    static constexpr const uint8_t effectId = FX_MODE_2DGHOSTRIDER;

    explicit Ghostrider2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        const size_t maxLighters = min(cols + rows, LIGHTERS_AM);

        if (aux0 != cols || aux1 != rows) {
            aux0 = cols;
            aux1 = rows;
            angleSpeed = hw_random8(0,20) - 10;
            gAngle = hw_random16();
            Vspeed = 5;
            gPosX = (cols/2) * 10;
            gPosY = (rows/2) * 10;
            for (size_t i = 0; i < maxLighters; i++) {
                lightersPosX[i] = gPosX;
                lightersPosY[i] = gPosY + i;
                time[i] = i * 2;
                reg[i] = false;
            }
        }

        if (strip.now > step) {
            step = strip.now + 1024 / (cols+rows);

            buffer.fadeToBlackBy((SEGMENT.speed>>2)+64);

            CRGB color = CRGB::White;
            buffer.wuPixel(gPosX * 256 / 10, gPosY * 256 / 10, color);

            gPosX += Vspeed * sin_t(radians(gAngle));
            gPosY += Vspeed * cos_t(radians(gAngle));
            gAngle += angleSpeed;
            if (gPosX < 0)               gPosX = (cols - 1) * 10;
            if (gPosX > (cols - 1) * 10) gPosX = 0;
            if (gPosY < 0)               gPosY = (rows - 1) * 10;
            if (gPosY > (rows - 1) * 10) gPosY = 0;
            for (size_t i = 0; i < maxLighters; i++) {
                time[i] += hw_random8(5, 20);
                if (time[i] >= 255 ||
                    (lightersPosX[i] <= 0) ||
                        (lightersPosX[i] >= (cols - 1) * 10) ||
                        (lightersPosY[i] <= 0) ||
                        (lightersPosY[i] >= (rows - 1) * 10)) {
                    reg[i] = true;
                }
                if (reg[i]) {
                    lightersPosY[i] = gPosY;
                    lightersPosX[i] = gPosX;
                    Angle[i] = gAngle + ((int)hw_random8(20) - 10);
                    time[i] = 0;
                    reg[i] = false;
                } else {
                    lightersPosX[i] += -7 * sin_t(radians(Angle[i]));
                    lightersPosY[i] += -7 * cos_t(radians(Angle[i]));
                }
                buffer.wuPixel(lightersPosX[i] * 256 / 10, lightersPosY[i] * 256 / 10, ColorFromPalette(SEGPALETTE, (256 - time[i])));
            }
            buffer.blur(SEGMENT.intensity>>3);
        }
        return true;
    }

private:
    int16_t  gPosX{};
    int16_t  gPosY{};
    uint16_t gAngle{};
    int8_t   angleSpeed{};
    uint16_t lightersPosX[LIGHTERS_AM]{};
    uint16_t lightersPosY[LIGHTERS_AM]{};
    uint16_t Angle[LIGHTERS_AM]{};
    uint16_t time[LIGHTERS_AM]{};
    bool     reg[LIGHTERS_AM]{};
    int8_t   Vspeed{};

    uint32_t step{};
    uint16_t aux0{};
    uint16_t aux1{};
};
#undef LIGHTERS_AM

