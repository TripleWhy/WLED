#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

////////////////////////////
//     2D Floating Blobs  //
////////////////////////////
//// Floating Blobs by stepko (c)2021 [https://editor.soulmatelights.com/gallery/573-blobs], adapted by Blaz Kristan (AKA blazoncek)
class Floatingblobs2dEffect : public BaseEffect<Floatingblobs2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    static constexpr size_t MAX_BLOBS = 8;

    using Self = Floatingblobs2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "Blobs@!,# blobs,Blur,Trail;!;!;2;c1=8";
    static constexpr const uint8_t effectId = FX_MODE_2DBLOBS;

    explicit Floatingblobs2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        size_t Amount = (parameters.intensity>>5) + 1; // NOTE: be sure to update MAX_BLOBS if you change this

        if (aux0 != cols || aux1 != rows) {
            aux0 = cols; // re-initialise if virtual size changes
            aux1 = rows;
            //buffer.fill(BLACK);
            for (size_t i = 0; i < MAX_BLOBS; i++) {
                r[i]  = hw_random8(1, cols>8 ? (cols/4) : 2);
                sX[i] = (float) hw_random8(3, cols) / (float)(256 - parameters.speed); // speed x
                sY[i] = (float) hw_random8(3, rows) / (float)(256 - parameters.speed); // speed y
                x[i]  = hw_random8(0, cols-1);
                y[i]  = hw_random8(0, rows-1);
                color[i] = hw_random8();
                grow[i]  = (r[i] < 1.f);
                if (sX[i] == 0) sX[i] = 1;
                if (sY[i] == 0) sY[i] = 1;
            }
        }

        buffer.fadeToBlackBy((parameters.custom2>>3)+1);

        // Bounce balls around
        for (size_t i = 0; i < Amount; i++) {
            if (step < strip.now) color[i] += 4; // slowly change color
            // change radius if needed
            if (grow[i]) {
                // enlarge radius until it is >= 4
                r[i] += (fabsf(sX[i]) > fabsf(sY[i]) ? fabsf(sX[i]) : fabsf(sY[i])) * 0.05f;
                if (r[i] >= MIN(cols/4.f,2.f)) {
                    grow[i] = false;
                }
            } else {
                // reduce radius until it is < 1
                r[i] -= (fabsf(sX[i]) > fabsf(sY[i]) ? fabsf(sX[i]) : fabsf(sY[i])) * 0.05f;
                if (r[i] < 1.f) {
                    grow[i] = true;
                }
            }
            uint32_t c = SEGMENT.color_from_palette(color[i], false, false, 0);
            if (r[i] > 1.f) buffer.fillCircle(roundf(x[i]), roundf(y[i]), roundf(r[i]), c);
            else            buffer.setPixelColor((int)roundf(x[i]), (int)roundf(y[i]), c);
            // move x
            if (x[i] + r[i] >= cols - 1) x[i] += (sX[i] * ((cols - 1 - x[i]) / r[i] + 0.005f));
            else if (x[i] - r[i] <= 0)   x[i] += (sX[i] * (x[i] / r[i] + 0.005f));
            else                         x[i] += sX[i];
            // move y
            if (y[i] + r[i] >= rows - 1) y[i] += (sY[i] * ((rows - 1 - y[i]) / r[i] + 0.005f));
            else if (y[i] - r[i] <= 0)   y[i] += (sY[i] * (y[i] / r[i] + 0.005f));
            else                         y[i] += sY[i];
            // bounce x
            if (x[i] < 0.01f) {
                sX[i] = (float)hw_random8(3, cols) / (256 - parameters.speed);
                x[i]  = 0.01f;
            } else if (x[i] > (float)cols - 1.01f) {
                sX[i] = (float)hw_random8(3, cols) / (256 - parameters.speed);
                sX[i] = -sX[i];
                x[i]  = (float)cols - 1.01f;
            }
            // bounce y
            if (y[i] < 0.01f) {
                sY[i] = (float)hw_random8(3, rows) / (256 - parameters.speed);
                y[i]  = 0.01f;
            } else if (y[i] > (float)rows - 1.01f) {
                sY[i] = (float)hw_random8(3, rows) / (256 - parameters.speed);
                sY[i] = -sY[i];
                y[i]  = (float)rows - 1.01f;
            }
        }
        buffer.blur(parameters.custom1>>2);

        if (step < strip.now) step = strip.now + 2000; // change colors every 2 seconds
        return true;
    }

private:
    float x[MAX_BLOBS], y[MAX_BLOBS];
    float sX[MAX_BLOBS], sY[MAX_BLOBS]; // speed
    float r[MAX_BLOBS];
    bool grow[MAX_BLOBS];
    byte color[MAX_BLOBS];

    uint32_t step{};
    uint16_t aux0{};
    uint16_t aux1{};
};

#endif //WLED_DISABLE_2D
