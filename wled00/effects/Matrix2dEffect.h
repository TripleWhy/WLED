#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

///////////////////////
//    2D Matrix      //
///////////////////////
// Matrix2D. By Jeremy Williams. Adapted by Andrew Tuline & improved by merkisoft and ewowi, and softhack007.
class Matrix2dEffect : public BaseEffect<Matrix2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = Matrix2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "Matrix@!,Spawning rate,Trail,,,Custom color;Spawn,Trail;;2";
    static constexpr const uint8_t effectId = FX_MODE_2DMATRIX;

    explicit Matrix2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        const int cols = coordinate.width;
        const int rows = coordinate.height;
        const auto XY = [&](int x, int y) { return (x%cols) + (y%rows) * cols; };

        unsigned dataSize = (coordinate.width+7) >> 3; //1 bit per LED for trails
        if (!data.resize(dataSize)) {
            return false;
        }

        if (parameters.call == 0) {
            buffer.fill(BLACK);
            step = 0;
        }

        uint8_t fade = map(parameters.custom1, 0, 255, 50, 250);    // equals trail size
        uint8_t speed = (256-parameters.speed) >> map(min(rows, 150), 0, 150, 0, 3);    // slower speeds for small displays

        uint32_t spawnColor;
        uint32_t trailColor;
        if (parameters.check1) {
            spawnColor = SEGCOLOR(0);
            trailColor = SEGCOLOR(1);
        } else {
            spawnColor = RGBW32(175,255,175,0);
            trailColor = RGBW32(27,130,39,0);
        }

        bool emptyScreen = true;
        if (strip.now - step >= speed) {
            step = strip.now;
            // move pixels one row down. Falling codes keep color and add trail pixels; all others pixels are faded
            // TODO: it would be better to paint trails idividually instead of relying on fadeToBlackBy()
            buffer.fadeToBlackBy(fade);
            for (int row = rows-1; row >= 0; row--) {
                for (int col = 0; col < cols; col++) {
                    unsigned index = XY(col, row) >> 3;
                    unsigned bitNum = XY(col, row) & 0x07;
                    if (bitRead(data[index], bitNum)) {
                        buffer.setPixelColor(col, row, trailColor);  // create trail
                        bitClear(data[index], bitNum);
                        if (row < rows-1) {
                            buffer.setPixelColor(col, row+1, spawnColor);
                            index = XY(col, row+1) >> 3;
                            bitNum = XY(col, row+1) & 0x07;
                            bitSet(data[index], bitNum);
                            emptyScreen = false;
                        }
                    }
                }
            }

            // spawn new falling code
            if (hw_random8() <= parameters.intensity || emptyScreen) {
                uint8_t spawnX = hw_random8(cols);
                buffer.setPixelColor(spawnX, 0, spawnColor);
                // update hint for next run
                unsigned index = XY(spawnX, 0) >> 3;
                unsigned bitNum = XY(spawnX, 0) & 0x07;
                bitSet(data[index], bitNum);
            }
        }
        return true;
    }

private:
    SegmentAllocator<byte>::vector data{};
    uint32_t step{};
};


#endif //WLED_DISABLE_2D
