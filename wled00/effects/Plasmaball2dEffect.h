#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//////////////////////////////
//     2D Plasma Ball       //
//////////////////////////////
// By: Stepko https://editor.soulmatelights.com/gallery/659-plasm-ball , Modified by: Andrew Tuline
class Plasmaball2dEffect : public BaseEffect<Plasmaball2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = Plasmaball2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "Plasma Ball@Speed,,Fade,Blur;;!;2";
    static constexpr const uint8_t effectId = FX_MODE_2DPLASMABALL;

    explicit Plasmaball2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        buffer.fadeToBlackBy(parameters.custom1>>2);
        uint_fast32_t t = (strip.now * 8) / (256 - parameters.speed);  // optimized to avoid float
        for (int i = 0; i < cols; i++) {
            unsigned thisVal = perlin8(i * 30, t, t);
            unsigned thisMax = map(thisVal, 0, 255, 0, cols-1);
            for (int j = 0; j < rows; j++) {
                unsigned thisVal_ = perlin8(t, j * 30, t);
                unsigned thisMax_ = map(thisVal_, 0, 255, 0, rows-1);
                int x = (i + thisMax_ - cols / 2);
                int y = (j + thisMax - cols / 2);
                int cx = (i + thisMax_);
                int cy = (j + thisMax);

                buffer.addPixelColor(i, j, ((x - y > -2) && (x - y < 2)) ||
                                                                            ((cols - 1 - x - y) > -2 && (cols - 1 - x - y < 2)) ||
                                                                            (cols - cx == 0) ||
                                                                            (cols - 1 - cx == 0) ||
                                                                            ((rows - cy == 0) ||
                                                                            (rows - 1 - cy == 0)) ? ColorFromPalette(SEGPALETTE, beat8(5), thisVal, LINEARBLEND) : CRGB::Black);
            }
        }
        buffer.blur(parameters.custom2>>5);
        return true;
    }

private:
};


#endif //WLED_DISABLE_2D
