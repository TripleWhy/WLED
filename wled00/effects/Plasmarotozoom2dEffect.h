#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/////////////////////////////
//  2D PLASMA ROTOZOOMER   //
/////////////////////////////
// Plasma Rotozoomer by ldirko (c)2020 [https://editor.soulmatelights.com/gallery/457-plasma-rotozoomer], adapted for WLED by Blaz Kristan (AKA blazoncek)
class Plasmarotozoom2dEffect : public BaseEffect<Plasmarotozoom2dEffect> {
private:
    using Self = Plasmarotozoom2dEffect;
    using Base = BaseEffect<Self>;

public:
    static constexpr const char metaData[] PROGMEM = "Rotozoomer@!,Scale,,,,Alt;;!;2;pal=54";
    static constexpr const uint8_t effectId = FX_MODE_2DPLASMAROTOZOOM;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d2;

    using Base::Base;

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        const int cols = coordinate.width;
        const int rows = coordinate.height;

        if (!plasma.resize(coordinate.width)) {
            return false;
        }

        unsigned ms = strip.now/15;

        // plasma
        for (int j = 0; j < rows; j++) {
            int index = j*cols;
            for (int i = 0; i < cols; i++) {
                if (parameters.check1) plasma[index+i] = (i * 4 ^ j * 4) + ms / 6;
                else                plasma[index+i] = inoise8(i * 40, j * 40, ms);
            }
        }

        // rotozoom
        float f       = (sin_t(a/2)+((128-parameters.intensity)/128.0f)+1.1f)/1.5f;  // scale factor
        kosinus = cos_t(a) * f;
        sinus   = sin_t(a) * f;

        a -= 0.03f + float(parameters.speed-128)*0.0002f;  // rotation speed
        if(a < -6283.18530718f)
            a += 6283.18530718f; // 1000*2*PI, protect sin/cos from very large input float values (will give wrong results)
        return true;
    }

    void nextRowImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        const int i = coordinate.getYAbsolute();
        u1 = i * kosinus;
        v1 = i * sinus;
    }

    uint32_t getPixelColorImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        const int j = coordinate.getXAbsolute();
        const int cols = coordinate.width;
        const int rows = coordinate.height;

        byte u = abs8(u1 - j * sinus) % cols;
        byte v = abs8(v1 + j * kosinus) % rows;
        return SEGMENT.color_from_palette(plasma[v*cols+u], false, PALETTE_SOLID_WRAP, 255);
    }

private:
    float a{};
    SegmentAllocator<byte>::vector plasma{};

    float kosinus{};
    float sinus{};
    float u1{};
    float v1{};
};

#endif // WLED_DISABLE_2D
