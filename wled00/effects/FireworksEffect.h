#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Fireworks function.
 */
class FireworksEffect : public BaseEffect<FireworksEffect, BufferedEffect<EffectDimensionality::d2VStrips>> {
private:
    using Self = FireworksEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2VStrips>>;

public:
    static constexpr const char metaData[] PROGMEM = "Fireworks@,Frequency;!,!;!;12;ix=192,pal=11";
    static constexpr const uint8_t effectId = FX_MODE_FIREWORKS;

    explicit FireworksEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        const uint16_t width  = coordinate.width;
        const uint16_t height = coordinate.height;

        if (parameters.call == 0) {
            sparkIndex = UINT16_MAX;
            oldSparkIndex = UINT16_MAX;
        }
        buffer.fade(SEGCOLOR(1), 128);

        uint8_t x = sparkIndex%width, y = sparkIndex/width; // 2D coordinates stored in upper and lower byte
        if (!step) {
            // fireworks mode (blur flares)
            bool valid1 = (sparkIndex < width*height);
            bool valid2 = (oldSparkIndex < width*height);
            uint32_t sv1 = 0, sv2 = 0;
            if (valid1) sv1 = buffer.getPixelColor(x, y); // get spark color
            if (valid2) sv2 = buffer.getPixelColor(x, y);
            buffer.blur(16); // used in mode_rain()
            if (valid1) { buffer.setPixelColor(x, y, sv1); } // restore spark color after blur
            if (valid2) { buffer.setPixelColor(x, y, sv2); } // restore old spark color after blur
        }

        for (int i=0; i<max(1, width/20); i++) {
            if (hw_random8(129 - (parameters.intensity >> 1)) == 0) {
                uint16_t index = hw_random16(width*height);
                x = index % width;
                y = index / width;
                uint32_t col = parameters.color_from_palette(hw_random8(), false, false, 0);
                buffer.setPixelColor(x, y, col);
                oldSparkIndex = sparkIndex;  // old spark
                sparkIndex = index;        // remember where spark occurred
            }
        }
        return true;
    }

protected:
    uint32_t step{};
    uint16_t sparkIndex{};
    uint16_t oldSparkIndex{};
};


#endif //WLED_DISABLE_2D
