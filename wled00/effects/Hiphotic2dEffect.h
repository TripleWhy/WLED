#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/////////////////////////
//     2D Hiphotic     //
/////////////////////////
//  By: ldirko  https://editor.soulmatelights.com/gallery/810 , Modified by: Andrew Tuline
class Hiphotic2dEffect : public BaseEffect<Hiphotic2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = Hiphotic2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "Hiphotic@X scale,Y scale,,,Speed;!;!;2";
    static constexpr const uint8_t effectId = FX_MODE_2DHIPHOTIC;

    explicit Hiphotic2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        const int cols = coordinate.width;
        const int rows = coordinate.height;
        const uint32_t a = strip.now / ((SEGMENT.custom3>>1)+1);

        for (int x = 0; x < cols; x++) {
            for (int y = 0; y < rows; y++) {
                buffer.setPixelColor(x, y, SEGMENT.color_from_palette(sin8_t(cos8_t(x * SEGMENT.speed/16 + a / 3) + sin8_t(y * SEGMENT.intensity/16 + a / 4) + a), false, PALETTE_SOLID_WRAP, 0));
            }
        }
        return true;
    }

private:
};


#endif //WLED_DISABLE_2D
