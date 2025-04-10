#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Fire flicker function
 */
class FireFlickerEffect : public BaseEffect<FireFlickerEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = FireFlickerEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Fire Flicker@!,!;!;!;01";
    static constexpr const uint8_t effectId = FX_MODE_FIRE_FLICKER;

    explicit FireFlickerEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        uint32_t cycleTime = 40 + (255 - SEGMENT.speed);
        uint32_t it = strip.now / cycleTime;
        if (step == it) return;

        byte w = (SEGCOLOR(0) >> 24);
        byte r = (SEGCOLOR(0) >> 16);
        byte g = (SEGCOLOR(0) >>  8);
        byte b = (SEGCOLOR(0)      );
        byte lum = (SEGMENT.palette == 0) ? MAX(w, MAX(r, MAX(g, b))) : 255;
        lum /= (((256-SEGMENT.intensity)/16)+1);
        for (unsigned i = 0; i < coordinate.width; i++) {
            byte flicker = hw_random8(lum);
            if (SEGMENT.palette == 0) {
                buffer.setPixelColor(i, RGBW32(MAX(r - flicker, 0), MAX(g - flicker, 0), MAX(b - flicker, 0), MAX(w - flicker, 0)));
            } else {
                buffer.setPixelColor(i, SEGMENT.color_from_palette(i, true, PALETTE_SOLID_WRAP, 0, 255 - flicker));
            }
        }

        step = it;
    }

private:
    uint32_t step{};
};


