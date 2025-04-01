#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Percentage display
 * Intensity values from 0-100 turn on the leds.
 */
class PercentEffect : public BaseEffect<PercentEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = PercentEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char* const metaData = "Percent@,% of fill,,,,One color;!,!;!";
    static constexpr const uint8_t effectId = FX_MODE_PERCENT;

    explicit PercentEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);


        unsigned percent = SEGMENT.intensity;
        percent = constrain(percent, 0, 200);
        unsigned active_leds = (percent < 100) ? roundf(coordinate.width * percent / 100.0f)
                                                                                     : roundf(coordinate.width * (200 - percent) / 100.0f);
        const bool oneColor = SEGMENT.check1;

        unsigned size = (1 + ((SEGMENT.speed * coordinate.width) >> 11));
        if (SEGMENT.speed == 255) size = 255;

        if (percent <= 100) {
            for (unsigned i = 0; i < coordinate.width; i++) {
            	if (i < aux1) {
                    if (oneColor)
                        buffer.setPixelColor(i, SEGMENT.color_from_palette(map(percent,0,100,0,255), false, false, 0));
                    else
                        buffer.setPixelColor(i, SEGMENT.color_from_palette(i, true, PALETTE_SOLID_WRAP, 0));
            	}
            	else {
                    buffer.setPixelColor(i, SEGCOLOR(1));
            	}
            }
        } else {
            for (unsigned i = 0; i < coordinate.width; i++) {
            	if (i < (coordinate.width - aux1)) {
                    buffer.setPixelColor(i, SEGCOLOR(1));
            	}
            	else {
                    if (oneColor)
                        buffer.setPixelColor(i, SEGMENT.color_from_palette(map(percent,100,200,255,0), false, false, 0));
                    else
                        buffer.setPixelColor(i, SEGMENT.color_from_palette(i, true, PALETTE_SOLID_WRAP, 0));
            	}
            }
        }

        if(active_leds > aux1) {  // smooth transition to the target value
            aux1 += size;
            if (aux1 > active_leds) aux1 = active_leds;
        } else if (active_leds < aux1) {
            if (aux1 > size) aux1 -= size; else aux1 = 0;
            if (aux1 < active_leds) aux1 = active_leds;
        }
    }

private:
    uint16_t aux1{};
};


