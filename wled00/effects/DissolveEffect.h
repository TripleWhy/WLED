#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Dissolve function: Blink several LEDs on and then off
 */
// Making this effect buffered appears to have been a fix for #4401
class DissolveEffect : public BaseEffect<DissolveEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = DissolveEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Dissolve@Repeat speed,Dissolve speed,,,,Random;!,!;!";
    static constexpr const uint8_t effectId = FX_MODE_DISSOLVE;

    explicit DissolveEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        const uint32_t color = SEGMENT.check1 ? SEGMENT.color_wheel(hw_random8()) : SEGCOLOR(0);

        if (SEGENV.call == 0) {
            buffer.fill(SEGCOLOR(1));
            dissolveToPrimary = true;
        }

        for (unsigned j = 0; j <= coordinate.width / 15; j++) {
            if (hw_random8() <= SEGMENT.intensity) {
                for (size_t times = 0; times < 10; times++) { //attempt to spawn a new pixel 10 times
                    unsigned i = hw_random16(coordinate.width);
                    if (dissolveToPrimary) { //dissolve to primary/palette
                        if (buffer.getPixelColor(i) == SEGCOLOR(1)) {
                            buffer.setPixelColor(i, color == SEGCOLOR(0) ? SEGMENT.color_from_palette(i, true, PALETTE_SOLID_WRAP, 0) : color);
                            break; //only spawn 1 new pixel per frame per 50 LEDs
                        }
                    } else { //dissolve to secondary
                        if (buffer.getPixelColor(i) != SEGCOLOR(1)) {
                            buffer.setPixelColor(i, SEGCOLOR(1));
                            break;
                        }
                    }
                }
            }
        }

        if (step > (255 - SEGMENT.speed) + 15U) {
            dissolveToPrimary = !dissolveToPrimary;
            step = 0;
        } else {
            step++;
        }
    }

private:
    uint32_t step{};
    bool dissolveToPrimary{};
};
