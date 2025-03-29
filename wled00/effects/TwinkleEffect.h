#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Blink several LEDs in random colors on, reset, repeat.
 * Inspired by www.tweaking4all.com/hardware/arduino/adruino-led-strip-effects/
 */
class TwinkleEffect : public BaseEffect<TwinkleEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = TwinkleEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char* const metaData = "Twinkle@!,!;!,!;!;;m12=0"; //pixels
    static constexpr const uint8_t effectId = FX_MODE_TWINKLE;
    static constexpr const uint8_t defaultPaletteId = 0u;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d1;

    explicit TwinkleEffect(const EffectInformation& ei) : Base{ei, true} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        buffer.fade(SEGCOLOR(1), 224);

        uint32_t cycleTime = 20 + (255 - SEGMENT.speed)*5;
        uint32_t it = strip.now / cycleTime;
        if (it != step)
        {
            unsigned maxOn = map(SEGMENT.intensity, 0, 255, 1, coordinate.width); // make sure at least one LED is on
            if (onCounter >= maxOn)
            {
                onCounter = 0;
                PRNG16 = hw_random(); //new seed for our PRNG
            }
            onCounter++;
            step = it;
        }

        for (unsigned i = 0; i < onCounter; i++)
        {
            PRNG16 = (uint16_t)(PRNG16 * 2053) + 13849; // next 'random' number
            uint32_t p = (uint32_t)coordinate.width * (uint32_t)PRNG16;
            unsigned j = p >> 16;
            setBufferPixelColor(j, SEGMENT.color_from_palette(j, true, PALETTE_SOLID_WRAP, 0));
        }
    }

private:
    uint32_t step{};
    uint16_t onCounter{};
    uint16_t PRNG16;
};
