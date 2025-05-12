#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//TODO
class LightningEffect : public BaseEffect<LightningEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = LightningEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Lightning@!,!,,,,,Overlay;!,!;!";
    static constexpr const uint8_t effectId = FX_MODE_LIGHTNING;

    explicit LightningEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        unsigned ledstart = hw_random16(coordinate.width);               // Determine starting location of flash
        unsigned ledlen = 1 + hw_random16(coordinate.width -ledstart);   // Determine length of flash (not to go beyond NUM_LEDS-1)
        uint8_t bri = 255/hw_random8(1, 3);

        if (aux1 == 0) //init, leader flash
        {
            aux1 = hw_random8(4, 4 + parameters.intensity/20); //number of flashes
            aux1 *= 2;

            bri = 52; //leader has lower brightness
            aux0 = 200; //200ms delay after leader
        }

        if (!parameters.check2) buffer.fill(SEGCOLOR(1));

        if (aux1 > 3 && !(aux1 & 0x01)) { //flash on even number >2
            for (unsigned i = ledstart; i < ledstart + ledlen; i++)
            {
                buffer.setPixelColor(i,parameters.color_from_palette(i, true, PALETTE_SOLID_WRAP, 0, bri));
            }
            aux1--;

            step = strip.now;
            //return hw_random8(4, 10); // each flash only lasts one frame/every 24ms... originally 4-10 milliseconds
        } else {
            if (strip.now - step > aux0) {
                aux1--;
                if (aux1 < 2) aux1 = 0;

                aux0 = (50 + hw_random8(100)); //delay between flashes
                if (aux1 == 2) {
                    aux0 = (hw_random8(255 - parameters.speed) * 100); // delay between strikes
                }
                step = strip.now;
            }
        }
        return true;
    }

private:
    uint32_t step{};
    uint16_t aux0{};
    uint16_t aux1{};
};


