#pragma once

#include "../FX.h"
#include "Effect.h"
#include "FairyEffect.h"

/*
 * Fairytwinkle. Like Colortwinkle, but starting from all lit and not relying on strip.getPixelColor
 * Warning: Uses 4 bytes of segment data per pixel
 */
class FairytwinkleEffect : public BaseEffect<FairytwinkleEffect> {
private:
    using Self = FairytwinkleEffect;
    using Base = BaseEffect<FairytwinkleEffect>;

    public:
    static constexpr const char metaData[] PROGMEM = "Fairytwinkle@!,!;!,!;!;;m12=0";
    static constexpr const uint8_t effectId = FX_MODE_FAIRYTWINKLE;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d1;

    using Base::Base;

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!flashers.resize(coordinate.width)) {
            return false;
        }

        now16 = strip.now & 0xFFFF;
        PRNG16 = 5100 + strip.getCurrSegmentId();

        riseFallTime = 400 + (255-SEGMENT.speed)*3;
        maxDur = riseFallTime/100 + ((255 - SEGMENT.intensity) >> 2) + 13 + ((255 - SEGMENT.intensity) >> 1);
        return true;
    }

    uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        const unsigned f = coordinate.getXAbsolute();
        uint16_t stateTime = now16 - flashers[f].stateStart;
        //random on/off time reached, switch state
        if (stateTime > flashers[f].stateDur * 100) {
            flashers[f].stateOn = !flashers[f].stateOn;
            bool init = !flashers[f].stateDur;
            if (flashers[f].stateOn) {
                flashers[f].stateDur = riseFallTime/100 + ((255 - SEGMENT.intensity) >> 2) + hw_random8(12 + ((255 - SEGMENT.intensity) >> 1)) +1;
            } else {
                flashers[f].stateDur = riseFallTime/100 + hw_random8(3 + ((255 - SEGMENT.speed) >> 6)) +1;
            }
            flashers[f].stateStart = now16;
            stateTime = 0;
            if (init) {
                flashers[f].stateStart -= riseFallTime; //start lit
                flashers[f].stateDur = riseFallTime/100 + hw_random8(12 + ((255 - SEGMENT.intensity) >> 1)) +5; //fire up a little quicker
                stateTime = riseFallTime;
            }
        }
        if (flashers[f].stateOn && flashers[f].stateDur > maxDur) flashers[f].stateDur = maxDur; //react more quickly on intensity change
        if (stateTime > riseFallTime) stateTime = riseFallTime; //for flasher brightness calculation, fades in first 255 ms of state
        unsigned fadeprog = 255 - ((stateTime * 255) / riseFallTime);
        uint8_t flasherBri = (flashers[f].stateOn) ? 255-gamma8(fadeprog) : gamma8(fadeprog);
        unsigned lastR = PRNG16;
        unsigned diff = 0;
        while (diff < 0x4000) { //make sure colors of two adjacent LEDs differ enough
            PRNG16 = (uint16_t)(PRNG16 * 2053) + 1384; //next 'random' number
            diff = (PRNG16 > lastR) ? PRNG16 - lastR : lastR - PRNG16;
        }
        return color_blend(SEGCOLOR(1), SEGMENT.color_from_palette(PRNG16 >> 8, false, false, 0), flasherBri);
    }

private:
    SegmentAllocator<FairyEffect::Flasher>::vector flashers{};
    unsigned now16;
    uint16_t PRNG16;
    unsigned riseFallTime;
    unsigned maxDur;
};


