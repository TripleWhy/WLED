#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Fairy, inspired by https://www.youtube.com/watch?v=zeOw5MZWq24
 */

#define FLASHERS_PER_ZONE 6
#define MAX_SHIMMER 92

class FairyEffect : public BaseEffect<FairyEffect, BufferedEffect<EffectDimensionality::d1>> {
public:
    //4 bytes
    struct Flasher {
        uint16_t stateStart;
        uint8_t stateDur;
        bool stateOn;
    };

private:
    using Self = FairyEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Fairy@!,# of flashers;!,!;!";
    static constexpr const uint8_t effectId = FX_MODE_FAIRY;

    explicit FairyEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        //set every pixel to a 'random' color from palette (using seed so it doesn't change between frames)
        uint16_t PRNG16 = 5100 + strip.getCurrSegmentId();
        for (unsigned i = 0; i < coordinate.width; i++) {
            PRNG16 = (uint16_t)(PRNG16 * 2053) + 1384; //next 'random' number
            buffer.setPixelColor(i, SEGMENT.color_from_palette(PRNG16 >> 8, false, false, 0));
        }

        //amount of flasher pixels depending on intensity (0: none, 255: every LED)
        if (SEGMENT.intensity == 0) return true;
        unsigned flasherDistance = ((255 - SEGMENT.intensity) / 28) +1; //1-10
        unsigned numFlashers = (coordinate.width / flasherDistance) +1;

        if (!resizeVector(flashers, numFlashers)) {
            return false;
        }

        unsigned now16 = strip.now & 0xFFFF;

        //Up to 11 flashers in one brightness zone, afterwards a new zone for every 6 flashers
        unsigned zones = numFlashers/FLASHERS_PER_ZONE;
        if (!zones) zones = 1;
        unsigned flashersInZone = numFlashers/zones;
        uint8_t flasherBri[FLASHERS_PER_ZONE*2 -1];

        for (unsigned z = 0; z < zones; z++) {
            unsigned flasherBriSum = 0;
            unsigned firstFlasher = z*flashersInZone;
            if (z == zones-1) flashersInZone = numFlashers-(flashersInZone*(zones-1));

            for (unsigned f = firstFlasher; f < firstFlasher + flashersInZone; f++) {
                unsigned stateTime = uint16_t(now16 - flashers[f].stateStart);
                //random on/off time reached, switch state
                if (stateTime > flashers[f].stateDur * 10) {
                    flashers[f].stateOn = !flashers[f].stateOn;
                    if (flashers[f].stateOn) {
                        flashers[f].stateDur = 12 + hw_random8(12 + ((255 - SEGMENT.speed) >> 2)); //*10, 250ms to 1250ms
                    } else {
                        flashers[f].stateDur = 20 + hw_random8(6 + ((255 - SEGMENT.speed) >> 2)); //*10, 250ms to 1250ms
                    }
                    //flashers[f].stateDur = 51 + hw_random8(2 + ((255 - SEGMENT.speed) >> 1));
                    flashers[f].stateStart = now16;
                    if (stateTime < 255) {
                        flashers[f].stateStart -= 255 -stateTime; //start early to get correct bri
                        flashers[f].stateDur += 26 - stateTime/10;
                        stateTime = 255 - stateTime;
                    } else {
                        stateTime = 0;
                    }
                }
                if (stateTime > 255) stateTime = 255; //for flasher brightness calculation, fades in first 255 ms of state
                //flasherBri[f - firstFlasher] = (flashers[f].stateOn) ? 255-SEGMENT.gamma8((510 - stateTime) >> 1) : SEGMENT.gamma8((510 - stateTime) >> 1);
                flasherBri[f - firstFlasher] = (flashers[f].stateOn) ? stateTime : 255 - (stateTime >> 0);
                flasherBriSum += flasherBri[f - firstFlasher];
            }
            //dim factor, to create "shimmer" as other pixels get less voltage if a lot of flashers are on
            unsigned avgFlasherBri = flasherBriSum / flashersInZone;
            unsigned globalPeakBri = 255 - ((avgFlasherBri * MAX_SHIMMER) >> 8); //183-255, suitable for 1/5th of LEDs flashers

            for (unsigned f = firstFlasher; f < firstFlasher + flashersInZone; f++) {
                uint8_t bri = (flasherBri[f - firstFlasher] * globalPeakBri) / 255;
                PRNG16 = (uint16_t)(PRNG16 * 2053) + 1384; //next 'random' number
                unsigned flasherPos = f*flasherDistance;
                buffer.setPixelColor(flasherPos, color_blend(SEGCOLOR(1), SEGMENT.color_from_palette(PRNG16 >> 8, false, false, 0), bri));
                for (unsigned i = flasherPos+1; i < flasherPos+flasherDistance && i < coordinate.width; i++) {
                    PRNG16 = (uint16_t)(PRNG16 * 2053) + 1384; //next 'random' number
                    buffer.setPixelColor(i, SEGMENT.color_from_palette(PRNG16 >> 8, false, false, 0, globalPeakBri));
                }
            }
        }
        return true;
    }

private:
    SegmentAllocator<Flasher>::vector flashers;
};


