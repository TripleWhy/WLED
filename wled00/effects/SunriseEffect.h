#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Mode simulates a gradual sunrise
 */
class SunriseEffect : public BaseEffect<SunriseEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = SunriseEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Sunrise@Time [min],Width;;!;;pal=35,sx=60";
    static constexpr const uint8_t effectId = FX_MODE_SUNRISE;

    explicit SunriseEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        //speed 0 - static sun
        //speed 1 - 60: sunrise time in minutes
        //speed 60 - 120 : sunset time in minutes - 60;
        //speed above: "breathing" rise and set
        if (SEGENV.call == 0 || SEGMENT.speed != previousSpeed) {
            step = millis(); //save starting time, millis() because strip.now can change from sync
            previousSpeed = SEGMENT.speed;
        }

        buffer.fill(BLACK);
        unsigned stage = 0xFFFF;

        uint32_t s10SinceStart = (millis() - step) /100; //tenths of seconds

        if (SEGMENT.speed > 120) { //quick sunrise and sunset
            unsigned counter = (strip.now >> 1) * (((SEGMENT.speed -120) >> 1) +1);
            stage = triwave16(counter);
        } else if (SEGMENT.speed) { //sunrise
            unsigned durMins = SEGMENT.speed;
            if (durMins > 60) durMins -= 60;
            uint32_t s10Target = durMins * 600;
            if (s10SinceStart > s10Target) s10SinceStart = s10Target;
            stage = map(s10SinceStart, 0, s10Target, 0, 0xFFFF);
            if (SEGMENT.speed > 60) stage = 0xFFFF - stage; //sunset
        }

        for (unsigned i = 0; i <= coordinate.width/2; i++)
        {
            //default palette is Fire    
            unsigned wave = triwave16((i * stage) / coordinate.width);
            wave = (wave >> 8) + ((wave * SEGMENT.intensity) >> 15);
            uint32_t c;
            if (wave > 240) { //clipped, full white sun
                c = SEGMENT.color_from_palette( 240, false, true, 255);
            } else { //transition
                c = SEGMENT.color_from_palette(wave, false, true, 255);
            }
            buffer.setPixelColor(i, c);
            buffer.setPixelColor(coordinate.width - i - 1, c);
        }
        return true;
    }

private:
    uint32_t step{};
    uint16_t previousSpeed{};
};


