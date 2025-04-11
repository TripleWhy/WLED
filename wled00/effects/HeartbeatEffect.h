#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Modulates the brightness similar to a heartbeat
 * (unimplemented?) tries to draw an ECG approximation on a 2D matrix
 */
class HeartbeatEffect : public BaseEffect<HeartbeatEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = HeartbeatEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Heartbeat@!,!;!,!;!;01;m12=1";
    static constexpr const uint8_t effectId = FX_MODE_HEARTBEAT;

    explicit HeartbeatEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        unsigned bpm = 40 + (SEGMENT.speed >> 3);
        uint32_t msPerBeat = (60000L / bpm);
        uint32_t secondBeat = (msPerBeat / 3);
        uint32_t bri_lower = aux1;
        unsigned long beatTimer = strip.now - step;

        bri_lower = bri_lower * 2042 / (2048 + SEGMENT.intensity);
        aux1 = bri_lower;

        if ((beatTimer > secondBeat) && !aux0) { // time for the second beat?
            aux1 = UINT16_MAX; //3/4 bri
            aux0 = 1;
        }
        if (beatTimer > msPerBeat) { // time to reset the beat timer?
            aux1 = UINT16_MAX; //full bri
            aux0 = 0;
            step = strip.now;
        }

        for (unsigned i = 0; i < coordinate.width; i++) {
            buffer.setPixelColor(i, color_blend(SEGMENT.color_from_palette(i, true, PALETTE_SOLID_WRAP, 0), SEGCOLOR(1), uint8_t(255 - (aux1 >> 8))));
        }
        return true;
    }

private:
    uint32_t step{};
    uint16_t aux0{};
    uint16_t aux1{};
};


