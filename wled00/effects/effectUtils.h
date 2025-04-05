#pragma once

#include <cstdint>
#include "../wled_math.h"

#define FRAMETIME        strip.getFrameTime()
/* How much data bytes each segment should max allocate to leave enough space for other segments,
  assuming each segment uses the same amount of data. 256 for ESP8266, 640 for ESP32. */
#define FAIR_DATA_PER_SEG (MAX_SEGMENT_DATA / strip.getMaxSegments())
#define SEGMENT          strip._segments[strip.getCurrSegmentId()]
#define SEGENV           strip._segments[strip.getCurrSegmentId()]

// paletteBlend: 0 - wrap when moving, 1 - always wrap, 2 - never wrap, 3 - none (undefined)
#define PALETTE_SOLID_WRAP   (strip.paletteBlend == 1 || strip.paletteBlend == 3)
#define PALETTE_MOVING_WRAP !(strip.paletteBlend == 2 || (strip.paletteBlend == 0 && SEGMENT.speed == 0))

// a few constants needed for AudioReactive effects
// for 22Khz sampling
constexpr float MAX_FREQUENCY  = 11025.0f; // sample frequency / 2 (as per Nyquist criterion)
constexpr float MAX_FREQ_LOG10 = 4.04238f; // log10(MAX_FREQUENCY)
// for 20Khz sampling
//constexpr float MAX_FREQUENCY  = 10240.0f;
//constexpr float MAX_FREQ_LOG10 = 4.0103f;
// for 10Khz sampling
//constexpr float MAX_FREQUENCY  = 5120.0f;
//constexpr float MAX_FREQ_LOG10 = 3.71f;

inline uint8_t sin_gap(uint16_t in) {
    if (in & 0x100)
      return 0;
    return sin8_t(in + 192); // correct phase shift of sine so that it starts and stops at 0
}

inline uint16_t triwave16(uint16_t in) {
    if (in < 0x8000)
        return in *2;
    return 0xFFFF - (in - 0x8000)*2;
}

/*
 * Generates a tristate square wave w/ attac & decay
 * @param x input value 0-255
 * @param pulsewidth 0-127
 * @param attdec attack & decay, max. pulsewidth / 2
 * @returns signed waveform value
 */
inline int8_t tristate_square8(uint8_t x, uint8_t pulsewidth, uint8_t attdec) {
    int8_t a = 127;
    if (x > 127) {
        a = -127;
        x -= 127;
    }

    if (x < attdec) { //inc to max
        return (int16_t) x * a / attdec;
    }
    else if (x < pulsewidth - attdec) { //max
        return a;
    }
    else if (x < pulsewidth) { //dec to 0
        return (int16_t) (pulsewidth - x) * a / attdec;
    }
    return 0;
}

