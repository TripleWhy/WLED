#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

// meteor effect & meteor smooth (merged by @dedehai)
// send a meteor from begining to to the end of the strip with a trail that randomly decays.
// adapted from https://www.tweaking4all.com/hardware/arduino/adruino-led-strip-effects/#LEDStripEffectMeteorRain
class MeteorEffect : public BaseEffect<MeteorEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = MeteorEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Meteor@!,Trail,,,,Gradient,,Smooth;;!;1";
    static constexpr const uint8_t effectId = FX_MODE_METEOR;

    explicit MeteorEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        if (!trail.resize(coordinate.width)) {
            return false;
        }

        const bool meteorSmooth = parameters.check3;
        const unsigned meteorSize = 1 + coordinate.width / 20; // 5%
        uint16_t meteorstart;
        if(meteorSmooth) meteorstart = map((step >> 6 & 0xFF), 0, 255, 0, coordinate.width -1);
        else {
            unsigned counter = strip.now * ((parameters.speed >> 2) + 8);
            meteorstart = (counter * coordinate.width) >> 16;
        }

        const int max = SEGMENT.palette==5 || !parameters.check1 ? 240 : 255;
        // fade all leds to colors[1] in LEDs one step
        for (unsigned i = 0; i < coordinate.width; i++) {
            uint32_t col;
            if (hw_random8() <= 255 - parameters.intensity) {
                if(meteorSmooth) {
                    if (trail[i] > 0) {
                        int change = trail[i] + 4 - hw_random8(24); //change each time between -20 and +4
                        trail[i] = constrain(change, 0, max);
                    }
                    col = parameters.check1 ? parameters.color_from_palette(i, true, false, 0, trail[i]) : parameters.color_from_palette(trail[i], false, true, 255);
                }
                else {
                    trail[i] = scale8(trail[i], 128 + hw_random8(127));
                    int index = trail[i];
                    int idx = 255;
                    int bri = SEGMENT.palette==35 || SEGMENT.palette==36 ? 255 : trail[i];
                    if (!parameters.check1) {
                        idx = 0;
                        index = map(i,0,coordinate.width,0,max);
                        bri = trail[i];
                    }
                    col = parameters.color_from_palette(index, false, false, idx, bri);  // full brightness for Fire
                }
                buffer.setPixelColor(i, col);
            }
        }

        // draw meteor
        for (unsigned j = 0; j < meteorSize; j++) {
            unsigned index = (meteorstart + j) % coordinate.width;
            if(meteorSmooth) {
                    trail[index] = max;
                    uint32_t col = parameters.check1 ? parameters.color_from_palette(index, true, false, 0, trail[index]) : parameters.color_from_palette(trail[index], false, true, 255);
                    buffer.setPixelColor(index, col);
            }
            else{
                int idx = 255;
                int i = trail[index] = max;
                if (!parameters.check1) {
                    i = map(index,0,coordinate.width,0,max);
                    idx = 0;
                }
                uint32_t col = parameters.color_from_palette(i, false, false, idx, 255); // full brightness
                buffer.setPixelColor(index, col);
            }
        }

        step += parameters.speed +1;
        return true;
    }

private:
    SegmentAllocator<byte>::vector trail{};
    uint32_t step{};
};


