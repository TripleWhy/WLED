#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//based on https://gist.github.com/kriegsman/5408ecd397744ba0393e
class ColortwinkleEffect : public BaseEffect<ColortwinkleEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = ColortwinkleEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Colortwinkles@Fade speed,Spawn speed;;!;;m12=0";
    static constexpr const uint8_t effectId = FX_MODE_COLORTWINKLE;

    explicit ColortwinkleEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        unsigned dataSize = (coordinate.width+7) >> 3; //1 bit per LED
        if (!resizeVector(data, dataSize)) {
            return false;
        }

        CRGBW col, prev;
        uint8_t fadeUpAmount = strip.getBrightness()>28 ? 8 + (SEGMENT.speed>>2) : 68-strip.getBrightness();
        uint8_t fadeDownAmount = strip.getBrightness()>28 ? 8 + (SEGMENT.speed>>3) : 68-strip.getBrightness();
        for (unsigned i = 0; i < coordinate.width; i++) {
            CRGBW cur = buffer.getPixelColor(i);
            prev = cur;
            unsigned index = i >> 3;
            unsigned  bitNum = i & 0x07;
            bool fadeUp = bitRead(data[index], bitNum);

            if (fadeUp) {
                CRGBW incrementalColor = color_fade(cur, fadeUpAmount, true);
                col = color_add(cur, incrementalColor);

                if (col.r == 255 || col.g == 255 || col.b == 255) {
                    bitWrite(data[index], bitNum, false);
                }

                if (cur == prev) {  //fix "stuck" pixels
                    color_add(col, col);
                    buffer.setPixelColor(i, col);
                }
                else buffer.setPixelColor(i, col);
            }
            else {
                col = color_fade(cur, 255 - fadeDownAmount);
                buffer.setPixelColor(i, col);
            }
        }

        for (unsigned j = 0; j <= coordinate.width / 50; j++) {
            if (hw_random8() <= SEGMENT.intensity) {
                for (unsigned times = 0; times < 5; times++) { //attempt to spawn a new pixel 5 times
                    int i = hw_random16(coordinate.width);
                    if (buffer.getPixelColor(i) == 0) {
                        unsigned index = i >> 3;
                        unsigned  bitNum = i & 0x07;
                        bitWrite(data[index], bitNum, true);
                        buffer.setPixelColor(i, ColorFromPalette(SEGPALETTE, hw_random8(), 64, NOBLEND));
                        break; //only spawn 1 new pixel per frame per 50 LEDs
                    }
                }
            }
        }

        return true;
    }

private:
    SegmentAllocator<byte>::vector data{};
};


