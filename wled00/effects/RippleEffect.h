#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//Water ripple
//propagation velocity from speed
//drop rate from intensity
class RippleEffect : public BaseEffect<RippleEffect, BufferedEffect<EffectDimensionality::d2>> {
public:
    struct Ripple {
        uint8_t state;
        uint8_t color;
        uint16_t pos;
    };

private:
    using Self = RippleEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;
#ifdef ESP8266
    static constexpr int MAX_RIPPLES = 56;
#else
    static constexpr int MAX_RIPPLES = 100;
#endif

public:
    static constexpr const char metaData[] PROGMEM = "Ripple@!,Waves,Blur,,,Rainbow,Overlay;,!;!;12;c1=0";
    static constexpr const uint8_t effectId = FX_MODE_RIPPLE;

    explicit RippleEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        unsigned maxRipples = min(1 + (int)(coordinate.width >> 2), MAX_RIPPLES);  // 56 max for 16 segment ESP8266
        ripples.resize(maxRipples);
        if (ripples.size() != maxRipples) {
            ripples.clear();
            return;
        }
        ripples.shrink_to_fit();

        if (SEGENV.call == 0) {
            aux0 = aux1 = hw_random8();
        }
        if (aux0 == aux1) {
            aux1 = hw_random8();
        } else if (aux1 > aux0) {
            aux0++;
        } else {
            aux0--;
        }
        if(SEGMENT.custom1 || SEGMENT.check2) // blur or overlay
                                                 buffer.fadeOut(250);
        else buffer.fill(SEGMENT.check1 ? color_blend(SEGMENT.color_wheel(aux0),BLACK,uint8_t(235)) : SEGCOLOR(1));

        //draw wave
        for (unsigned i = 0; i < maxRipples; i++) {
            unsigned ripplestate = ripples[i].state;
            if (ripplestate) {
                unsigned rippledecay = (SEGMENT.speed >> 4) +1; //faster decay if faster propagation
                unsigned rippleorigin = ripples[i].pos;
                uint32_t col = SEGMENT.color_from_palette(ripples[i].color, false, false, 255);
                unsigned propagation = ((ripplestate/rippledecay - 1) * (SEGMENT.speed + 1));
                int propI = propagation >> 8;
                unsigned propF = propagation & 0xFF;
                unsigned amp = (ripplestate < 17) ? triwave8((ripplestate-1)*8) : map(ripplestate,17,255,255,2);

                if (SEGMENT.is2D()) {
                    propI /= 2;
                    unsigned cx = rippleorigin >> 8;
                    unsigned cy = rippleorigin & 0xFF;
                    unsigned mag = scale8(sin8_t((propF>>2)), amp);
                    if (propI > 0) buffer.drawCircle(cx, cy, propI, color_blend(buffer.getPixelColor(cx + propI, cy), col, mag), true);
                }
                ripplestate += rippledecay;
                ripples[i].state = (ripplestate > 254) ? 0 : ripplestate;
            } else {//randomly create new wave
                constexpr uint32_t IBN = 5100;
                if (hw_random16(IBN + 10000u) <= (SEGMENT.intensity >> (SEGMENT.is2D()*3))) {
                    ripples[i].state = 1;
                    ripples[i].pos = SEGMENT.is2D() ? ((hw_random8(coordinate.width)<<8) | (hw_random8(coordinate.height))) : hw_random16(coordinate.width);
                    ripples[i].color = hw_random8(); //color
                }
            }
        }
        buffer.blur(SEGMENT.custom1>>1);
    }

private:
    SegmentAllocator<Ripple>::vector ripples{};
    uint16_t aux0{};
    uint16_t aux1{};
};
#undef MAX_RIPPLES

#endif //WLED_DISABLE_2D
