#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * K.I.T.T.
 */
class LarsonScannerEffect : public BaseEffect<LarsonScannerEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = LarsonScannerEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Scanner@!,Trail,Delay,,,Shift palette,Bi-delay,Dual;!,!,!;!;;m12=0,c1=0,o1=0,o3=0";
    static constexpr const uint8_t effectId = FX_MODE_LARSON_SCANNER;

    explicit LarsonScannerEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }


        const unsigned speed  = FRAMETIME * map(SEGMENT.speed, 0, 255, 96, 2); // map into useful range
        const unsigned pixels = coordinate.width / speed; // how many pixels to advance per frame

        buffer.fadeOut(255-SEGMENT.intensity);

        if (step > strip.now) return true;  // we have a pause

        unsigned index = aux1 + pixels;
        // are we slow enough to use frames per pixel?
        if (pixels == 0) {
            const unsigned frames = speed / coordinate.width; // how many frames per 1 pixel
            if (step++ < frames) return true;
            step = 0;
            index++;
        }

        if (index > coordinate.width) {

            aux0 = !aux0; // change direction
            aux1 = 0;            // reset position
            // set delay
            if (aux0 || SEGMENT.check2) step = strip.now + SEGMENT.custom1 * 25; // multiply by 25ms
            else step = 0;
        } else {

            uint32_t cycleTime = 10 + (255 - SEGMENT.speed)*2;
            uint32_t it = strip.now / cycleTime;
            const bool moving = SEGMENT.check1;
            // paint as many pixels as needed
            for (unsigned i = aux1; i < index; i++) {
                unsigned j = (aux0) ? i : coordinate.width - 1 - i;
                unsigned palIdx = moving ? (j+it)%coordinate.width : j;
                uint32_t c = SEGMENT.color_from_palette(palIdx, true, moving, 0);
                buffer.setPixelColor(j, c);
                if (SEGMENT.check3) {
                    buffer.setPixelColor(coordinate.width - 1 - j, SEGCOLOR(2) ? SEGCOLOR(2) : c);
                }
            }
            aux1 = index;
        }
        return true;
    }

private:
    uint32_t step{};
    uint16_t aux0{};
    uint16_t aux1{};
};


