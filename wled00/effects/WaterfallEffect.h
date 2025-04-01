#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"
#include "effectUtils.h"

///////////////////////
//   ** Waterfall    //
///////////////////////
// Combines peak detection with FFT_MajorPeak and FFT_Magnitude.
// Waterfall. By: Andrew Tuline
// effect can work on single pixels, we just lose the shifting effect
class WaterfallEffect : public BaseEffect<WaterfallEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = WaterfallEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char* const metaData = "Waterfall@!,Adjust color,Select bin,Volume (min);!,!;!;01f;c2=0,m12=2,si=0";
    static constexpr const uint8_t effectId = FX_MODE_WATERFALL;

    explicit WaterfallEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        um_data_t *um_data    = getAudioData();
        uint8_t samplePeak    = *(uint8_t*)um_data->u_data[3];
        float   FFT_MajorPeak = *(float*)  um_data->u_data[4];
        uint8_t *maxVol       =  (uint8_t*)um_data->u_data[6];
        uint8_t *binNum       =  (uint8_t*)um_data->u_data[7];
        float   my_magnitude  = *(float*)   um_data->u_data[5] / 8.0f;

        if (FFT_MajorPeak < 1) FFT_MajorPeak = 1;                                         // log10(0) is "forbidden" (throws exception)

        if (SEGENV.call == 0) {
            aux0 = 255;
            SEGMENT.custom1 = *binNum;
            SEGMENT.custom2 = *maxVol * 2;
        }

        *binNum = SEGMENT.custom1;                              // Select a bin.
        *maxVol = SEGMENT.custom2 / 2;                          // Our volume comparator.

        uint8_t secondHand = micros() / (256-SEGMENT.speed)/500 + 1 % 16;
        if (aux0 != secondHand) {                        // Triggered millis timing.
            aux0 = secondHand;

            //uint8_t pixCol = (log10f((float)FFT_MajorPeak) - 2.26f) * 177;  // 10Khz sampling - log10 frequency range is from 2.26 (182hz) to 3.7 (5012hz). Let's scale accordingly.
            uint8_t pixCol = (log10f(FFT_MajorPeak) - 2.26f) * 150;           // 22Khz sampling - log10 frequency range is from 2.26 (182hz) to 3.967 (9260hz). Let's scale accordingly.
            if (FFT_MajorPeak < 182.0f) pixCol = 0;                           // handle underflow

            unsigned k = coordinate.width-1;
            if (samplePeak) {
                buffer.setPixelColor(k, (uint32_t)CRGB(CHSV(92,92,92)));
            } else {
                buffer.setPixelColor(k, color_blend(SEGCOLOR(1), SEGMENT.color_from_palette(pixCol+SEGMENT.intensity, false, PALETTE_SOLID_WRAP, 0), (uint8_t)my_magnitude));
            }
            // loop will not execute if coordinate.width equals 1
            for (unsigned i = 0; i < k; i++) {
                buffer.setPixelColor(i, buffer.getPixelColor(i+1)); // shift left
            }
        }
    }

private:
    uint16_t aux0{};
};


