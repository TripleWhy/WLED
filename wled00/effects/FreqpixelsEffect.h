#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//////////////////////
//   ** Freqpixels  //
//////////////////////
// Start frequency = 60 Hz and log10(60) = 1.78
// End frequency = 5120 Hz and lo10(5120) = 3.71
//  SEGMENT.speed select faderate
//  SEGMENT.intensity select colour index
class FreqpixelsEffect : public BaseEffect<FreqpixelsEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = FreqpixelsEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Freqpixels@Fade rate,Starting color and # of pixels;!,!,;!;1f;m12=0,si=0";
    static constexpr const uint8_t effectId = FX_MODE_FREQPIXELS;

    explicit FreqpixelsEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }
                                    // Freqpixel. By Andrew Tuline.
        um_data_t *um_data = getAudioData();
        float FFT_MajorPeak = *(float*)um_data->u_data[4];
        float my_magnitude  = *(float*)um_data->u_data[5] / 16.0f;
        if (FFT_MajorPeak < 1) FFT_MajorPeak = 1.0f; // log10(0) is "forbidden" (throws exception)

        // this code translates to speed * (2 - speed/255) which is a) speed*2 or b) speed (when speed is 255)
        // and since fade_out() can only take 0-255 it will behave incorrectly when speed > 127
        //uint16_t fadeRate = 2*SEGMENT.speed - SEGMENT.speed*SEGMENT.speed/255;    // Get to 255 as quick as you can.
        unsigned fadeRate = SEGMENT.speed*SEGMENT.speed; // Get to 255 as quick as you can.
        fadeRate = map(fadeRate, 0, 65535, 1, 255);

        int fadeoutDelay = (256 - SEGMENT.speed) / 64;
        if ((fadeoutDelay <= 1 ) || ((SEGENV.call % fadeoutDelay) == 0)) buffer.fadeOut(fadeRate);

        uint8_t pixCol = (log10f(FFT_MajorPeak) - 1.78f) * 255.0f/(MAX_FREQ_LOG10 - 1.78f);  // Scale log10 of frequency values to the 255 colour index.
        if (FFT_MajorPeak < 61.0f) pixCol = 0;                                               // handle underflow
        for (int i=0; i < SEGMENT.intensity/32+1; i++) {
            unsigned locn = hw_random16(0,coordinate.width);
            buffer.setPixelColor(locn, color_blend(SEGCOLOR(1), SEGMENT.color_from_palette(SEGMENT.intensity+pixCol, false, PALETTE_SOLID_WRAP, 0), (uint8_t)my_magnitude));
        }
        return true;
    }

private:
};


