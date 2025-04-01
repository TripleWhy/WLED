#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"
#include "effectUtils.h"

////////////////////
//   ** Freqmap   //
////////////////////
// Map FFT_MajorPeak to coordinate.width. Would be better if a higher framerate.
class FreqmapEffect : public BaseEffect<FreqmapEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = FreqmapEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char* const metaData = "Freqmap@Fade rate,Starting color;!,!;!;1f;m12=0,si=0";
    static constexpr const uint8_t effectId = FX_MODE_FREQMAP;

    explicit FreqmapEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        // Start frequency = 60 Hz and log10(60) = 1.78
        // End frequency = MAX_FREQUENCY in Hz and lo10(MAX_FREQUENCY) = MAX_FREQ_LOG10

        um_data_t *um_data = getAudioData();
        float FFT_MajorPeak = *(float*)um_data->u_data[4];
        float my_magnitude  = *(float*)um_data->u_data[5] / 4.0f;
        if (FFT_MajorPeak < 1) FFT_MajorPeak = 1;                                         // log10(0) is "forbidden" (throws exception)

        if (SEGENV.call == 0) buffer.fill(BLACK);
        int fadeoutDelay = (256 - SEGMENT.speed) / 32;
        if ((fadeoutDelay <= 1 ) || ((SEGENV.call % fadeoutDelay) == 0)) buffer.fadeOut(SEGMENT.speed);

        int locn = (log10f((float)FFT_MajorPeak) - 1.78f) * (float)coordinate.width/(MAX_FREQ_LOG10 - 1.78f);  // log10 frequency range is from 1.78 to 3.71. Let's scale to coordinate.width.
        if (locn < 1) locn = 0; // avoid underflow

        if (locn >= (int)coordinate.width) locn = coordinate.width-1;
        unsigned pixCol = (log10f(FFT_MajorPeak) - 1.78f) * 255.0f/(MAX_FREQ_LOG10 - 1.78f);   // Scale log10 of frequency values to the 255 colour index.
        if (FFT_MajorPeak < 61.0f) pixCol = 0;                                                 // handle underflow

        uint8_t bright = (uint8_t)my_magnitude;

        buffer.setPixelColor(locn, color_blend(SEGCOLOR(1), SEGMENT.color_from_palette(SEGMENT.intensity+pixCol, false, PALETTE_SOLID_WRAP, 0), bright));
    }

private:
};


