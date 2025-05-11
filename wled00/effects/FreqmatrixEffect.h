#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

///////////////////////
//   ** Freqmatrix   //
///////////////////////
class FreqmatrixEffect : public BaseEffect<FreqmatrixEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = FreqmatrixEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Freqmatrix@Speed,Sound effect,Low bin,High bin,Sensitivity;;;01f;m12=3,si=0";
    static constexpr const uint8_t effectId = FX_MODE_FREQMATRIX;

    explicit FreqmatrixEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }
                                    // Freqmatrix. By Andreas Pleschung.
        // No need to prevent from executing on single led strips, we simply change pixel 0 each time and avoid the shift
        um_data_t *um_data = getAudioData();
        float FFT_MajorPeak = *(float*)um_data->u_data[4];
        float volumeSmth    = *(float*)um_data->u_data[0];

        if (parameters.call == 0) {
            buffer.fill(BLACK);
        }

        uint8_t secondHand = micros()/(256-parameters.speed)/500 % 16;
        if(aux0 != secondHand) {
            aux0 = secondHand;

            uint8_t sensitivity = map(parameters.custom3, 0, 31, 1, 10); // reduced resolution slider
            int pixVal = (volumeSmth * parameters.intensity * sensitivity) / 256.0f;
            if (pixVal > 255) pixVal = 255;

            float intensity = map(pixVal, 0, 255, 0, 100) / 100.0f;  // make a brightness from the last avg

            CRGB color = CRGB::Black;

            if (FFT_MajorPeak > MAX_FREQUENCY) FFT_MajorPeak = 1;
            // MajorPeak holds the freq. value which is most abundant in the last sample.
            // With our sampling rate of 10240Hz we have a usable freq range from roughly 80Hz to 10240/2 Hz
            // we will treat everything with less than 65Hz as 0

            if (FFT_MajorPeak < 80) {
                color = CRGB::Black;
            } else {
                int upperLimit = 80 + 42 * parameters.custom2;
                int lowerLimit = 80 + 3 * parameters.custom1;
                uint8_t i =  lowerLimit!=upperLimit ? map(FFT_MajorPeak, lowerLimit, upperLimit, 0, 255) : FFT_MajorPeak;  // may under/overflow - so we enforce uint8_t
                unsigned b = 255 * intensity;
                if (b > 255) b = 255;
                color = CHSV(i, 240, (uint8_t)b); // implicit conversion to RGB supplied by FastLED
            }

            // shift the pixels one pixel up
            buffer.setPixelColor(0, RGBW32(color.r, color.g, color.b, 0));
            // if coordinate.width equals 1 this loop won't execute
            for (int i = coordinate.width - 1; i > 0; i--) buffer.setPixelColor(i, buffer.getPixelColor(i-1)); //move to the left
        }
        return true;
    }

private:
    uint16_t aux0{};
};


