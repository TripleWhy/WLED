#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//////////////////////
//   ** Freqwave    //
//////////////////////
// Assign a color to the central (starting pixels) based on the predominant frequencies and the volume. The color is being determined by mapping the MajorPeak from the FFT
// and then mapping this to the HSV color circle. Currently we are sampling at 10240 Hz, so the highest frequency we can look at is 5120Hz.
//
// parameters.custom1: the lower cut off point for the FFT. (many, most time the lowest values have very little information since they are FFT conversion artifacts. Suggested value is close to but above 0
// parameters.custom2: The high cut off point. This depends on your sound profile. Most music looks good when this slider is between 50% and 100%.
// parameters.custom3: "preamp" for the audio signal for audio10.
//
// I suggest that for this effect you turn the brightness to 95%-100% but again it depends on your soundprofile you find yourself in.
// Instead of using colorpalettes, This effect works on the HSV color circle with red being the lowest frequency
//
// As a compromise between speed and accuracy we are currently sampling with 10240Hz, from which we can then determine with a 512bin FFT our max frequency is 5120Hz.
// Depending on the music stream you have you might find it useful to change the frequency mapping.
class FreqwaveEffect : public BaseEffect<FreqwaveEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = FreqwaveEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Freqwave@Speed,Sound effect,Low bin,High bin,Pre-amp;;;01f;m12=2,si=0";
    static constexpr const uint8_t effectId = FX_MODE_FREQWAVE;

    explicit FreqwaveEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }
                                        // Freqwave. By Andreas Pleschung.
        // As before, this effect can also work on single pixels, we just lose the shifting effect
        um_data_t *um_data = getAudioData();
        float FFT_MajorPeak = *(float*)um_data->u_data[4];
        float volumeSmth    = *(float*)um_data->u_data[0];

        if (parameters.call == 0) {
            buffer.fill(BLACK);
        }

        uint8_t secondHand = micros()/(256-parameters.speed)/500 % 16;
        if(aux0 != secondHand) {
            aux0 = secondHand;

            float sensitivity = mapf(parameters.custom3, 1, 31, 1, 10); // reduced resolution slider
            float pixVal = min(255.0f, volumeSmth * (float)parameters.intensity / 256.0f * sensitivity);
            float intensity = mapf(pixVal, 0.0f, 255.0f, 0.0f, 100.0f) / 100.0f;  // make a brightness from the last avg

            CRGB color = 0;

            if (FFT_MajorPeak > MAX_FREQUENCY) FFT_MajorPeak = 1.0f;
            // MajorPeak holds the freq. value which is most abundant in the last sample.
            // With our sampling rate of 10240Hz we have a usable freq range from roughly 80Hz to 10240/2 Hz
            // we will treat everything with less than 65Hz as 0

            if (FFT_MajorPeak < 80) {
                color = CRGB::Black;
            } else {
                int upperLimit = 80 + 42 * parameters.custom2;
                int lowerLimit = 80 + 3 * parameters.custom1;
                uint8_t i =  lowerLimit!=upperLimit ? map(FFT_MajorPeak, lowerLimit, upperLimit, 0, 255) : FFT_MajorPeak; // may under/overflow - so we enforce uint8_t
                unsigned b = min(255.0f, 255.0f * intensity);
                color = CHSV(i, 240, (uint8_t)b); // implicit conversion to RGB supplied by FastLED
            }

            buffer.setPixelColor(coordinate.width/2, RGBW32(color.r, color.g, color.b, 0));

            // shift the pixels one pixel outwards
            // if coordinate.width equals 1 these loops won't execute
            for (unsigned i = coordinate.width - 1; i > coordinate.width/2; i--) buffer.setPixelColor(i, buffer.getPixelColor(i-1)); //move to the left
            for (unsigned i = 0; i < coordinate.width/2; i++)          buffer.setPixelColor(i, buffer.getPixelColor(i+1)); // move to the right
        }
        return true;
    }

private:
    uint16_t aux0{};
};


