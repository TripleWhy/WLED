#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/////////////////////////
//   ** DJLight        //
/////////////////////////
class DjLightEffect : public BaseEffect<DjLightEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = DjLightEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "DJ Light@Speed;;;01f;m12=2,si=0";
    static constexpr const uint8_t effectId = FX_MODE_DJLIGHT;

    explicit DjLightEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }
                                         // Written by ??? Adapted by Will Tatam.
        // No need to prevent from executing on single led strips, only mid will be set (mid = 0)
        const int mid = coordinate.width / 2;

        um_data_t *um_data = getAudioData();
        uint8_t *fftResult = (uint8_t*)um_data->u_data[2];

        if (parameters.call == 0) {
            buffer.fill(BLACK);
        }

        uint8_t secondHand = micros()/(256-parameters.speed)/500+1 % 64;
        if (aux0 != secondHand) {                        // Triggered millis timing.
            aux0 = secondHand;

            CRGB color = CRGB(fftResult[15]/2, fftResult[5]/2, fftResult[0]/2); // 16-> 15 as 16 is out of bounds
            color.fadeToBlackBy(map(fftResult[4], 0, 255, 255, 4));
            buffer.setPixelColor(mid, RGBW32(color.r, color.g, color.b, 0));     // TODO - Update

            // if coordinate.width equals 1 these loops won't execute
            for (int i = coordinate.width - 1; i > mid; i--) buffer.setPixelColor(i, buffer.getPixelColor(i-1)); // move to the left
            for (int i = 0; i < mid; i++)                    buffer.setPixelColor(i, buffer.getPixelColor(i+1)); // move to the right
        }
        return true;
    }

private:
    uint16_t aux0{};
};


