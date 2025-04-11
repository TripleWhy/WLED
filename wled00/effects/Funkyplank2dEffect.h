#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/////////////////////////
//  ** 2D Funky plank  //
/////////////////////////
// Written by ??? Adapted by Will Tatam.
class FunkyPlank2dEffect : public BaseEffect<FunkyPlank2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = FunkyPlank2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "Funky Plank@Scroll speed,,# of bands;;;2f;si=0";
    static constexpr const uint8_t effectId = FX_MODE_2DFUNKYPLANK;

    explicit FunkyPlank2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        int NUMB_BANDS = map(SEGMENT.custom1, 0, 255, 1, 16);
        int barWidth = (cols / NUMB_BANDS);
        int bandInc = 1;
        if (barWidth == 0) {
            // Matrix narrower than fft bands
            barWidth = 1;
            bandInc = (NUMB_BANDS / cols);
        }

        um_data_t *um_data = getAudioData();
        uint8_t *fftResult = (uint8_t*)um_data->u_data[2];

        if (SEGENV.call == 0) {
            buffer.fill(BLACK);
        }

        uint8_t secondHand = micros()/(256-SEGMENT.speed)/500+1 % 64;
        if (aux0 != secondHand) {                        // Triggered millis timing.
            aux0 = secondHand;

            // display values of
            int b = 0;
            for (int band = 0; band < NUMB_BANDS; band += bandInc, b++) {
                int hue = fftResult[band % 16];
                int v = map(fftResult[band % 16], 0, 255, 10, 255);
                for (int w = 0; w < barWidth; w++) {
                    int xpos = (barWidth * b) + w;
                    buffer.setPixelColor(xpos, 0, CHSV(hue, 255, v));
                }
            }

            // Update the display:
            for (int i = (rows - 1); i > 0; i--) {
                for (int j = (cols - 1); j >= 0; j--) {
                    buffer.setPixelColor(j, i, buffer.getPixelColor(j, i-1));
                }
            }
        }
        return true;
    }

private:
    uint16_t aux0{};
};


#endif //WLED_DISABLE_2D
