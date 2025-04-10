#pragma once

#include <array>
#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"
#include "RippleEffect.h"

///////////////////////////////////////////////////////////////////////////////
/********************     audio enhanced routines     ************************/
///////////////////////////////////////////////////////////////////////////////


/////////////////////////////////
//     * Ripple Peak           //
/////////////////////////////////
// * Ripple peak. By Andrew Tuline.
// This currently has no controls.
class RipplepeakEffect : public BaseEffect<RipplepeakEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = RipplepeakEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;
    using Ripple = RippleEffect::Ripple;
    static constexpr unsigned maxRipples = 16;

public:
    static constexpr const char metaData[] PROGMEM = "Ripple Peak@Fade rate,Max # of ripples,Select bin,Volume (min);!,!;!;1v;c2=0,m12=0,si=0";
    static constexpr const uint8_t effectId = FX_MODE_RIPPLEPEAK;

    explicit RipplepeakEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        um_data_t *um_data = getAudioData();
        uint8_t samplePeak    = *(uint8_t*)um_data->u_data[3];
        #ifdef ESP32
        float   FFT_MajorPeak = *(float*)  um_data->u_data[4];
        #endif
        uint8_t *maxVol       =  (uint8_t*)um_data->u_data[6];
        uint8_t *binNum       =  (uint8_t*)um_data->u_data[7];

        // printUmData();

        if (SEGENV.call == 0) {
            SEGMENT.custom1 = *binNum;
            SEGMENT.custom2 = *maxVol * 2;
        }

        *binNum = SEGMENT.custom1;                              // Select a bin.
        *maxVol = SEGMENT.custom2 / 2;                          // Our volume comparator.

        buffer.fadeOut(240);                                  // Lower frame rate means less effective fading than FastLED
        buffer.fadeOut(240);

        for (int i = 0; i < SEGMENT.intensity/16; i++) {   // Limit the number of ripples.
            if (samplePeak) ripples[i].state = 255;

            switch (ripples[i].state) {
                case 254:     // Inactive mode
                    break;

                case 255:                                           // Initialize ripple variables.
                    ripples[i].pos = hw_random16(coordinate.width);
                    #ifdef ESP32
                        if (FFT_MajorPeak > 1)                          // log10(0) is "forbidden" (throws exception)
                        ripples[i].color = (int)(log10f(FFT_MajorPeak)*128);
                        else ripples[i].color = 0;
                    #else
                        ripples[i].color = hw_random8();
                    #endif
                    ripples[i].state = 0;
                    break;

                case 0:
                    buffer.setPixelColor(ripples[i].pos, SEGMENT.color_from_palette(ripples[i].color, false, PALETTE_SOLID_WRAP, 0));
                    ripples[i].state++;
                    break;

                case maxRipples:                                    // At the end of the ripples. 254 is an inactive mode.
                    ripples[i].state = 254;
                    break;

                default:                                            // Middle of the ripples.
                    buffer.setPixelColor((ripples[i].pos + ripples[i].state + coordinate.width) % coordinate.width, color_blend(SEGCOLOR(1), SEGMENT.color_from_palette(ripples[i].color, false, PALETTE_SOLID_WRAP, 0), uint8_t(2*255/ripples[i].state)));
                    buffer.setPixelColor((ripples[i].pos - ripples[i].state + coordinate.width) % coordinate.width, color_blend(SEGCOLOR(1), SEGMENT.color_from_palette(ripples[i].color, false, PALETTE_SOLID_WRAP, 0), uint8_t(2*255/ripples[i].state)));
                    ripples[i].state++;                               // Next step.
                    break;
            } // switch step
        } // for i
    }

private:
    std::array<Ripple, maxRipples> ripples{};
};


