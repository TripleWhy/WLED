#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

///////////////////////
//   * GRAVCENTER    //
///////////////////////
// Gravcenter effects By Andrew Tuline.
// Gravcenter base function for Gravcenter (0), Gravcentric (1), Gravimeter (2), Gravfreq (3) (merged by @dedehai)
class GravcenterEffectBase : public BaseEffect<GravcenterEffectBase, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = GravcenterEffectBase;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    explicit GravcenterEffectBase(const EffectInformation& ei, unsigned mode)
        : Base{ei, false},
          mode{mode}
    {
    }

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        um_data_t *um_data = getAudioData();
        float   volumeSmth  = *(float*)  um_data->u_data[0];

        if(mode == 1) buffer.fadeOut(253);  // //Gravcentric
        else if(mode == 2) buffer.fadeOut(249);  // Gravimeter
        else if(mode == 3) buffer.fadeOut(250);  // Gravfreq
        else buffer.fadeOut(251);  // Gravcenter

        float mySampleAvg;
        int tempsamp;
        float segmentSampleAvg = volumeSmth * (float)SEGMENT.intensity / 255.0f;

        if(mode == 2) { //Gravimeter
            segmentSampleAvg *= 0.25; // divide by 4, to compensate for later "sensitivity" upscaling
            mySampleAvg = mapf(segmentSampleAvg*2.0, 0, 64, 0, (coordinate.width-1)); // map to pixels availeable in current segment
            tempsamp = constrain(mySampleAvg,0,coordinate.width-1);       // Keep the sample from overflowing.
        }
        else { // Gravcenter or Gravcentric or Gravfreq
            segmentSampleAvg *= 0.125f; // divide by 8, to compensate for later "sensitivity" upscaling
            mySampleAvg = mapf(segmentSampleAvg*2.0, 0.0f, 32.0f, 0.0f, (float)coordinate.width/2.0f); // map to pixels availeable in current segment
            tempsamp = constrain(mySampleAvg, 0, coordinate.width/2);     // Keep the sample from overflowing.
        }

        uint8_t gravity = 8 - SEGMENT.speed/32;
        int offset = 1;
        if(mode == 2) offset = 0;  // Gravimeter
        if (tempsamp >= topLED) topLED = tempsamp-offset;
        else if (gravityCounter % gravity == 0) topLED--;

        if(mode == 1) {  //Gravcentric
            for (int i=0; i<tempsamp; i++) {
                uint8_t index = segmentSampleAvg*24+strip.now/200;
                buffer.setPixelColor(i+coordinate.width/2, SEGMENT.color_from_palette(index, false, PALETTE_SOLID_WRAP, 0));
                buffer.setPixelColor(coordinate.width/2-1-i, SEGMENT.color_from_palette(index, false, PALETTE_SOLID_WRAP, 0));
            }
            if (topLED >= 0) {
                buffer.setPixelColor(topLED+coordinate.width/2, CRGB::Gray);
                buffer.setPixelColor(coordinate.width/2-1-topLED, CRGB::Gray);
            }
        }
        else if(mode == 2) { //Gravimeter
            for (int i=0; i<tempsamp; i++) {
                uint8_t index = inoise8(i*segmentSampleAvg+strip.now, 5000+i*segmentSampleAvg);
                buffer.setPixelColor(i, color_blend(SEGCOLOR(1), SEGMENT.color_from_palette(index, false, PALETTE_SOLID_WRAP, 0), uint8_t(segmentSampleAvg*8)));
            }
            if (topLED > 0) {
                buffer.setPixelColor(topLED, SEGMENT.color_from_palette(strip.now, false, PALETTE_SOLID_WRAP, 0));
            }
        }
        else if(mode == 3) { //Gravfreq
            for (int i=0; i<tempsamp; i++) {
                float   FFT_MajorPeak = *(float*)um_data->u_data[4]; // used in mode 3: Gravfreq
                if (FFT_MajorPeak < 1) FFT_MajorPeak = 1;
                uint8_t index = (log10f(FFT_MajorPeak) - (MAX_FREQ_LOG10 - 1.78f)) * 255;
                buffer.setPixelColor(i+coordinate.width/2, SEGMENT.color_from_palette(index, false, PALETTE_SOLID_WRAP, 0));
                buffer.setPixelColor(coordinate.width/2-i-1, SEGMENT.color_from_palette(index, false, PALETTE_SOLID_WRAP, 0));
            }
            if (topLED >= 0) {
                buffer.setPixelColor(topLED+coordinate.width/2, CRGB::Gray);
                buffer.setPixelColor(coordinate.width/2-1-topLED, CRGB::Gray);
            }
        }
        else { //Gravcenter
            for (int i=0; i<tempsamp; i++) {
                uint8_t index = inoise8(i*segmentSampleAvg+strip.now, 5000+i*segmentSampleAvg);
                buffer.setPixelColor(i+coordinate.width/2, color_blend(SEGCOLOR(1), SEGMENT.color_from_palette(index, false, PALETTE_SOLID_WRAP, 0), uint8_t(segmentSampleAvg*8)));
                buffer.setPixelColor(coordinate.width/2-i-1, color_blend(SEGCOLOR(1), SEGMENT.color_from_palette(index, false, PALETTE_SOLID_WRAP, 0), uint8_t(segmentSampleAvg*8)));
            }
            if (topLED >= 0) {
                buffer.setPixelColor(topLED+coordinate.width/2, SEGMENT.color_from_palette(strip.now, false, PALETTE_SOLID_WRAP, 0));
                buffer.setPixelColor(coordinate.width/2-1-topLED, SEGMENT.color_from_palette(strip.now, false, PALETTE_SOLID_WRAP, 0));
            }
        }
        gravityCounter = (gravityCounter + 1) % gravity;
    }

private:
    const unsigned mode;
    int topLED{};
    int gravityCounter{};
};


