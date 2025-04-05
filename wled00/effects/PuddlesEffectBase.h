#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//////////////////////
//   * PUDDLES      //
//////////////////////
// Puddles/Puddlepeak By Andrew Tuline. Merged by @dedehai
class PuddlesEffectBase : public BaseEffect<PuddlesEffectBase, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = PuddlesEffectBase;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    explicit PuddlesEffectBase(const EffectInformation& ei, bool peakdetect)
        : Base{ei, false},
          peakdetect{peakdetect}
    {
    }

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        unsigned size = 0;
        uint8_t fadeVal = map(SEGMENT.speed, 0, 255, 224, 254);
        unsigned pos = hw_random16(SEGLEN);                          // Set a random starting position.
        buffer.fadeOut(fadeVal);

        um_data_t *um_data = getAudioData();
        int volumeRaw      = *(int16_t*)um_data->u_data[1];
        uint8_t samplePeak = *(uint8_t*)um_data->u_data[3];
        uint8_t *maxVol    =  (uint8_t*)um_data->u_data[6];
        uint8_t *binNum    =  (uint8_t*)um_data->u_data[7];
        float   volumeSmth = *(float*)  um_data->u_data[0];

        if(peakdetect) {                                            // puddles peak
            *binNum = SEGMENT.custom1;                              // Select a bin.
            *maxVol = SEGMENT.custom2 / 2;                          // Our volume comparator.
            if (samplePeak == 1) {
                size = volumeSmth * SEGMENT.intensity /256 /4 + 1;  // Determine size of the flash based on the volume.
                if (pos+size>= SEGLEN) size = SEGLEN - pos;
            }
        }
        else {                                                      // puddles
            if (volumeRaw > 1) {
                size = volumeRaw * SEGMENT.intensity /256 /8 + 1;   // Determine size of the flash based on the volume.
                if (pos+size >= SEGLEN) size = SEGLEN - pos;
            }
        }

        for (unsigned i=0; i<size; i++) {                           // Flash the LED's.
            buffer.setPixelColor(pos+i, SEGMENT.color_from_palette(strip.now, false, PALETTE_SOLID_WRAP, 0));
        }
    }

private:
    const bool peakdetect;
};


