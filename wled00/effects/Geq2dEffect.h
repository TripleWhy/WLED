#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/////////////////////////
//     ** 2D GEQ       //
/////////////////////////
// By Will Tatam. Code reduction by Ewoud Wijma.
class Geq2dEffect : public BaseEffect<Geq2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = Geq2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "GEQ@Fade speed,Ripple decay,# of bands,,,Color bars;!,,Peaks;!;2f;c1=255,c2=64,pal=11,si=0";
    static constexpr const uint8_t effectId = FX_MODE_2DGEQ;

    explicit Geq2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        const int NUM_BANDS = map(SEGMENT.custom1, 0, 255, 1, 16);
        const int cols = coordinate.width;
        const int rows = coordinate.height;

        previousBarHeight.resize(coordinate.width);
        if (previousBarHeight.size() != coordinate.width) {
            previousBarHeight.clear();
            return;
        }
        previousBarHeight.shrink_to_fit();

        um_data_t *um_data = getAudioData();
        uint8_t *fftResult = (uint8_t*)um_data->u_data[2];

        if (SEGENV.call == 0) for (int i=0; i<cols; i++) previousBarHeight[i] = 0;

        bool rippleTime = false;
        if (strip.now - step >= (256U - SEGMENT.intensity)) {
            step = strip.now;
            rippleTime = true;
        }

        int fadeoutDelay = (256 - SEGMENT.speed) / 64;
        if ((fadeoutDelay <= 1 ) || ((SEGENV.call % fadeoutDelay) == 0)) buffer.fadeToBlackBy(SEGMENT.speed);

        for (int x=0; x < cols; x++) {
            uint8_t  band       = map(x, 0, cols, 0, NUM_BANDS);
            if (NUM_BANDS < 16) band = map(band, 0, NUM_BANDS - 1, 0, 15); // always use full range. comment out this line to get the previous behaviour.
            band = constrain(band, 0, 15);
            unsigned colorIndex = band * 17;
            int barHeight  = map(fftResult[band], 0, 255, 0, rows); // do not subtract -1 from rows here
            if (barHeight > previousBarHeight[x]) previousBarHeight[x] = barHeight; //drive the peak up

            uint32_t ledColor = BLACK;
            for (int y=0; y < barHeight; y++) {
                if (SEGMENT.check1) //color_vertical / color bars toggle
                    colorIndex = map(y, 0, rows-1, 0, 255);

                ledColor = SEGMENT.color_from_palette(colorIndex, false, PALETTE_SOLID_WRAP, 0);
                buffer.setPixelColor(x, rows-1 - y, ledColor);
            }
            if (previousBarHeight[x] > 0)
                buffer.setPixelColor(x, rows - previousBarHeight[x], (SEGCOLOR(2) != BLACK) ? SEGCOLOR(2) : ledColor);

            if (rippleTime && previousBarHeight[x]>0) previousBarHeight[x]--;    //delay/ripple effect
        }
    }

private:
    SegmentAllocator<uint16_t>::vector previousBarHeight{}; //array of previous bar heights per frequency band
    uint32_t step{};
};


#endif //WLED_DISABLE_2D
