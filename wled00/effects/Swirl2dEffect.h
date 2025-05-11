#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/////////////////////////
//    * 2D Swirl       //
/////////////////////////
// By: Mark Kriegsman https://gist.github.com/kriegsman/5adca44e14ad025e6d3b , modified by Andrew Tuline
class Swirl2dEffect : public BaseEffect<Swirl2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = Swirl2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "Swirl@!,Sensitivity,Blur;,Bg Swirl;!;2v;ix=64,si=0";
    static constexpr const uint8_t effectId = FX_MODE_2DSWIRL;

    explicit Swirl2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        if (parameters.call == 0) {
            buffer.fill(BLACK);
        }

        const uint8_t borderWidth = 2;

        buffer.blur(parameters.custom1);

        int  i = beatsin8_t( 27*parameters.speed/255, borderWidth, cols - borderWidth);
        int  j = beatsin8_t( 41*parameters.speed/255, borderWidth, rows - borderWidth);
        int ni = (cols - 1) - i;
        int nj = (cols - 1) - j;

        um_data_t *um_data = getAudioData();
        float volumeSmth  = *(float*)   um_data->u_data[0]; //ewowi: use instead of sampleAvg???
        int   volumeRaw   = *(int16_t*) um_data->u_data[1];

        buffer.addPixelColor( i, j, ColorFromPalette(SEGPALETTE, (strip.now / 11 + volumeSmth*4), volumeRaw * parameters.intensity / 64, LINEARBLEND)); //CHSV( ms / 11, 200, 255);
        buffer.addPixelColor( j, i, ColorFromPalette(SEGPALETTE, (strip.now / 13 + volumeSmth*4), volumeRaw * parameters.intensity / 64, LINEARBLEND)); //CHSV( ms / 13, 200, 255);
        buffer.addPixelColor(ni,nj, ColorFromPalette(SEGPALETTE, (strip.now / 17 + volumeSmth*4), volumeRaw * parameters.intensity / 64, LINEARBLEND)); //CHSV( ms / 17, 200, 255);
        buffer.addPixelColor(nj,ni, ColorFromPalette(SEGPALETTE, (strip.now / 29 + volumeSmth*4), volumeRaw * parameters.intensity / 64, LINEARBLEND)); //CHSV( ms / 29, 200, 255);
        buffer.addPixelColor( i,nj, ColorFromPalette(SEGPALETTE, (strip.now / 37 + volumeSmth*4), volumeRaw * parameters.intensity / 64, LINEARBLEND)); //CHSV( ms / 37, 200, 255);
        buffer.addPixelColor(ni, j, ColorFromPalette(SEGPALETTE, (strip.now / 41 + volumeSmth*4), volumeRaw * parameters.intensity / 64, LINEARBLEND)); //CHSV( ms / 41, 200, 255);
        return true;
    }

private:
};


#endif //WLED_DISABLE_2D
