#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//////////////////////
//   * NOISEFIRE    //
//////////////////////
// I am the god of hellfire. . . Volume (only) reactive fire routine. Oh, look how short this is.
class NoisefireEffect : public BaseEffect<NoisefireEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = NoisefireEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char* const metaData = "Noisefire@!,!;;;01v;m12=2,si=0";
    static constexpr const uint8_t effectId = FX_MODE_NOISEFIRE;

    explicit NoisefireEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);
                                     // Noisefire. By Andrew Tuline.
        CRGBPalette16 myPal = CRGBPalette16(CHSV(0,255,2),    CHSV(0,255,4),    CHSV(0,255,8), CHSV(0, 255, 8),  // Fire palette definition. Lower value = darker.
                                                                                CHSV(0, 255, 16), CRGB::Red,        CRGB::Red,     CRGB::Red,
                                                                                CRGB::DarkOrange, CRGB::DarkOrange, CRGB::Orange,  CRGB::Orange,
                                                                                CRGB::Yellow,     CRGB::Orange,     CRGB::Yellow,  CRGB::Yellow);

        um_data_t *um_data = getAudioData();
        float   volumeSmth   = *(float*)  um_data->u_data[0];

        if (SEGENV.call == 0) buffer.fill(BLACK);

        for (unsigned i = 0; i < coordinate.width; i++) {
            unsigned index = inoise8(i*SEGMENT.speed/64,strip.now*SEGMENT.speed/64*coordinate.width/255);  // X location is constant, but we move along the Y at the rate of millis(). By Andrew Tuline.
            index = (255 - i*256/coordinate.width) * index/(256-SEGMENT.intensity);                       // Now we need to scale index so that it gets blacker as we get close to one of the ends.
                                                                                                                                                                                    // This is a simple y=mx+b equation that's been scaled. index/128 is another scaling.

            buffer.setPixelColor(i, ColorFromPalette(myPal, index, volumeSmth*2, LINEARBLEND)); // Use my own palette.
        }
    }

private:
};


