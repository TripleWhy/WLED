#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

// Distortion waves - ldirko
// https://editor.soulmatelights.com/gallery/1089-distorsion-waves
// adapted for WLED by @blazoncek
class Distortionwaves2dEffect : public BaseEffect<Distortionwaves2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = Distortionwaves2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "Distortion Waves@!,Scale;;;2";
    static constexpr const uint8_t effectId = FX_MODE_2DDISTORTIONWAVES;

    explicit Distortionwaves2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        uint8_t speed = parameters.speed/32;
        uint8_t scale = parameters.intensity/32;

        uint8_t  w = 2;

        unsigned a  = strip.now/32;
        unsigned a2 = a/2;
        unsigned a3 = a/3;

        unsigned cx =  beatsin8_t(10-speed,0,cols-1)*scale;
        unsigned cy =  beatsin8_t(12-speed,0,rows-1)*scale;
        unsigned cx1 = beatsin8_t(13-speed,0,cols-1)*scale;
        unsigned cy1 = beatsin8_t(15-speed,0,rows-1)*scale;
        unsigned cx2 = beatsin8_t(17-speed,0,cols-1)*scale;
        unsigned cy2 = beatsin8_t(14-speed,0,rows-1)*scale;
        
        unsigned xoffs = 0;
        for (int x = 0; x < cols; x++) {
            xoffs += scale;
            unsigned yoffs = 0;

            for (int y = 0; y < rows; y++) {
                 yoffs += scale;

                byte rdistort = cos8_t((cos8_t(((x<<3)+a )&255)+cos8_t(((y<<3)-a2)&255)+a3   )&255)>>1; 
                byte gdistort = cos8_t((cos8_t(((x<<3)-a2)&255)+cos8_t(((y<<3)+a3)&255)+a+32 )&255)>>1; 
                byte bdistort = cos8_t((cos8_t(((x<<3)+a3)&255)+cos8_t(((y<<3)-a) &255)+a2+64)&255)>>1; 

                byte valueR = rdistort+ w*  (a- ( ((xoffs - cx)  * (xoffs - cx)  + (yoffs - cy)  * (yoffs - cy))>>7  ));
                byte valueG = gdistort+ w*  (a2-( ((xoffs - cx1) * (xoffs - cx1) + (yoffs - cy1) * (yoffs - cy1))>>7 ));
                byte valueB = bdistort+ w*  (a3-( ((xoffs - cx2) * (xoffs - cx2) + (yoffs - cy2) * (yoffs - cy2))>>7 ));

                valueR = gamma8(cos8_t(valueR));
                valueG = gamma8(cos8_t(valueG));
                valueB = gamma8(cos8_t(valueB));

                buffer.setPixelColor(x, y, RGBW32(valueR, valueG, valueB, 0)); 
            }
        }
        return true;
    }

private:
};


#endif //WLED_DISABLE_2D
