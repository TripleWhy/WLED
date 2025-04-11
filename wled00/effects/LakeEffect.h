#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//Calm effect, like a lake at night
class LakeEffect : public BaseEffect<LakeEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = LakeEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Lake@!;Fx;!";
    static constexpr const uint8_t effectId = FX_MODE_LAKE;

    explicit LakeEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        unsigned sp = SEGMENT.speed/10;
        int wave1 = beatsin8_t(sp +2, -64,64);
        int wave2 = beatsin8_t(sp +1, -64,64);
        int wave3 = beatsin8_t(sp +2,   0,80);

        for (unsigned i = 0; i < coordinate.width; i++)
        {
            int index = cos8_t((i*15)+ wave1)/2 + cubicwave8((i*23)+ wave2)/2;
            uint8_t lum = (index > wave3) ? index - wave3 : 0;
            buffer.setPixelColor(i, SEGMENT.color_from_palette(index, false, false, 0, lum));
        }
        return true;
    }

private:
};


