#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//Railway Crossing / Christmas Fairy lights
class RailwayEffect : public BaseEffect<RailwayEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = RailwayEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Railway@!,Smoothness;1,2;!;;pal=3";
    static constexpr const uint8_t effectId = FX_MODE_RAILWAY;

    explicit RailwayEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        unsigned dur = (256 - parameters.speed) * 40;
        uint16_t rampdur = (dur * parameters.intensity) >> 8;
        if (step > dur)
        {
            //reverse direction
            step = 0;
            aux0 = !aux0;
        }
        unsigned pos = 255;
        if (rampdur != 0)
        {
            unsigned p0 = (step * 255) / rampdur;
            if (p0 < 255) pos = p0;
        }
        if (aux0) pos = 255 - pos;
        for (unsigned i = 0; i < coordinate.width; i += 2)
        {
            buffer.setPixelColor(i, parameters.color_from_palette(255 - pos, false, false, 255)); // do not use color 1 or 2, always use palette
            if (i < coordinate.width -1)
            {
                buffer.setPixelColor(i + 1, parameters.color_from_palette(pos, false, false, 255)); // do not use color 1 or 2, always use palette
            }
        }
        step += FRAMETIME;
        return true;
    }

private:
    uint32_t step{};
    uint16_t aux0{};
};


