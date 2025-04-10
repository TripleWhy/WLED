#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Emulates a traffic light.
 */
class TrafficLightEffect : public BaseEffect<TrafficLightEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = TrafficLightEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Traffic Light@!,US style;,!;!";
    static constexpr const uint8_t effectId = FX_MODE_TRAFFIC_LIGHT;

    explicit TrafficLightEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        for (unsigned i=0; i < coordinate.width; i++)
            buffer.setPixelColor(i, SEGMENT.color_from_palette(i, true, PALETTE_SOLID_WRAP, 1));
        uint32_t mdelay = 500;
        for (unsigned i = 0; i < coordinate.width-2 ; i+=3)
        {
            switch (aux0)
            {
                case 0: buffer.setPixelColor(i, 0x00FF0000); mdelay = 150 + (100 * (uint32_t)(255 - SEGMENT.speed));break;
                case 1: buffer.setPixelColor(i, 0x00FF0000); mdelay = 150 + (20 * (uint32_t)(255 - SEGMENT.speed)); buffer.setPixelColor(i+1, 0x00EECC00); break;
                case 2: buffer.setPixelColor(i+2, 0x0000FF00); mdelay = 150 + (100 * (uint32_t)(255 - SEGMENT.speed));break;
                case 3: buffer.setPixelColor(i+1, 0x00EECC00); mdelay = 150 + (20 * (uint32_t)(255 - SEGMENT.speed));break;
            }
        }

        if (strip.now - step > mdelay)
        {
            aux0++;
            if (aux0 == 1 && SEGMENT.intensity > 140) aux0 = 2; //skip Red + Amber, to get US-style sequence
            if (aux0 > 3) aux0 = 0;
            step = strip.now;
        }
    }

private:
    uint32_t step{};
    uint16_t aux0{};
};


