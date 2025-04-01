#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Best of both worlds from Palette and Spot effects. By Aircoookie
 */
class FlowEffect : public BaseEffect<FlowEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = FlowEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char* const metaData = "Flow@!,Zones;;!;;m12=1";
    static constexpr const uint8_t effectId = FX_MODE_FLOW;

    explicit FlowEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        unsigned counter = 0;
        if (SEGMENT.speed != 0)
        {
            counter = strip.now * ((SEGMENT.speed >> 2) +1);
            counter = counter >> 8;
        }

        unsigned maxZones = coordinate.width / 6; //only looks good if each zone has at least 6 LEDs
        unsigned zones = (SEGMENT.intensity * maxZones) >> 8;
        if (zones & 0x01) zones++; //zones must be even
        if (zones < 2) zones = 2;
        unsigned zoneLen = coordinate.width / zones;
        unsigned offset = (coordinate.width - zones * zoneLen) >> 1;

        buffer.fill(SEGMENT.color_from_palette(-counter, false, true, 255));

        for (unsigned z = 0; z < zones; z++)
        {
            unsigned pos = offset + z * zoneLen;
            for (unsigned i = 0; i < zoneLen; i++)
            {
                unsigned colorIndex = (i * 255 / zoneLen) - counter;
                unsigned led = (z & 0x01) ? i : (zoneLen -1) -i;
                if (SEGMENT.reverse) led = (zoneLen -1) -led;
                buffer.setPixelColor(pos + led, SEGMENT.color_from_palette(colorIndex, false, true, 255));
            }
        }
    }

private:
};


