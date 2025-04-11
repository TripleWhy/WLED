#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

class SpotsEffectBase : public BaseEffect<SpotsEffectBase, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = SpotsEffectBase;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    explicit SpotsEffectBase(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate, uint16_t threshold) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        if (!SEGMENT.check2)
            buffer.fill(SEGCOLOR(1));

        unsigned maxZones = coordinate.width >> 2;
        unsigned zones = 1 + ((SEGMENT.intensity * maxZones) >> 8);
        unsigned zoneLen = coordinate.width / zones;
        unsigned offset = (coordinate.width - zones * zoneLen) >> 1;

        for (unsigned z = 0; z < zones; z++)
        {
            unsigned pos = offset + z * zoneLen;
            for (unsigned i = 0; i < zoneLen; i++)
            {
                unsigned wave = triwave16((i * 0xFFFF) / zoneLen);
                if (wave > threshold) {
                    unsigned index = 0 + pos + i;
                    unsigned s = (wave - threshold)*255 / (0xFFFF - threshold);
                    buffer.setPixelColor(index, color_blend(SEGMENT.color_from_palette(index, true, PALETTE_SOLID_WRAP, 0), SEGCOLOR(1), uint8_t(255-s)));
                }
            }
        }
        return true;
    }

private:
};


