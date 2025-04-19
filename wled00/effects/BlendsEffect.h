#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
  Blends random colors across palette
  Modified, originally by Mark Kriegsman https://gist.github.com/kriegsman/1f7ccbbfa492a73c015e
*/
class BlendsEffect : public BaseEffect<BlendsEffect> {
private:
    using Self = BlendsEffect;
    using Base = BaseEffect<Self>;

public:
    static constexpr const char metaData[] PROGMEM = "Blends@Shift speed,Blend speed;;!";
    static constexpr const uint8_t effectId = FX_MODE_BLENDS;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d1;

    using Base::Base;

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        pixelLen = coordinate.width > UINT8_MAX ? UINT8_MAX : coordinate.width;
        if (!pixels.resize(pixelLen)) {
            return false;
        }

        uint8_t blendSpeed = map(SEGMENT.intensity, 0, UINT8_MAX, 10, 128);
        unsigned shift = (strip.now * ((SEGMENT.speed >> 3) +1)) >> 8;

        for (unsigned i = 0; i < pixelLen; i++) {
            pixels[i] = color_blend(pixels[i], SEGMENT.color_from_palette(shift + quadwave8((i + 1) * 16), false, PALETTE_SOLID_WRAP, 255), blendSpeed);
            shift += 3;
        }
        return true;
    }

    uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        return pixels[coordinate.getXAbsolute() % pixelLen];
    }

private:
    SegmentAllocator<uint32_t>::vector pixels{};
    unsigned pixelLen{};
};


