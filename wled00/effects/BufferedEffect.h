#pragma once

#include "../wled.h"
#include "Effect.h"
#include "PixelBuffer.h"

// The buffered Effect is split into two parts:
//  1. BufferedEffectBase contains everything that doesn't depend on a template parameter
//  2. BufferedEffect contains everything that depends on a template parameter.
// Don't use BufferedEffectBase directly, use BufferedEffect.

class BufferedEffectBase : public Effect {
private:
    using Self = BufferedEffectBase;
    using Base = Effect;

    template<EffectDimensionality>
    friend class BufferedEffect;

protected:
    using Base::Base;
};

template<EffectDimensionality _dimensionality>
class BufferedEffect : public BufferedEffectBase {
private:
    using Self = BufferedEffect;
    using Base = BufferedEffectBase;
    using Buffer = PixelBuffer<_dimensionality>;

public:
    static constexpr const EffectDimensionality dimensionality = _dimensionality;
    static_assert(_dimensionality != EffectDimensionality::d0, "0D effect buffers don't make sense... or do they? In any case they are not currently implemented.");

protected:
    explicit BufferedEffect(const EffectInformation& ei, bool initBufferWithCurrentState)
        : Base{ei}
    {
        if (!initBufferWithCurrentState) {
            return;
        }
        const unsigned width = Segment::getEffectWidth<dimensionality>();
        const unsigned height = Segment::getEffectHeight<dimensionality>();
        const size_t length = width * height;

        if (!buffer.pixels.resize(length)) {
            return;
        }

        for (unsigned y = 0u; y < height; ++y) {
            for (unsigned x = 0u; x < width; ++x) {
                buffer.pixels[Buffer::convertToLinear(x, y)] = SEGMENT.getPixelColor(Buffer::convertToSegmentPixelIndex(x, y));
            }
        }
    }

public:
    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        return buffer.pixels.resize(coordinate.width * coordinate.height);
    }

    uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        return buffer.getPixelColorLinear(Buffer::convertToLinear(coordinate.getXAbsolute(), coordinate.getYAbsolute()));
    }

protected:
    Buffer buffer{};
};
