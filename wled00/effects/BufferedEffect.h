#pragma once

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

public:
    static constexpr const EffectDimensionality dimensionality = _dimensionality;
    static_assert(_dimensionality != EffectDimensionality::d0, "0D effect buffers don't make sense... or do they? In any case they are not currently implemented.");

protected:
    explicit BufferedEffect(const EffectInformation& ei, bool initBufferWithCurrentState)
        : Base{ei}
    {
        if (initBufferWithCurrentState) {
            buffer.copySegmentPixels();
        }
    }

public:
    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if constexpr (dimensionality == EffectDimensionality::d1) {
            return buffer.resize(coordinate.width);
        } else {
            return buffer.resize(coordinate.width, coordinate.height);
        }
    }

    uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        if constexpr (dimensionality == EffectDimensionality::d1) {
            return buffer.getPixelColor(coordinate.getXAbsolute());
        } else {
            return buffer.getPixelColor(coordinate.getXAbsolute(), coordinate.getYAbsolute());
        }
    }

protected:
    PixelBuffer<_dimensionality> buffer{};
};
