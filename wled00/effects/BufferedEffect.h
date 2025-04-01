#pragma once

#include "../FX.h"
#include "Effect.h"

// The buffered Effect is split into two parts:
//  1. BufferedEffectBase contains everything that doesn't depend on a template parameter
//  2. BufferedEffect contains everything that depends on a template parameter.
// Don't use BufferedEffectBase directly, use BufferedEffect.

class BufferedEffectBase : public Effect {
protected:
    class PixelBufferBase {
        template<EffectDimensionality>
        friend class BufferedEffect;
    public:
        void fill(uint32_t color) {
            std::fill(pixels.begin(), pixels.end(), color);
        }

        /*
         * fade out function, higher rate = quicker fade
         * fading is highly dependant on frame rate (higher frame rates, faster fading)
         * each frame will fade at max 9% or as little as 0.8%
         */
        //TODO there is room for optimization here
        void fade(uint32_t targetColor, uint8_t rate) {
            rate = (256-rate) >> 1;
            const int mappedRate = 256 / (rate + 1);

            for (uint32_t& color : pixels) {
                if (color == targetColor) {
                    continue; // already at target color
                }
                for (int i = 0; i < 32; i += 8) {
                    uint8_t c2 = (targetColor>>i);  // get background channel
                    uint8_t c1 = (color>>i);      // get foreground channel
                    // we can't use bitshift since we are using int
                    int delta = (c2 - c1) * mappedRate / 256;
                    // if fade isn't complete, make sure delta is at least 1 (fixes rounding issues)
                    if (delta == 0) {
                        delta += (c2 == c1) ? 0 : (c2 > c1) ? 1 : -1;
                    }
                    // stuff new value back into color
                    color &= ~(0xFF<<i);
                    color |= ((c1 + delta) & 0xFF) << i;
                }
            }
        }

    protected:
        uint32_t getPixelColorLinear(unsigned i) const {
            if (static_cast<size_t>(i) >= pixels.size()) [[unlikely]] {
                Serial.printf("BufferedEffect::PixelBuffer::getPixelColor: %d >= %u\n", i, pixels.size());
                std::terminate();
            }
            return pixels[static_cast<size_t>(i)];
        }
        void setPixelColorLinear(unsigned i, uint32_t c) {
            if (static_cast<size_t>(i) >= pixels.size()) [[unlikely]] {
                std::terminate();
            }
            pixels[static_cast<size_t>(i)] = c;
        }
        void blendPixelColorLinear(unsigned n, uint32_t color, uint8_t blend) {
            setPixelColorLinear(n, color_blend(getPixelColorLinear(n), color, blend));
        }

    private:
        std::vector<uint32_t> pixels;
    };

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
protected:
    class PixelBuffer : public PixelBufferBase {
    public:
        inline void setPixelColor(unsigned x, uint32_t color)
        {
            static_assert(dimensionality == EffectDimensionality::d1, "Use more coordinate arguments.");
            setPixelColorLinear(x, color);
        }

        inline void setPixelColor(unsigned x, unsigned y, uint32_t color)
        {
            static_assert(dimensionality != EffectDimensionality::d1, "Use fewer coordinate arguments.");
            setPixelColorLinear(convertToLinear(x, y), color);
        }

        inline uint32_t getPixelColor(unsigned x)
        {
            static_assert(dimensionality == EffectDimensionality::d1, "Use more coordinate arguments.");
            return getPixelColorLinear(x);
        }

        inline uint32_t getPixelColor(unsigned x, unsigned y)
        {
            static_assert(dimensionality != EffectDimensionality::d1, "Use fewer coordinate arguments.");
            return getPixelColorLinear(convertToLinear(x, y));
        }

        inline void blendPixelColor(unsigned x, uint32_t color, uint8_t blend)
        {
            static_assert(dimensionality == EffectDimensionality::d1, "Use more coordinate arguments.");
            blendPixelColorLinear(x, color, blend);
        }

        inline void blendPixelColor(unsigned x, unsigned y, uint32_t color, uint8_t blend)
        {
            static_assert(dimensionality != EffectDimensionality::d1, "Use fewer coordinate arguments.");
            blendPixelColorLinear(convertToLinear(x, y), color, blend);
        }
    };

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
        if (!initBufferWithCurrentState) {
            return;
        }
        const unsigned width = Segment::getEffectWidth<dimensionality>();
        const unsigned height = Segment::getEffectHeight<dimensionality>();
        const size_t length = width * height;

        buffer.pixels.resize(length); // don't initialize the buffer with specific values
        if (buffer.pixels.size() != length) {
            buffer.pixels.clear();
            return;
        }

        for (unsigned y = 0u; y < height; ++y) {
            for (unsigned x = 0u; x < width; ++x) {
                buffer.pixels[convertToLinear(x, y)] = SEGMENT.getPixelColor(convertSegmentPixelIndex(x, y));
            }
        }
    }

public:
    void nextFrameImpl(const EffectCoordinate& coordinate) {
        const size_t length = Segment::getEffectWidth<dimensionality>() * Segment::getEffectHeight<dimensionality>();
        buffer.pixels.resize(length, 0u);
        if (buffer.pixels.size() != length) {
            buffer.pixels.clear();
            return;
        }
    }

    uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        return buffer.getPixelColorLinear(convertToLinear(coordinate.getXAbsolute(), coordinate.getYAbsolute()));
    }

private:
    static inline unsigned convertToLinear(unsigned x, unsigned y) {
        if constexpr (dimensionality == EffectDimensionality::d0) {
            return 0;
        } else if constexpr (dimensionality == EffectDimensionality::d1) {
            return x;
        } else {
            return y * Segment::getEffectWidth<dimensionality>() + x;
        }
    }
    static inline unsigned convertSegmentPixelIndex(unsigned x, unsigned y) {
        if constexpr (dimensionality == EffectDimensionality::d2VStrips) {
            return ((x) | (int((y) + 1) << 16)); // original indexToVStrip
        } else {
            return convertToLinear(x, y);
        }
    }

protected:
    PixelBuffer buffer{};
};
