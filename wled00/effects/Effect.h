#pragma once

#include <memory>
#include "../memory/CircularAllocator.h"
#include "../wled.h"
#include "effectUtils.h"

class Effect;
class LazyColor;
class EffectCoordinate;

enum class EffectDimensionality : uint8_t {
    d0 = 0,
    d1 = 1,
    d2 = 2,
    //reserved: d3 = 3,
    d2VStrips = 4,
};

// Kinda emulates a v-table without needing an actual v-table.
struct EffectInformation {
    using MakeEffectFunction    = SegmentAllocator<Effect>::unique_ptr (*)();
    using NextFrameFunction     = void     (*)(Effect* effect, const EffectCoordinate& coordinate);
    using NextRowFunction       = void     (*)(Effect* effect, const EffectCoordinate& coordinate);
    using GetPixelColorFunction = uint32_t (*)(Effect* effect, const EffectCoordinate& coordinate, const LazyColor& currentColor);

    const char* metaData;
    const uint8_t effectId;
    const EffectDimensionality dimensionality;
    const MakeEffectFunction makeEffect;
    const NextFrameFunction nextFrame;
    const NextRowFunction nextRow;
    const GetPixelColorFunction getPixelColor;
};
static_assert(std::is_pod_v<EffectInformation>);

class Effect {
public:
    explicit constexpr Effect(const EffectInformation& ei) : info(ei) {}
    virtual ~Effect() = default;
    constexpr uint8_t getEffectId() const {
        return info.effectId;
    }
    constexpr EffectDimensionality getDimensionality() const {
        return info.dimensionality;
    }
    constexpr void nextFrame(const EffectCoordinate& coordinate) {
        info.nextFrame(this, coordinate);
    }
    constexpr void nextRow(const EffectCoordinate& coordinate) {
        info.nextRow(this, coordinate);
    }
    constexpr uint32_t getPixelColor(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        return info.getPixelColor(this, coordinate, currentColor);
    }

    template<typename Vector>
    static inline bool resizeVector(Vector& vector, size_t size, bool preserveContent = false) {
        if (vector.size() == size) {
            return true;
        }
        if (!preserveContent) {
            vector.clear();
            vector.shrink_to_fit();
        }
        vector.resize(size);
        if (vector.size() != size) {
            vector.clear();
            vector.shrink_to_fit();
            return false;
        }
        vector.shrink_to_fit();
        return true;
    }

private:
    const EffectInformation& info;
};

template<typename T, typename Base = Effect>
class BaseEffect : public Base {
public:
    using Base::Base;

    static constexpr EffectInformation effectInformation {
        T::metaData,
        T::effectId,
        T::dimensionality,
        &T::makeEffect,
        &T::nextFrame,
        &T::nextRow,
        &T::getPixelColor,
    };

    static SegmentAllocator<Effect>::unique_ptr makeEffect() {
        // Don't try `new T` here, that would not use the allocator.
        typename SegmentAllocator<T>::unique_ptr t = SegmentAllocator<T>::make_unique(T::effectInformation);
        return SegmentAllocator<Effect>::unique_ptr(static_cast<Effect*>(t.release()));
    }

    static void nextFrame(Effect* effect, const EffectCoordinate& coordinate) {
        static_cast<T*>(effect)->nextFrameImpl(coordinate);
    }

    static void nextRow(Effect* effect, const EffectCoordinate& coordinate) {
        static_cast<T*>(effect)->nextRowImpl(coordinate);
    }

    static uint32_t getPixelColor(Effect* effect, const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        return static_cast<T*>(effect)->getPixelColorImpl(coordinate, currentColor);
    }

    // Hide by redefining this function in a sub class if needed.
    constexpr void nextRowImpl(const EffectCoordinate& coordinate) {
    }
};

class EffectCoordinate {
public:
    constexpr EffectCoordinate(unsigned width, unsigned height) : width{width}, height{height} {}
    constexpr EffectCoordinate(const EffectCoordinate&) = delete;
    constexpr EffectCoordinate(EffectCoordinate&&) = delete;
    constexpr EffectCoordinate& operator=(const EffectCoordinate&) = delete;
    constexpr EffectCoordinate& operator=(EffectCoordinate&&) = delete;

    constexpr unsigned getXAbsolute() const {
        return x;
    }
    constexpr unsigned getYAbsolute() const {
        return y;
    }
    constexpr void setXAbsolute(unsigned x) {
        EffectCoordinate::x = x;
    }
    constexpr void setYAbsolute(unsigned y) {
        EffectCoordinate::y = y;
    }

public:
    const unsigned width{0u};
    const unsigned height{0u};

private:
    unsigned x{0u};
    unsigned y{0u};
};
