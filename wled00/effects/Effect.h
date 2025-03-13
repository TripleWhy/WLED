#pragma once

#include "../wled.h"
#include <memory>

class Effect;
class LazyColor;
class EffectCoordinate;

// Kinda emulates a v-table without needing an actual v-table.
struct EffectInformation {
    using MakeEffectFunction    = std::unique_ptr<Effect> (*)();
    using NextFrameFunction     = void     (*)(Effect* effect);
    using NextRowFunction       = void     (*)(Effect* effect, const EffectCoordinate& coordinate);
    using GetPixelColorFunction = uint32_t (*)(Effect* effect, const EffectCoordinate& coordinate, const LazyColor& currentColor);

    const char* metaData;
    const uint8_t effectId;
    const uint8_t defaultPaletteId;
    const MakeEffectFunction makeEffect;
    const NextFrameFunction nextFrame;
    const NextRowFunction nextRow;
    const GetPixelColorFunction getPixelColor;
};
static_assert(std::is_pod_v<EffectInformation>);

class Effect {
public:
    explicit constexpr Effect(const EffectInformation& ei) : info(ei) {}
    constexpr uint8_t getEffectId() const {
        return info.effectId;
    }
    constexpr uint8_t getDefaultPaletteId() const {
        return info.defaultPaletteId;
    }

    constexpr void nextFrame() {
        info.nextFrame(this);
    }
    constexpr void nextRow(const EffectCoordinate& coordinate) {
        info.nextRow(this, coordinate);
    }
    constexpr uint32_t getPixelColor(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        return info.getPixelColor(this, coordinate, currentColor);
    }

private:
    const EffectInformation& info;
};

template<typename T, typename Base = Effect>
class BaseEffect : public Base {
public:
    using Base::Base;

    static std::unique_ptr<Effect> makeEffect() {
        return std::make_unique<T>(T::effectInformation);
    }

    static void nextFrame(Effect* effect) {
        static_cast<T*>(effect)->nextFrameImpl();
    }

    static void nextRow(Effect* effect, const EffectCoordinate& coordinate) {
        static_cast<T*>(effect)->nextRowImpl(coordinate);
    }

    static uint32_t getPixelColor(Effect* effect, const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        return static_cast<T*>(effect)->getPixelColorImpl(coordinate, currentColor);
    }
};

class EffectFactory {
public:
    explicit constexpr EffectFactory(const EffectInformation& ei) : info(ei) {}
    constexpr uint8_t getEffectId() const {
        return info.effectId;
    }
    constexpr const char* getMetaData() const {
        return info.metaData;
    }
    std::unique_ptr<Effect> makeEffect() const {
        return info.makeEffect();
    }
private:
    const EffectInformation& info;
};

class EffectCoordinate {
public:
    constexpr EffectCoordinate() = default;
    constexpr EffectCoordinate(unsigned x, unsigned y) : x(x), y(y) {}
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
    // constexpr uint8_t getXRelative8() const {
    //     return scale<uint8_t>(x, SEGMENT.vWidth());
    // }
    // constexpr uint16_t getXRelative16() const {
    //     return scale<uint16_t>(x, SEGMENT.vWidth());
    // }
    // constexpr uint8_t getYRelative8() const {
    //     return scale<uint8_t>(y, SEGMENT.vHeight());
    // }
    // constexpr uint16_t getYRelative16() const {
    //     return scale<uint16_t>(y, SEGMENT.vHeight());
    // }

    constexpr void setXAbsolute(unsigned x) {
        EffectCoordinate::x = x;
    }
    constexpr void setYAbsolute(unsigned y) {
        EffectCoordinate::y = y;
    }

private:
    // This version requires calling vWidth/vHeight and computing with it every call. It would be faster not doing that.
    template<typename T>
    static constexpr inline T scale(unsigned input, unsigned inputBound) {
        constexpr T outputMax = std::numeric_limits<T>::max();
        return static_cast<T>((input * outputMax) / (std::max(2u, inputBound) - 1));
    }

private:
    unsigned x{0};
    unsigned y{0};
};
static_assert(sizeof(EffectCoordinate) == 2 * sizeof(unsigned));
