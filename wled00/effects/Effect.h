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
    const uint8_t maxDimensions;
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
    constexpr uint8_t getMaxDimensions() const {
        return info.maxDimensions;
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

    static constexpr EffectInformation effectInformation {
        T::metaData,
        T::effectId,
        T::defaultPaletteId,
        T::maxDimensions,
        &T::makeEffect,
        &T::nextFrame,
        &T::nextRow,
        &T::getPixelColor,
    };

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

    // Hide by redefining this function in a sub class if needed.
    constexpr void nextRowImpl(const EffectCoordinate& coordinate) {
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

private:
    unsigned x{0u};
    unsigned y{0u};
};
