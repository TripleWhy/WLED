#pragma once

#include "../wled.h" // for debug prints

// template <class T> 
// struct EffectDescriptor {
//     static constexpr uint8_t getEffectId() {
//         // return static_cast<T*>(this)->getEffectIdImpl();
//         return T::getEffectIdImpl();
//     }
// };

// struct StaticEffectDescriptor : public EffectDescriptor<StaticEffectDescriptor>
// {
//     static constexpr uint8_t getEffectIdImpl() {
//         return 0;
//     }
// };

class Effect;

// Kinda emulates a v-table without needing an actual v-table.
struct EffectInformation {
    const char* metaData;
    uint8_t effectId;
    uint8_t defaultPaletteId;
    std::unique_ptr<Effect> (*makeEffect)();
};
static_assert(std::is_pod_v<EffectInformation>);

class Effect {
public:
    explicit constexpr Effect(const EffectInformation& ei) : info(ei) {}
    // explicit Effect(const EffectInformation& ei) : info(ei) {
    //     Serial.println(F("Effect()"));
    //     Serial.println(reinterpret_cast<intptr_t>(this), HEX);
    //     Serial.println(reinterpret_cast<intptr_t>(&info), HEX);
    // }
    // ~Effect() {
    //     Serial.println(F("~Effect()"));
    //     Serial.println(reinterpret_cast<intptr_t>(this), HEX);
    //     Serial.println(reinterpret_cast<intptr_t>(&info), HEX);
    //     Serial.println(reinterpret_cast<Effect*>(0)->getEffectId());
    // }
    constexpr uint8_t getEffectId() const {
        return info.effectId;
    }
    // constexpr uint8_t getDefaultPaletteId() const {
    //     return info.defaultPaletteId;
    // }
    uint8_t getDefaultPaletteId() const {
        // Serial.println(F("getDefaultPaletteId"));
        // Serial.println(reinterpret_cast<intptr_t>(this), HEX);
        // Serial.println(reinterpret_cast<intptr_t>(&info), HEX);
        // Serial.println(info.defaultPaletteId);
        return info.defaultPaletteId;
    }
    virtual void nextFrame() {}
    virtual void nextRow(int y) {}
    virtual uint32_t getPixelColor(int x, int y, uint32_t currentColor) {
        return 0;
    }
private:
    const EffectInformation& info;
};

// class EffectFactory {
//     virtual const uint8_t getId() = 0;
//     virtual const char* getMetaData() = 0;
// };

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
