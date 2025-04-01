#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//////////////////////////////
//     Flow Stripe          //
//////////////////////////////
// By: ldirko  https://editor.soulmatelights.com/gallery/392-flow-led-stripe , modifed by: Andrew Tuline
class FlowStripeEffect : public BaseEffect<FlowStripeEffect> {
private:
    using Self = FlowStripeEffect;
    using Base = BaseEffect<Self>;

public:
    static constexpr const char* const metaData = "Flow Stripe@Hue speed,Effect speed;;";
    static constexpr const uint8_t effectId = FX_MODE_FLOWSTRIPE;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d1;

    using Base::Base;

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        hl = coordinate.width * 10 / 13;
        hue = strip.now / (SEGMENT.speed+1);
        t = strip.now / (SEGMENT.intensity/8+1);
    }

    uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        const unsigned i = coordinate.getXAbsolute();
        int c = (abs((int)i - hl) / hl) * 127;
        c = sin8_t(c);
        c = sin8_t(c / 2 + t);
        const byte b = sin8_t(c + t/8);
        CRGB rgb;
        rgb.setHSV(b + hue, 255, 255);
        return RGBW32(rgb.r, rgb.g, rgb.b, 0);
    }

private:
    int hl{};
    uint8_t hue{};
    uint32_t t{};
};


