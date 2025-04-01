#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

// combined function from original pride and colorwaves
class ColorwavesPrideEffectBase : public BaseEffect<ColorwavesPrideEffectBase, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = ColorwavesPrideEffectBase;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d1;

    explicit ColorwavesPrideEffectBase(const EffectInformation& ei, bool isPride2015)
        : Base{ei, false},
          isPride2015{isPride2015}
    {
    }

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        unsigned duration = 10 + SEGMENT.speed;

        uint8_t sat8 = isPride2015 ? beatsin88_t(87, 220, 250) : 255;
        unsigned brightdepth = beatsin88_t(341, 96, 224);
        unsigned brightnessthetainc16 = beatsin88_t(203, (25 * 256), (40 * 256));
        unsigned msmultiplier = beatsin88_t(147, 23, 60);

        unsigned hue16 = sHue16;
        unsigned hueinc16 = isPride2015 ? beatsin88_t(113, 1, 3000) :
                                          beatsin88_t(113, 60, 300) * SEGMENT.intensity * 10 / 255;

        sPseudotime += duration * msmultiplier;
        sHue16 += duration * beatsin88_t(400, 5, 9);
        unsigned brightnesstheta16 = sPseudotime;

        for (unsigned i = 0; i < coordinate.width; i++) {
            hue16 += hueinc16;
            uint8_t hue8;

            if (isPride2015) {
                hue8 = hue16 >> 8;
            } else {
                unsigned h16_128 = hue16 >> 7;
                hue8 = (h16_128 & 0x100) ? (255 - (h16_128 >> 1)) : (h16_128 >> 1);
            }

            brightnesstheta16 += brightnessthetainc16;
            unsigned b16 = sin16_t(brightnesstheta16) + 32768;
            unsigned bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
            uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
            bri8 += (255 - brightdepth);

            if (isPride2015) {
                CRGB newcolor = CHSV(hue8, sat8, bri8);
                buffer.blendPixelColor(i, RGBW32(newcolor.r, newcolor.g, newcolor.b, 0), 64);
            } else {
                buffer.blendPixelColor(i, SEGMENT.color_from_palette(hue8, false, PALETTE_SOLID_WRAP, 0, bri8), 128);
            }
        }
    }

private:
    const bool isPride2015;
    unsigned sPseudotime{};
    unsigned sHue16{};
};
