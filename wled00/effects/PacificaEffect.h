#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//  "Pacifica"
//  Gentle, blue-green ocean waves.
//  December 2019, Mark Kriegsman and Mary Corey March.
//  For Dan.
//
//
// In this animation, there are four "layers" of waves of light.
//
// Each layer moves independently, and each is scaled separately.
//
// All four wave layers are added together on top of each other, and then
// another filter is applied that adds "whitecaps" of brightness where the
// waves line up with each other more.  Finally, another pass is taken
// over the led array to 'deepen' (dim) the blues and greens.
//
// The speed and scale and motion each layer varies slowly within independent
// hand-chosen ranges, which is why the code has a lot of low-speed 'beatsin8' functions
// with a lot of oddly specific numeric ranges.
//
// These three custom blue-green color palettes were inspired by the colors found in
// the waters off the southern coast of California, https://goo.gl/maps/QQgd97jjHesHZVxQ7
//
// Modified for WLED, based on https://github.com/FastLED/FastLED/blob/master/examples/Pacifica/Pacifica.ino
class PacificaEffect : public BaseEffect<PacificaEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = PacificaEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Pacifica@!,Angle;;!;;pal=51";
    static constexpr const uint8_t effectId = FX_MODE_PACIFICA;

    explicit PacificaEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        uint32_t nowOld = strip.now;

        CRGBPalette16 pacifica_palette_1 =
            { 0x000507, 0x000409, 0x00030B, 0x00030D, 0x000210, 0x000212, 0x000114, 0x000117,
                0x000019, 0x00001C, 0x000026, 0x000031, 0x00003B, 0x000046, 0x14554B, 0x28AA50 };
        CRGBPalette16 pacifica_palette_2 =
            { 0x000507, 0x000409, 0x00030B, 0x00030D, 0x000210, 0x000212, 0x000114, 0x000117,
                0x000019, 0x00001C, 0x000026, 0x000031, 0x00003B, 0x000046, 0x0C5F52, 0x19BE5F };
        CRGBPalette16 pacifica_palette_3 =
            { 0x000208, 0x00030E, 0x000514, 0x00061A, 0x000820, 0x000927, 0x000B2D, 0x000C33,
                0x000E39, 0x001040, 0x001450, 0x001860, 0x001C70, 0x002080, 0x1040BF, 0x2060FF };

        if (SEGMENT.palette) {
            pacifica_palette_1 = SEGPALETTE;
            pacifica_palette_2 = SEGPALETTE;
            pacifica_palette_3 = SEGPALETTE;
        }

        // Increment the four "color index start" counters, one for each wave layer.
        // Each is incremented at a different speed, and the speeds vary over time.
        unsigned sCIStart1 = aux0, sCIStart2 = aux1, sCIStart3 = step & 0xFFFF, sCIStart4 = (step >> 16);
        uint32_t deltams = (FRAMETIME >> 2) + ((FRAMETIME * parameters.speed) >> 7);
        uint64_t deltat = (strip.now >> 2) + ((strip.now * parameters.speed) >> 7);
        strip.now = deltat;

        unsigned speedfactor1 = beatsin16_t(3, 179, 269);
        unsigned speedfactor2 = beatsin16_t(4, 179, 269);
        uint32_t deltams1 = (deltams * speedfactor1) / 256;
        uint32_t deltams2 = (deltams * speedfactor2) / 256;
        uint32_t deltams21 = (deltams1 + deltams2) / 2;
        sCIStart1 += (deltams1 * beatsin88_t(1011,10,13));
        sCIStart2 -= (deltams21 * beatsin88_t(777,8,11));
        sCIStart3 -= (deltams1 * beatsin88_t(501,5,7));
        sCIStart4 -= (deltams2 * beatsin88_t(257,4,6));
        aux0 = sCIStart1; aux1 = sCIStart2;
        step = (sCIStart4 << 16) | (sCIStart3 & 0xFFFF);

        // Clear out the LED array to a dim background blue-green
        //buffer.fill(132618);

        unsigned basethreshold = beatsin8_t( 9, 55, 65);
        unsigned wave = beat8( 7 );

        for (unsigned i = 0; i < coordinate.width; i++) {
            CRGB c = CRGB(2, 6, 10);
            // Render each of four layers, with different scales and speeds, that vary over time
            c += pacifica_one_layer(parameters, i, pacifica_palette_1, sCIStart1, beatsin16_t(3, 11 * 256, 14 * 256), beatsin8_t(10, 70, 130), 0-beat16(301));
            c += pacifica_one_layer(parameters, i, pacifica_palette_2, sCIStart2, beatsin16_t(4,  6 * 256,  9 * 256), beatsin8_t(17, 40,  80),   beat16(401));
            c += pacifica_one_layer(parameters, i, pacifica_palette_3, sCIStart3,                         6 * 256 , beatsin8_t(9, 10,38)   , 0-beat16(503));
            c += pacifica_one_layer(parameters, i, pacifica_palette_3, sCIStart4,                         5 * 256 , beatsin8_t(8, 10,28)   ,   beat16(601));

            // Add extra 'white' to areas where the four layers of light have lined up brightly
            unsigned threshold = scale8( sin8_t( wave), 20) + basethreshold;
            wave += 7;
            unsigned l = c.getAverageLight();
            if (l > threshold) {
                unsigned overage = l - threshold;
                unsigned overage2 = qadd8(overage, overage);
                c += CRGB(overage, overage2, qadd8(overage2, overage2));
            }

            //deepen the blues and greens
            c.blue  = scale8(c.blue,  145);
            c.green = scale8(c.green, 200);
            c |= CRGB( 2, 5, 7);

            buffer.setPixelColor(i, RGBW32(c.r, c.g, c.b, 0));
        }

        strip.now = nowOld;
        return true;
    }

private:
    // Add one layer of waves into the led array
    static CRGB pacifica_one_layer(TransitionableParameters& parameters, uint16_t i, const CRGBPalette16& p, uint16_t cistart, uint16_t wavescale, uint8_t bri, uint16_t ioff)
    {
        unsigned ci = cistart;
        unsigned waveangle = ioff;
        unsigned wavescale_half = (wavescale >> 1) + 20;

        waveangle += ((120 + parameters.intensity) * i); //original 250 * i
        unsigned s16 = sin16_t(waveangle) + 32768;
        unsigned cs = scale16(s16, wavescale_half) + wavescale_half;
        ci += (cs * i);
        unsigned sindex16 = sin16_t(ci) + 32768;
        unsigned sindex8 = scale16(sindex16, 240);
        return CRGB(p.ColorFromPalette(sindex8, bri, LINEARBLEND));
    }

private:
    uint32_t step{};
    uint16_t aux0{};
    uint16_t aux1{};
};


