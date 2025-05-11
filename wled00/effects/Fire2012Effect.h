#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

// WLED limitation: Analog Clock overlay will NOT work when Fire2012 is active
// Fire2012 by Mark Kriegsman, July 2012
// as part of "Five Elements" shown here: http://youtu.be/knWiGsmgycY
////
// This basic one-dimensional 'fire' simulation works roughly as follows:
// There's a underlying array of 'heat' cells, that model the temperature
// at each point along the line.  Every cycle through the simulation,
// four steps are performed:
//  1) All cells cool down a little bit, losing heat to the air
//  2) The heat from each cell drifts 'up' and diffuses a little
//  3) Sometimes randomly new 'sparks' of heat are added at the bottom
//  4) The heat from each cell is rendered as a color into the leds array
//     The heat-to-color mapping uses a black-body radiation approximation.
//
// Temperature is in arbitrary units from 0 (cold black) to 255 (white hot).
//
// This simulation scales it self a bit depending on SEGLEN; it should look
// "OK" on anywhere from 20 to 100 LEDs without too much tweaking.
//
// I recommend running this simulation at anywhere from 30-100 frames per second,
// meaning an interframe delay of about 10-35 milliseconds.
//
// Looks best on a high-density LED setup (60+ pixels/meter).
//
//
// There are two main parameters you can play with to control the look and
// feel of your fire: COOLING (used in step 1 above) (Speed = COOLING), and SPARKING (used
// in step 3 above) (Effect Intensity = Sparking).
class Fire2012Effect : public BaseEffect<Fire2012Effect, BufferedEffect<EffectDimensionality::d2VStrips>> {
private:
    using Self = Fire2012Effect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2VStrips>>;

public:
    static constexpr const char metaData[] PROGMEM = "Fire 2012@Cooling,Spark rate,,2D Blur,Boost;;!;1;pal=35,sx=64,ix=160,m12=1,c2=128";
    static constexpr const uint8_t effectId = FX_MODE_FIRE_2012;

    explicit Fire2012Effect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        const unsigned strips = coordinate.height;
        const unsigned heatSize = strips * coordinate.width;
        if (!heat.resize(heatSize)) {
            return false;
        }

        const uint32_t it = strip.now >> 5; //div 32

        for (unsigned stripNr=0; stripNr<strips; stripNr++)
            runStrip(parameters, coordinate, stripNr, &heat[stripNr * coordinate.width], it);

        if (SEGMENT.is2D()) {
            uint8_t blurAmount = parameters.custom2 >> 2;
            if (blurAmount > 48) blurAmount += blurAmount-48;             // extra blur when slider > 192  (bush burn)
            if (blurAmount < 16) buffer.blur2d(0, parameters.custom2 >> 1);  // no side-burn when slider < 64 (faster)
            else buffer.blur(blurAmount);
        }

        if (it != step)
            step = it;
        return true;
    }

private:
    void runStrip(TransitionableParameters& parameters, const EffectCoordinate& coordinate, uint16_t stripNr, byte* heat, uint32_t it) {
        const uint8_t ignition = MAX(3,coordinate.width/10);  // ignition area: 10% of segment length or minimum 3 pixels

        // Step 1.  Cool down every cell a little
        for (unsigned i = 0; i < coordinate.width; i++) {
            uint8_t cool = (it != step) ? hw_random8((((20 + parameters.speed/3) * 16) / coordinate.width)+2) : hw_random8(4);
            uint8_t minTemp = (i<ignition) ? (ignition-i)/4 + 16 : 0;  // should not become black in ignition area
            uint8_t temp = qsub8(heat[i], cool);
            heat[i] = temp<minTemp ? minTemp : temp;
        }

        if (it != step) {
            // Step 2.  Heat from each cell drifts 'up' and diffuses a little
            for (int k = coordinate.width -1; k > 1; k--) {
                heat[k] = (heat[k - 1] + (heat[k - 2]<<1) ) / 3;  // heat[k-2] multiplied by 2
            }

            // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
            if (hw_random8() <= parameters.intensity) {
                uint8_t y = hw_random8(ignition);
                uint8_t boost = (17+parameters.custom3) * (ignition - y/2) / ignition; // integer math!
                heat[y] = qadd8(heat[y], hw_random8(96+2*boost,207+boost));
            }
        }

        // Step 4.  Map from heat cells to LED colors
        for (unsigned j = 0; j < coordinate.width; j++) {
            buffer.setPixelColor(j, stripNr, ColorFromPalette(SEGPALETTE, heat[j], 255, LINEARBLEND_NOWRAP));
        }
    }

private:
    SegmentAllocator<byte>::vector heat{};
    uint32_t step{};
};

