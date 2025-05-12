#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//////////////////////
//    ** Blurz      //
//////////////////////
// Blurz. By Andrew Tuline.
class BlurzEffect : public BaseEffect<BlurzEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = BlurzEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Blurz@Fade rate,Blur;!,Color mix;!;1f;m12=0,si=0";
    static constexpr const uint8_t effectId = FX_MODE_BLURZ;

    explicit BlurzEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        um_data_t *um_data = getAudioData();
        uint8_t *fftResult = (uint8_t*)um_data->u_data[2];

        if (parameters.call == 0) {
            buffer.fill(BLACK);
            aux0 = 0;
        }

        int fadeoutDelay = (256 - parameters.speed) / 32;
        if ((fadeoutDelay <= 1 ) || ((parameters.call % fadeoutDelay) == 0)) buffer.fade(SEGCOLOR(1), parameters.speed);

        step += FRAMETIME;
        if (step > SPEED_FORMULA_L) {
            unsigned segLoc = hw_random16(coordinate.width);
            buffer.setPixelColor(segLoc, color_blend(SEGCOLOR(1), parameters.color_from_palette(2*fftResult[aux0%16]*240/max(1, (int)coordinate.width-1), false, PALETTE_SOLID_WRAP, 0), uint8_t(2*fftResult[aux0%16])));
            ++(aux0) %= 16; // make sure it doesn't cross 16

            step = 1;
            buffer.blur(parameters.intensity); // note: blur > 210 results in a alternating pattern, this could be fixed by mapping but some may like it (very old bug)
        }
        return true;
    }

private:
    uint32_t step{};
    uint16_t aux0{};
};


