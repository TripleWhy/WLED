#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

//////////////////////
//   * JUGGLES      //
//////////////////////
class JugglesEffect : public BaseEffect<JugglesEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = JugglesEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Juggles@!,# of balls;!,!;!;01v;m12=0,si=0";
    static constexpr const uint8_t effectId = FX_MODE_JUGGLES;

    explicit JugglesEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }
                                         // Juggles. By Andrew Tuline.
        um_data_t *um_data = getAudioData();
        float   volumeSmth   = *(float*)  um_data->u_data[0];

        buffer.fadeOut(224); // 6.25%
        uint8_t my_sampleAgc = fmax(fmin(volumeSmth, 255.0), 0);

        for (size_t i=0; i<parameters.intensity/32+1U; i++) {
            // if coordinate.width equals 1, we will always set color to the first and only pixel, but the effect is still good looking
            buffer.setPixelColor(beatsin16_t(parameters.speed/4+i*2,0,coordinate.width-1), color_blend(SEGCOLOR(1), SEGMENT.color_from_palette(strip.now/4+i*2, false, PALETTE_SOLID_WRAP, 0), my_sampleAgc));
        }
        return true;
    }

private:
};


