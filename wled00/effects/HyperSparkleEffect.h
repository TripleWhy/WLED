#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Like flash sparkle. With more flash.
 * Inspired by www.tweaking4all.com/hardware/arduino/adruino-led-strip-effects/
 */
class HyperSparkleEffect : public BaseEffect<HyperSparkleEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = HyperSparkleEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Sparkle+@!,!,,,,Move,Overlay;Bg,Fx;!;;m12=0";
    static constexpr const uint8_t effectId = FX_MODE_HYPER_SPARKLE;

    explicit HyperSparkleEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        uint32_t cycleTime = 10 + (255 - parameters.speed)*2;
        uint32_t it = strip.now / cycleTime;
        const bool moving = parameters.check1;
        if (!parameters.check2) {
            for (unsigned i = 0; i < coordinate.width; i++) {
                unsigned palIdx = moving ? (i+it)%coordinate.width : i;
                buffer.setPixelColor(i, parameters.color_from_palette(palIdx, true, moving, 0));
            }
        }

        if (strip.now > flashTimestamp + flashPauseDuration) {
          if (hw_random8((255-parameters.intensity) >> 4) == 0) {
            int len = max(1, (int)coordinate.width/3);
            for (int i = 0; i < len; i++) {
                buffer.setPixelColor(hw_random16(coordinate.width), SEGCOLOR(1));
            }
          }
          flashTimestamp = strip.now;
          flashPauseDuration = 255-parameters.speed;
        }
        return true;
    }

private:
    uint32_t flashTimestamp{};
    uint16_t flashPauseDuration{};
};
