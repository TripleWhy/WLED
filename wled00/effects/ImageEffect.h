#pragma once
#ifdef WLED_ENABLE_GIF

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
  Image effect
  Draws a .gif image from filesystem on the matrix/strip
*/
class ImageEffect : public BaseEffect<ImageEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = ImageEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Image@!,;;;12;sx=128";
    static constexpr const uint8_t effectId = FX_MODE_IMAGE;

    explicit ImageEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        //TODO
        renderImageToSegment(SEGMENT);
        // if (status != 0 && status != 254 && status != 255) {
        //   Serial.print("GIF renderer return: ");
        //   Serial.println(status);
        // }

    }

private:
};

#endif // WLED_ENABLE_GIF