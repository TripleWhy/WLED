#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

#define SPOT_TYPE_SOLID       0
#define SPOT_TYPE_GRADIENT    1
#define SPOT_TYPE_2X_GRADIENT 2
#define SPOT_TYPE_2X_DOT      3
#define SPOT_TYPE_3X_DOT      4
#define SPOT_TYPE_4X_DOT      5
#define SPOT_TYPES_COUNT      6
#ifdef ESP8266
  #define SPOT_MAX_COUNT 17          //Number of simultaneous waves
#else
  #define SPOT_MAX_COUNT 49          //Number of simultaneous waves
#endif

/*
 * Spotlights moving back and forth that cast dancing shadows.
 * Shine this through tree branches/leaves or other close-up objects that cast
 * interesting shadows onto a ceiling or tarp.
 *
 * By Steve Pomeroy @xxv
 */
class DancingShadowsEffect : public BaseEffect<DancingShadowsEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    //13 bytes
    struct Spotlight {
        float speed{};
        uint8_t colorIdx{};
        int16_t position{};
        unsigned long lastUpdateTime{};
        uint8_t width{};
        uint8_t type{};
    };

private:
    using Self = DancingShadowsEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "Dancing Shadows@!,# of shadows;!;!";
    static constexpr const uint8_t effectId = FX_MODE_DANCING_SHADOWS;

    explicit DancingShadowsEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        unsigned numSpotlights = map(SEGMENT.intensity, 0, 255, 2, SPOT_MAX_COUNT);  // 49 on 32 segment ESP32, 17 on 16 segment ESP8266
        bool initialize = previousSpotlightCount != numSpotlights;
        previousSpotlightCount = numSpotlights;

        if (!resizeVector(spotlights, numSpotlights)) {
            return false;
        }

        buffer.fill(BLACK);

        unsigned long time = strip.now;
        bool respawn = false;

        for (size_t i = 0; i < numSpotlights; i++) {
            if (!initialize) {
                // advance the position of the spotlight
                int delta = (float)(time - spotlights[i].lastUpdateTime) *
                                        (spotlights[i].speed * ((1.0 + SEGMENT.speed)/100.0));

                if (abs(delta) >= 1) {
                    spotlights[i].position += delta;
                    spotlights[i].lastUpdateTime = time;
                }

                respawn = (spotlights[i].speed > 0.0 && spotlights[i].position > (int)(coordinate.width + 2))
                             || (spotlights[i].speed < 0.0 && spotlights[i].position < -(spotlights[i].width + 2));
            }

            if (initialize || respawn) {
                spotlights[i].colorIdx = hw_random8();
                spotlights[i].width = hw_random8(1, 10);

                spotlights[i].speed = 1.0/hw_random8(4, 50);

                if (initialize) {
                    spotlights[i].position = hw_random16(coordinate.width);
                    spotlights[i].speed *= hw_random8(2) ? 1.0 : -1.0;
                } else {
                    if (hw_random8(2)) {
                        spotlights[i].position = coordinate.width + spotlights[i].width;
                        spotlights[i].speed *= -1.0;
                    }else {
                        spotlights[i].position = -spotlights[i].width;
                    }
                }

                spotlights[i].lastUpdateTime = time;
                spotlights[i].type = hw_random8(SPOT_TYPES_COUNT);
            }

            uint32_t color = SEGMENT.color_from_palette(spotlights[i].colorIdx, false, false, 255);
            int start = spotlights[i].position;

            if (spotlights[i].width <= 1) {
                if (start >= 0 && start < (int)coordinate.width) {
                    buffer.blendPixelColor(start, color, 128);
                }
            } else {
                switch (spotlights[i].type) {
                    case SPOT_TYPE_SOLID:
                        for (size_t j = 0; j < spotlights[i].width; j++) {
                            if ((start + j) >= 0 && (start + j) < coordinate.width) {
                                buffer.blendPixelColor(start + j, color, 128);
                            }
                        }
                    break;

                    case SPOT_TYPE_GRADIENT:
                        for (size_t j = 0; j < spotlights[i].width; j++) {
                            if ((start + j) >= 0 && (start + j) < coordinate.width) {
                                buffer.blendPixelColor(start + j, color, cubicwave8(map(j, 0, spotlights[i].width - 1, 0, 255)));
                            }
                        }
                    break;

                    case SPOT_TYPE_2X_GRADIENT:
                        for (size_t j = 0; j < spotlights[i].width; j++) {
                            if ((start + j) >= 0 && (start + j) < coordinate.width) {
                                buffer.blendPixelColor(start + j, color, cubicwave8(2 * map(j, 0, spotlights[i].width - 1, 0, 255)));
                            }
                        }
                    break;

                    case SPOT_TYPE_2X_DOT:
                        for (size_t j = 0; j < spotlights[i].width; j += 2) {
                            if ((start + j) >= 0 && (start + j) < coordinate.width) {
                                buffer.blendPixelColor(start + j, color, 128);
                            }
                        }
                    break;

                    case SPOT_TYPE_3X_DOT:
                        for (size_t j = 0; j < spotlights[i].width; j += 3) {
                            if ((start + j) >= 0 && (start + j) < coordinate.width) {
                                buffer.blendPixelColor(start + j, color, 128);
                            }
                        }
                    break;

                    case SPOT_TYPE_4X_DOT:
                        for (size_t j = 0; j < spotlights[i].width; j += 4) {
                            if ((start + j) >= 0 && (start + j) < coordinate.width) {
                                buffer.blendPixelColor(start + j, color, 128);
                            }
                        }
                    break;
                }
            }
        }

        return true;
    }

private:
    SegmentAllocator<Spotlight>::vector spotlights{};
    uint16_t previousSpotlightCount{};
};

