#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Spotlights moving back and forth that cast dancing shadows.
 * Shine this through tree branches/leaves or other close-up objects that cast
 * interesting shadows onto a ceiling or tarp.
 *
 * By Steve Pomeroy @xxv
 */
class DancingShadowsEffect : public BaseEffect<DancingShadowsEffect, BufferedEffect<EffectDimensionality::d1>> {
public:
    enum class SpotType : uint8_t {
        SOLID =       0,
        GRADIENT =    1,
        GRADIENT_X2 = 2,
        DOT_X2 =      3,
        DOT_X3 =      4,
        DOT_X4 =      5,
        COUNT
    };

private:
    #ifdef ESP8266
    static constexpr long SPOT_MAX_COUNT = 17;          //Number of simultaneous waves
    #else
    static constexpr long SPOT_MAX_COUNT = 49;          //Number of simultaneous waves
    #endif

    //13 bytes
    struct Spotlight {
        float speed{};
        uint8_t colorIdx{};
        int16_t position{};
        unsigned long lastUpdateTime{};
        uint8_t width{};
        SpotType type{};
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

        if (!spotlights.resize(numSpotlights)) {
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
                spotlights[i].type = static_cast<SpotType>(hw_random8(static_cast<uint8_t>(SpotType::COUNT)));
            }

            uint32_t color = SEGMENT.color_from_palette(spotlights[i].colorIdx, false, false, 255);
            int start = spotlights[i].position;

            if (spotlights[i].width <= 1) {
                if (start >= 0 && start < (int)coordinate.width) {
                    buffer.blendPixelColor(start, color, 128);
                }
            } else {
                switch (spotlights[i].type) {
                    case SpotType::SOLID:
                        for (size_t j = 0; j < spotlights[i].width; j++) {
                            if ((start + j) >= 0 && (start + j) < coordinate.width) {
                                buffer.blendPixelColor(start + j, color, 128);
                            }
                        }
                    break;

                    case SpotType::GRADIENT:
                        for (size_t j = 0; j < spotlights[i].width; j++) {
                            if ((start + j) >= 0 && (start + j) < coordinate.width) {
                                buffer.blendPixelColor(start + j, color, cubicwave8(map(j, 0, spotlights[i].width - 1, 0, 255)));
                            }
                        }
                    break;

                    case SpotType::GRADIENT_X2:
                        for (size_t j = 0; j < spotlights[i].width; j++) {
                            if ((start + j) >= 0 && (start + j) < coordinate.width) {
                                buffer.blendPixelColor(start + j, color, cubicwave8(2 * map(j, 0, spotlights[i].width - 1, 0, 255)));
                            }
                        }
                    break;

                    case SpotType::DOT_X2:
                        for (size_t j = 0; j < spotlights[i].width; j += 2) {
                            if ((start + j) >= 0 && (start + j) < coordinate.width) {
                                buffer.blendPixelColor(start + j, color, 128);
                            }
                        }
                    break;

                    case SpotType::DOT_X3:
                        for (size_t j = 0; j < spotlights[i].width; j += 3) {
                            if ((start + j) >= 0 && (start + j) < coordinate.width) {
                                buffer.blendPixelColor(start + j, color, 128);
                            }
                        }
                    break;

                    case SpotType::DOT_X4:
                        for (size_t j = 0; j < spotlights[i].width; j += 4) {
                            if ((start + j) >= 0 && (start + j) < coordinate.width) {
                                buffer.blendPixelColor(start + j, color, 128);
                            }
                        }
                    break;

                    case SpotType::COUNT:
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

