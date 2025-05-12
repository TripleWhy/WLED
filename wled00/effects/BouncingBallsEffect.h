#pragma once

#include <algorithm>
#include "../FX.h"
#include "Effect.h"

class BouncingBallsEffect : public BaseEffect<BouncingBallsEffect> {
    struct Ball {
        unsigned long lastBounceTime{strip.now};
        float impactVelocity{};
        float height{};
    };

private:
    using Self = BouncingBallsEffect;
    using Base = BaseEffect<Self>;
    static constexpr unsigned maxNumBalls = 16;

public:
    static constexpr const char metaData[] PROGMEM = "Bouncing Balls@Gravity,balls per line,,,lines,,Overlay;!,!,!;!;1;m12=1";
    static constexpr const uint8_t effectId = FX_MODE_BOUNCINGBALLS;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d2VStrips;

    using Base::Base;

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        numBalls = (parameters.intensity * (maxNumBalls - 1)) / 255 + 1; // minimum 1 ball
        strips = std::clamp(static_cast<unsigned>(parameters.custom3), 1u, coordinate.height);
        useBackgroundColor = !parameters.check2;

        if (useBackgroundColor)
            backgroundColor = (SEGCOLOR(2) ? BLACK : SEGCOLOR(1));
        ballSize = std::max(1u, coordinate.height / strips);

        for (size_t i = 0; i < numBalls; ++i) {
            if (SEGMENT.palette) {
                ballColors[i] = parameters.color_wheel(i*(256/MAX(numBalls, 8)));
            } else if (SEGCOLOR(2)) {
                ballColors[i] = SEGCOLOR(i % NUM_COLORS);
            } else {
                ballColors[i] = SEGCOLOR(0);
            }
        }

        if (!balls.resize(maxNumBalls * strips)) { //TODO reduce to actual ball count instead of max ball count?
            return false;
        }

        for (unsigned stripNr = 0; stripNr < strips; ++stripNr)
            runVirtualStrip(parameters, stripNr, coordinate.width, &balls[stripNr * maxNumBalls]);

        return true;
    }

    void nextRowImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        stripIndex = (coordinate.getYAbsolute() * strips) / coordinate.height;
    }

    uint32_t getPixelColorImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        for (size_t ballIndex = 0; ballIndex < numBalls; ballIndex++) {
            const Ball& ball = balls[stripIndex * maxNumBalls + ballIndex];
            const float pixelHeight = ball.height * (coordinate.width - 1);
            if (pixelHeight - (ballSize / 2) <= coordinate.getXAbsolute() && coordinate.getXAbsolute() < pixelHeight + ((ballSize + 1) / 2))
                return ballColors[ballIndex];
        }

        if (useBackgroundColor)
            return backgroundColor;
        else
            return currentColor.getColor();
    }

private:
    // virtualStrip idea by @ewowi (Ewoud Wijma)
    // requires virtual strip # to be embedded into upper 16 bits of index in setPixelColor()
    // the following functions will not work on virtual strips: fill(), fade_out(), fadeToBlack(), blur()
    void runVirtualStrip(TransitionableParameters& parameters, size_t stripNr, unsigned width, Ball* balls) {
        constexpr float gravity = -9.81f; // standard value of gravity
        constexpr float initialVelocityFactor = 4.4294469f; // sqrtf(-2.0f * gravity);
        // number of balls based on intensity setting to max of 7 (cycles colors)
        // non-chosen color is a random color
        const unsigned long time = strip.now;

        const float bounceTimeFactor = 0.001f / ((255-parameters.speed)/64 +1);
        for (size_t i = 0; i < numBalls; i++) {
            // time since last bounce in seconds
            const float timeSec = (time - balls[i].lastBounceTime) * bounceTimeFactor;
            balls[i].height = ((0.5f * gravity) * timeSec + balls[i].impactVelocity) * timeSec; // avoid use pow(x, 2) - its extremely slow !

            if (balls[i].height <= 0.0f) {
                balls[i].height = 0.0f;
                //damping for better effect using multiple balls
                const float dampening = 0.9f - float(i)/float(numBalls * numBalls); // avoid use pow(x, 2) - its extremely slow !
                balls[i].impactVelocity = dampening * balls[i].impactVelocity;
                balls[i].lastBounceTime = time;

                if (balls[i].impactVelocity < 0.015f) {
                    float impactVelocityStart = initialVelocityFactor * hw_random8(5,11)/10.0f; // randomize impact velocity
                    balls[i].impactVelocity = impactVelocityStart;
                }
            } else if (balls[i].height > 1.0f) {
                continue; // do not draw OOB ball
            }
        }
    }

private:
    unsigned numBalls;
    unsigned strips;
    std::array<uint32_t, maxNumBalls> ballColors;
    SegmentAllocator<Ball>::vector balls;
    uint32_t backgroundColor;
    bool useBackgroundColor;
    unsigned stripIndex;
    unsigned ballSize;
};
