#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

class HalloweenEyesEffect : public BaseEffect<HalloweenEyesEffect> {
private:
    enum eyeState : uint8_t {
        initializeOn = 0,
        on,
        blink,
        initializeOff,
        off,

        count
    };

private:
    using Self = HalloweenEyesEffect;
    using Base = BaseEffect<Self>;

public:
    static constexpr const char metaData[] PROGMEM = "Halloween Eyes@Eye off time,Eye on time,,,,,Overlay;!,!;!;12";
    static constexpr const uint8_t effectId = FX_MODE_HALLOWEEN_EYES;
    static constexpr const EffectDimensionality dimensionality = EffectDimensionality::d2VStrips;

    using Base::Base;

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        const unsigned maxWidth = strip.isMatrix ? coordinate.width : coordinate.width;
        const unsigned HALLOWEEN_EYE_SPACE = MAX(2, strip.isMatrix ? coordinate.width>>4: coordinate.width>>5);
        unsigned HALLOWEEN_EYE_WIDTH = HALLOWEEN_EYE_SPACE/2;
        unsigned eyeLength = (2*HALLOWEEN_EYE_WIDTH) + HALLOWEEN_EYE_SPACE;

        state = static_cast<eyeState>(state % eyeState::count);
        unsigned duration = max(uint16_t{1u}, storedDuration);
        const uint32_t elapsedTime = strip.now - startTime;

        switch (state) {
            case eyeState::initializeOn: {
                // initialize the eyes-on state:
                // - select eye position and color
                // - select a duration
                // - immediately switch to eyes on state.

                startPos = hw_random16(0, maxWidth - eyeLength - 1);
                color = hw_random8();
                if (strip.isMatrix)
                    eyeY = hw_random16(coordinate.height-1);
                duration = 128u + hw_random16(parameters.intensity*64u);
                storedDuration = duration;
                state = eyeState::on;
                [[fallthrough]];
            }
            case eyeState::on: {
                // eyes-on steate:
                // - fade eyes in for some time
                // - keep eyes on until the pre-selected duration is over
                // - randomly switch to the blink (sub-)state, and initialize it with a blink duration (more precisely, a blink end time stamp)
                // - never switch to the blink state if the animation just started or is about to end

                start2ndEye = startPos + HALLOWEEN_EYE_WIDTH + HALLOWEEN_EYE_SPACE;
                // If the user reduces the input while in this state, limit the duration.
                duration = min(duration, (128u + (parameters.intensity * 64u)));

                constexpr uint32_t minimumOnTimeBegin = 1024u;
                constexpr uint32_t minimumOnTimeEnd = 1024u;
                const uint32_t fadeInAnimationState = elapsedTime * uint32_t{256u * 8u} / duration;
                const uint32_t backgroundColor = SEGCOLOR(1);
                eyeColor = parameters.color_from_palette(color, false, false, 0);
                if (fadeInAnimationState < 256u) {
                    eyeColor = color_blend(backgroundColor, eyeColor, uint8_t(fadeInAnimationState));
                } else if (elapsedTime > minimumOnTimeBegin) {
                    const uint32_t remainingTime = (elapsedTime >= duration) ? 0u : (duration - elapsedTime);
                    if (remainingTime > minimumOnTimeEnd) {
                        if (hw_random8() < 4u)
                        {
                            eyeColor = backgroundColor;
                            state = eyeState::blink;
                            blinkEndTime = strip.now + hw_random8(8, 128);
                        }
                    }
                }
                break;
            }
            case eyeState::blink: {
                // eyes-on but currently blinking state:
                // - wait until the blink time is over, then switch back to eyes-on

                if (strip.now >= blinkEndTime) {
                    state = eyeState::on;
                }
                break;
            }
            case eyeState::initializeOff: {
                // initialize eyes-off state:
                // - select a duration
                // - immediately switch to eyes-off state

                const unsigned eyeOffTimeBase = parameters.speed*128u;
                duration = eyeOffTimeBase + hw_random16(eyeOffTimeBase);
                storedDuration = duration;
                state = eyeState::off;
                [[fallthrough]];
            }
            case eyeState::off: {
                // eyes-off state:
                // - not much to do here

                // If the user reduces the input while in this state, limit the duration.
                const unsigned eyeOffTimeBase = parameters.speed*128u;
                duration = min(duration, (2u * eyeOffTimeBase));
                break;
            }
            case eyeState::count: {
                // Can't happen, not an actual state.
                state = eyeState::initializeOn;
                break;
            }
        }

        if (elapsedTime > duration) {
            // The current state duration is over, switch to the next state.
            switch (state) {
                case eyeState::initializeOn:
                case eyeState::on:
                case eyeState::blink:
                    state = eyeState::initializeOff;
                    break;
                case eyeState::initializeOff:
                case eyeState::off:
                case eyeState::count:
                default:
                    state = eyeState::initializeOn;
                    break;
            }
            startTime = strip.now;
        }
        return true;
    }

    uint32_t getPixelColorImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        if (
            (state == eyeState::on)
            && (coordinate.getYAbsolute() == eyeY)
            && (
                ((startPos <= coordinate.getXAbsolute()) && (coordinate.getXAbsolute() < startPos + HALLOWEEN_EYE_WIDTH))
                || ((start2ndEye <= coordinate.getXAbsolute()) && (coordinate.getXAbsolute() < start2ndEye + HALLOWEEN_EYE_WIDTH))
            )
        ) {
            return eyeColor;
        }

        // background
        if (!parameters.check2)
            return SEGCOLOR(1);
        return currentColor.getColor();
    }

private:
    eyeState state{};
    uint8_t color{};
    uint16_t startPos{};
    // duration + endTime could theoretically be replaced by a single endTime, however we would lose
    // the ability to end the animation early when the user reduces the animation time.
    uint16_t storedDuration{};
    uint32_t startTime{};
    uint32_t blinkEndTime{};

    unsigned HALLOWEEN_EYE_WIDTH{};
    uint32_t eyeColor{};
    unsigned eyeY{};
    unsigned start2ndEye{};
};


#endif //WLED_DISABLE_2D
