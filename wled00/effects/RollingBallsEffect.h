#pragma once

#include <array>
#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 *  bouncing balls on a track track Effect modified from Aircoookie's bouncing balls
 *  Courtesy of pjhatch (https://github.com/pjhatch)
 *  https://github.com/wled-dev/WLED/pull/1039
 */
// modified for balltrack mode
class RollingBallsEffect : public BaseEffect<RollingBallsEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    struct RollingBall {
        unsigned long lastBounceUpdate;
        float mass; // could fix this to be = 1. if memory is an issue
        float velocity;
        float height;
    };

private:
    using Self = RollingBallsEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;
    static constexpr unsigned maxNumBalls = 16; // 255/16 + 1

public:
    static constexpr const char* const metaData = "Rolling Balls@!,# of balls,,,,Collide,Overlay,Trails;!,!,!;!;1;m12=1";
    static constexpr const uint8_t effectId = FX_MODE_ROLLINGBALLS;

    explicit RollingBallsEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        // number of balls based on intensity setting to max of 16 (cycles colors)
        // non-chosen color is a random color
        unsigned numBalls = SEGMENT.intensity/16 + 1;
        bool hasCol2 = SEGCOLOR(2);

        if (SEGENV.call == 0) {
            buffer.fill(hasCol2 ? BLACK : SEGCOLOR(1));                    // start clean
            for (unsigned i = 0; i < maxNumBalls; i++) {
                balls[i].lastBounceUpdate = strip.now;
                balls[i].velocity = 20.0f * float(hw_random16(1000, 10000))/10000.0f;  // number from 1 to 10
                if (hw_random8()<128) balls[i].velocity = -balls[i].velocity;    // 50% chance of reverse direction
                balls[i].height = (float(hw_random16(0, 10000)) / 10000.0f);     // from 0. to 1.
                balls[i].mass   = (float(hw_random16(1000, 10000)) / 10000.0f);  // from .1 to 1.
            }
        }

        float cfac = float(scale8(8, 255-SEGMENT.speed) +1)*20000.0f; // this uses the Aircoookie conversion factor for scaling time using speed slider

        if (SEGMENT.check3) buffer.fadeOut(250); // 2-8 pixel trails (optional)
        else {
        	if (!SEGMENT.check2) buffer.fill(hasCol2 ? BLACK : SEGCOLOR(1)); // don't fill with background color if user wants to see trails
        }

        for (unsigned i = 0; i < numBalls; i++) {
            float timeSinceLastUpdate = float((strip.now - balls[i].lastBounceUpdate))/cfac;
            float thisHeight = balls[i].height + balls[i].velocity * timeSinceLastUpdate; // this method keeps higher resolution
            // test if intensity level was increased and some balls are way off the track then put them back
            if (thisHeight < -0.5f || thisHeight > 1.5f) {
                thisHeight = balls[i].height = (float(hw_random16(0, 10000)) / 10000.0f); // from 0. to 1.
                balls[i].lastBounceUpdate = strip.now;
            }
            // check if reached ends of the strip
            if ((thisHeight <= 0.0f && balls[i].velocity < 0.0f) || (thisHeight >= 1.0f && balls[i].velocity > 0.0f)) {
                balls[i].velocity = -balls[i].velocity; // reverse velocity
                balls[i].lastBounceUpdate = strip.now;
                balls[i].height = thisHeight;
            }
            // check for collisions
            if (SEGMENT.check1) {
                for (unsigned j = i+1; j < numBalls; j++) {
                    if (balls[j].velocity != balls[i].velocity) {
                        //  tcollided + balls[j].lastBounceUpdate is acutal time of collision (this keeps precision with long to float conversions)
                        float tcollided = (cfac*(balls[i].height - balls[j].height) +
                                    balls[i].velocity*float(balls[j].lastBounceUpdate - balls[i].lastBounceUpdate))/(balls[j].velocity - balls[i].velocity);

                        if ((tcollided > 2.0f) && (tcollided < float(strip.now - balls[j].lastBounceUpdate))) { // 2ms minimum to avoid duplicate bounces
                            balls[i].height = balls[i].height + balls[i].velocity*(tcollided + float(balls[j].lastBounceUpdate - balls[i].lastBounceUpdate))/cfac;
                            balls[j].height = balls[i].height;
                            balls[i].lastBounceUpdate = (unsigned long)(tcollided + 0.5f) + balls[j].lastBounceUpdate;
                            balls[j].lastBounceUpdate = balls[i].lastBounceUpdate;
                            float vtmp = balls[i].velocity;
                            balls[i].velocity = ((balls[i].mass - balls[j].mass)*vtmp              + 2.0f*balls[j].mass*balls[j].velocity)/(balls[i].mass + balls[j].mass);
                            balls[j].velocity = ((balls[j].mass - balls[i].mass)*balls[j].velocity + 2.0f*balls[i].mass*vtmp)             /(balls[i].mass + balls[j].mass);
                            thisHeight = balls[i].height + balls[i].velocity*(strip.now - balls[i].lastBounceUpdate)/cfac;
                        }
                    }
                }
            }

            uint32_t color = SEGCOLOR(0);
            if (SEGMENT.palette) {
                //color = SEGMENT.color_wheel(i*(256/MAX(numBalls, 8)));
                color = SEGMENT.color_from_palette(i*255/numBalls, false, PALETTE_SOLID_WRAP, 0);
            } else if (hasCol2) {
                color = SEGCOLOR(i % NUM_COLORS);
            }

            if (thisHeight < 0.0f) thisHeight = 0.0f;
            if (thisHeight > 1.0f) thisHeight = 1.0f;
            unsigned pos = round(thisHeight * (coordinate.width - 1));
            buffer.setPixelColor(pos, color);
            balls[i].lastBounceUpdate = strip.now;
            balls[i].height = thisHeight;
        }
    }

private:
    std::array<RollingBall, maxNumBalls> balls{};
};
