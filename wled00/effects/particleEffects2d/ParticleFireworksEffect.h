#pragma once
#ifndef WLED_DISABLE_2D
#ifndef WLED_DISABLE_PARTICLESYSTEM2D

#include "../../FX.h"
#include "../Effect.h"
#include "Particle2dEffect.h"

/*
  Particle Fireworks
  Rockets shoot up and explode in a random color, sometimes in a defined pattern
  by DedeHai (Damian Schneider)
*/
class ParticleFireworksEffect : public BaseEffect<ParticleFireworksEffect, Particle2dEffect<ParticleFireworksEffect>> {
private:
    using Self = ParticleFireworksEffect;
    using Base = BaseEffect<Self, Particle2dEffect<Self>>;

    static constexpr uint32_t NUMBEROFSOURCES = 8;

public:
    static constexpr const char metaData[] PROGMEM = "PS Fireworks@Launches,Explosion Size,Fuse,Blur,Gravity,Cylinder,Ground,Fast;;!;2;pal=11,ix=50,c1=40,c2=0,c3=12";
    static constexpr const uint8_t effectId = FX_MODE_PARTICLEFIREWORKS;

    using Base::Base;

    bool init(const EffectCoordinate& coordinate) {
        if (!Base::init(coordinate, NUMBEROFSOURCES, false, false)) {
            return false;
        }

        PartSys.setKillOutOfBounds(true); // out of bounds particles dont return (except on top, taken care of by gravity setting)
        PartSys.setWallHardness(120); // ground bounce is fixed
        const uint32_t numRockets = min(PartSys.sources.size(), (uint32_t)NUMBEROFSOURCES);
        for (uint32_t j = 0; j < numRockets; j++) {
            PartSys.sources[j].source.ttl = 500 * j; // first rocket starts immediately, others follow soon
            PartSys.sources[j].source.vy = -1; // at negative speed, no particles are emitted and if rocket dies, it will be relaunched
        }
        return true;
    }

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        uint32_t numRockets;

        PartSys.updateSystem(coordinate.width, coordinate.height); // update system properties (dimensions and data pointers)
        numRockets = map(SEGMENT.speed, 0 , 255, 4, min(PartSys.sources.size(), (uint32_t)NUMBEROFSOURCES));

        PartSys.setWrapX(SEGMENT.check1);
        PartSys.setBounceY(SEGMENT.check2);
        PartSys.setGravity(map(SEGMENT.custom3, 0, 31, SEGMENT.check2 ? 1 : 0, 10)); // if bounded, set gravity to minimum of 1 or they will bounce at top
        PartSys.setMotionBlur(map(SEGMENT.custom2, 0, 255, 0, 245)); // anable motion blur

        // update the rockets, set the speed state
        for (uint32_t j = 0; j < numRockets; j++) {
                PartSys.applyGravity(PartSys.sources[j].source);
                PartSys.particleMoveUpdate(PartSys.sources[j].source, PartSys.sources[j].sourceFlags);
                if (PartSys.sources[j].source.ttl == 0) {
                    if (PartSys.sources[j].source.vy > 0) { // rocket has died and is moving up. stop it so it will explode (is handled in the code below)
                        PartSys.sources[j].source.vy = 0;
                    }
                    else if (PartSys.sources[j].source.vy < 0) { // rocket is exploded and time is up (ttl=0 and negative speed), relaunch it
                        PartSys.sources[j].source.y = PS_P_RADIUS; // start from bottom
                        PartSys.sources[j].source.x = (PartSys.maxX >> 2) + hw_random(PartSys.maxX >> 1); // centered half
                        PartSys.sources[j].source.vy = (SEGMENT.custom3) + hw_random16(SEGMENT.custom1 >> 3) + 5; // rocket speed TODO: need to adjust for segment height
                        PartSys.sources[j].source.vx = hw_random16(7) - 3; // not perfectly straight up
                        PartSys.sources[j].source.sat = 30; // low saturation -> exhaust is off-white
                        PartSys.sources[j].source.ttl = hw_random16(SEGMENT.custom1) + (SEGMENT.custom1 >> 1); // set fuse time
                        PartSys.sources[j].maxLife = 40; // exhaust particle life
                        PartSys.sources[j].minLife = 10;
                        PartSys.sources[j].vx = 0;  // emitting speed
                        PartSys.sources[j].vy = -5;  // emitting speed
                        PartSys.sources[j].var = 4; // speed variation around vx,vy (+/- var)
                    }
             }
        }
        // check each rocket's state and emit particles according to its state: moving up = emit exhaust, at top = explode; falling down = standby time
        uint32_t emitparticles, frequency, baseangle, hueincrement; // number of particles to emit for each rocket's state
        // variables for circular explosions
        int32_t speed{}, currentspeed, percircle;
        int32_t counter = 0;
        uint16_t angle{};
        unsigned angleincrement;
        bool circularexplosion = false;

        // emit particles for each rocket
        for (uint32_t j = 0; j < numRockets; j++) {
            // determine rocket state by its speed:
            if (PartSys.sources[j].source.vy > 0) { // moving up, emit exhaust
                emitparticles = 1;
            }
            else if (PartSys.sources[j].source.vy < 0) { // falling down, standby time
                emitparticles = 0;
            }
            else { // speed is zero, explode!
                PartSys.sources[j].source.hue = hw_random16(); // random color
                PartSys.sources[j].source.sat = hw_random16(55) + 200;
                PartSys.sources[j].maxLife = 200;
                PartSys.sources[j].minLife = 100;
                PartSys.sources[j].source.ttl = hw_random16((2000 - ((uint32_t)SEGMENT.speed << 2))) + 550 - (SEGMENT.speed << 1); // standby time til next launch
                PartSys.sources[j].var = ((SEGMENT.intensity >> 4) + 5); // speed variation around vx,vy (+/- var)
                PartSys.sources[j].source.vy = -1; // set speed negative so it will emit no more particles after this explosion until relaunch
                #ifdef ESP8266
                emitparticles = hw_random16(SEGMENT.intensity >> 3) + (SEGMENT.intensity >> 3) + 5; // defines the size of the explosion
                #else
                emitparticles = hw_random16(SEGMENT.intensity >> 2) + (SEGMENT.intensity >> 2) + 5; // defines the size of the explosion
                #endif

                if (hw_random() & 1) { // 50% chance for circular explosion
                    circularexplosion = true;
                    speed = 2 + hw_random16(3) + ((SEGMENT.intensity >> 6));
                    currentspeed = speed;
                    angleincrement = 2730 + hw_random16(5461); // minimum 15° + random(30°)
                    angle = hw_random16(); // random start angle
                    baseangle = angle; // save base angle for modulation
                    percircle = 0xFFFF / angleincrement + 1; // number of particles to make complete circles
                    hueincrement = hw_random16() & 127; // &127 is equivalent to %128
                    int circles = 1 + hw_random16(3) + ((SEGMENT.intensity >> 6));
                    frequency = hw_random16() & 127; // modulation frequency (= "waves per circle"), x.4 fixed point
                    emitparticles = percircle * circles;
                    PartSys.sources[j].var = angle & 1; // 0 or 1 variation, angle is random
                }
            }
            uint32_t i;
            for (i = 0; i < emitparticles; i++) {
                if (circularexplosion) {
                    int32_t sineMod = 0xEFFF + sin16_t((uint16_t)(((angle * frequency) >> 4) + baseangle)); // shifted to positive values
                    currentspeed = (speed/2 + ((sineMod * speed) >> 16)) >> 1; // sine modulation on speed based on emit angle
                    PartSys.angleEmit(PartSys.sources[j], angle, currentspeed); // note: compiler warnings can be ignored, variables are set just above
                    counter++;
                    if (counter > percircle) { // full circle completed, increase speed
                        counter = 0;
                        speed += 3 + ((SEGMENT.intensity >> 6)); // increase speed to form a second wave
                        PartSys.sources[j].source.hue += hueincrement; // new color for next circle
                        PartSys.sources[j].source.sat = 100 + hw_random16(156);
                    }
                    angle += angleincrement; // set angle for next particle
                }
                else { // random explosion or exhaust
                    PartSys.sprayEmit(PartSys.sources[j]);
                    if ((j % 3) == 0) {
                        PartSys.sources[j].source.hue = hw_random16(); // random color for each particle (this is also true for exhaust, but that is white anyways)
                    }
                }
            }
            if (i == 0) // no particles emitted, this rocket is falling
                PartSys.sources[j].source.y = 1000; // reset position so gravity wont pull it to the ground and bounce it (vy MUST stay negative until relaunch)
            circularexplosion = false; // reset for next rocket
        }
        if (SEGMENT.check3) { // fast speed, move particles twice
            for (uint32_t i = 0; i < PartSys.usedParticles; i++) {
                PartSys.particleMoveUpdate(PartSys.particles[i], PartSys.particleFlags[i], nullptr, nullptr);
            }
        }
        PartSys.update(buffer); // update and render
        return true;
    }

private:
};


#endif //WLED_DISABLE_2D
#endif //WLED_DISABLE_PARTICLESYSTEM2D
