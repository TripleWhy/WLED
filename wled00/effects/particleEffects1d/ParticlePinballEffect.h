#pragma once
#ifndef WLED_DISABLE_PARTICLESYSTEM1D

#include "../../FX.h"
#include "../Effect.h"
#include "Particle1dEffect.h"

/*
  Particle Replacement for "Bbouncing Balls by Aircoookie"
  Also replaces rolling balls and juggle (and maybe popcorn)
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
class ParticlePinballEffect : public BaseEffect<ParticlePinballEffect, Particle1dEffect<ParticlePinballEffect>> {
private:
    using Self = ParticlePinballEffect;
    using Base = BaseEffect<Self, Particle1dEffect<Self>>;

public:
    static constexpr const char metaData[] PROGMEM = "PS Pinball@Speed,!,Size,Blur,Gravity,Collide,Rolling,Position Color;,!;!;1;pal=0,ix=220,c2=0,c3=8,o1=1";
    static constexpr const uint8_t effectId = FX_MODE_PSPINBALL;

    using Base::Base;

    bool init(const EffectCoordinate& coordinate) {
        if (!Base::init(coordinate, 1, 128, true)) {
            return false;
        }

        PartSys.sources[0].sourceFlags.collide = true; // seeded particles will collide (if enabled)
        PartSys.sources[0].source.x = PS_P_RADIUS_1D; //emit at bottom
        PartSys.setKillOutOfBounds(true); // out of bounds particles dont return
        PartSys.setUsedParticles(255); // use all available particles for init
        return true;
    }

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        // Particle System settings
        //uint32_t hardness = 240 + (parameters.custom1>>4);
        PartSys.updateSystem(coordinate.width); // update system properties (dimensions and data pointers)
        PartSys.setGravity(map(parameters.custom3, 0 , 31, 0 , 16)); // set gravity (8 is default strength)
        PartSys.setBounce(parameters.custom3); // disables bounce if no gravity is used
        PartSys.setMotionBlur(parameters.custom2); // anable motion blur
        PartSys.enableParticleCollisions(parameters.check1, 255); // enable collisions and set particle collision to high hardness
        PartSys.setUsedParticles(parameters.intensity);
        PartSys.setColorByPosition(parameters.check3);

        bool updateballs = false;
        if (aux1 != parameters.speed + parameters.intensity + parameters.check2 + parameters.custom1 + PartSys.usedParticles) { // user settings change or more particles are available
            step = parameters.call; // reset delay
            updateballs = true;
            PartSys.sources[0].maxLife = parameters.custom3 ? 5000 : 0xFFFF; // maximum lifetime in frames/2 (very long if not using gravity, this is enough to travel 4000 pixels at min speed)
            PartSys.sources[0].minLife = PartSys.sources[0].maxLife >> 1;
        }

        if (parameters.check2) { //rolling balls
            PartSys.setGravity(0);
            PartSys.setWallHardness(255);
            int speedsum = 0;
            for (uint32_t i = 0; i < PartSys.usedParticles; i++) {
                    PartSys.particles[i].ttl = 260; // keep particles alive
                if (updateballs) { //speed changed or particle is dead, set particle properties
                    PartSys.particleFlags[i].collide = true;
                    if (PartSys.particles[i].x == 0) { // still at initial position (when not switching from a PS)
                        PartSys.particles[i].x = hw_random16(PartSys.maxX); // random initial position for all particles
                        PartSys.particles[i].vx = (hw_random16() & 0x01) ? 1 : -1; // random initial direction
                    }
                    PartSys.particles[i].hue = hw_random8(); //set ball colors to random
                    PartSys.advPartProps[i].sat = 255;
                    PartSys.advPartProps[i].size = parameters.custom1;
                }
                speedsum += abs(PartSys.particles[i].vx);
            }
            int32_t avgSpeed = speedsum / PartSys.usedParticles;
            int32_t setSpeed = 2 + (parameters.speed >> 3);
            if (avgSpeed < setSpeed) { // if balls are slow, speed up some of them at random to keep the animation going
                for (int i = 0; i < setSpeed - avgSpeed; i++) {
                    int idx = hw_random16(PartSys.usedParticles);
                    PartSys.particles[idx].vx += PartSys.particles[idx].vx >= 0 ? 1 : -1; // add 1, keep direction
                }
            }
            else if (avgSpeed > setSpeed + 8) // if avg speed is too high, apply friction to slow them down
                PartSys.applyFriction(1);
        }
        else { //bouncing balls
            PartSys.setWallHardness(220);
            PartSys.sources[0].var = parameters.speed >> 3;
            int32_t newspeed = 2 + (parameters.speed >> 1) - (parameters.speed >> 3);
            PartSys.sources[0].v = newspeed;
            //check for balls that are 'laying on the ground' and remove them
            for (uint32_t i = 0; i < PartSys.usedParticles; i++) {
                if (PartSys.particles[i].vx == 0 && PartSys.particles[i].x < (PS_P_RADIUS_1D + parameters.custom1))
                    PartSys.particles[i].ttl = 0;
                if (updateballs) {
                    PartSys.advPartProps[i].size = parameters.custom1;
                    if (parameters.custom3 == 0) //gravity off, update speed
                        PartSys.particles[i].vx = PartSys.particles[i].vx > 0 ? newspeed : -newspeed; //keep the direction
                }
            }

            // every nth frame emit a ball
            if (parameters.call > step) {
                int interval = 260 - ((int)parameters.intensity);
                step += interval + hw_random16(interval);
                PartSys.sources[0].source.hue = hw_random16(); //set ball color
                PartSys.sources[0].sat = 255;
                PartSys.sources[0].size = parameters.custom1;
                PartSys.sprayEmit(PartSys.sources[0]);
            }
        }
        aux1 = parameters.speed + parameters.intensity + parameters.check2 + parameters.custom1 + PartSys.usedParticles;
        for (uint32_t i = 0; i < PartSys.usedParticles; i++) {
            PartSys.particleMoveUpdate(PartSys.particles[i], PartSys.particleFlags[i]); // double the speed
        }

        PartSys.update(buffer, parameters); // update and render
        return true;
    }

private:
    uint32_t step{};
    uint16_t aux0{1};
    uint16_t aux1{5000}; //set out of range to ensure uptate on first call
};


#endif //WLED_DISABLE_PARTICLESYSTEM1D
