#pragma once
#ifndef WLED_DISABLE_PARTICLESYSTEM1D

#include "../../FX.h"
#include "../Effect.h"
#include "Particle1dEffect.h"

/*
  Particle Fireworks 1D replacement
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
class ParticleFireworks1dEffect : public BaseEffect<ParticleFireworks1dEffect, Particle1dEffect<ParticleFireworks1dEffect>> {
private:
    using Self = ParticleFireworks1dEffect;
    using Base = BaseEffect<Self, Particle1dEffect<Self>>;

public:
    static constexpr const char metaData[] PROGMEM = "PS Fireworks 1D@Gravity,Explosion,Firing side,Blur,Saturation,,Colorful,Smooth;,!;!;1;sx=150,c2=30,c3=31,o2=1";
    static constexpr const uint8_t effectId = FX_MODE_PSFIREWORKS1D;

    using Base::Base;

    bool init(const EffectCoordinate& coordinate) {
        if (!Base::init(coordinate, 4, 150, true)) {
            return false;
        }

        PartSys.setKillOutOfBounds(true);
        PartSys.sources[0].sourceFlags.custom1 = 1; // set rocket state to standby
        return true;
    }

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        // Particle System settings
        PartSys.updateSystem(coordinate.width); // update system properties (dimensions and data pointers)
        PartSys.setParticleSize(parameters.check3); // 1 or 2 pixel rendering
        PartSys.setMotionBlur(parameters.custom2); // anable motion blur

        int32_t gravity = (1 + (parameters.speed >> 3));
        if (!parameters.check1) // gravity enabled for sparks
         PartSys.setGravity(0); // disable
        else
         PartSys.setGravity(gravity); // set gravity

        if (PartSys.sources[0].sourceFlags.custom1 == 1) { // rocket is on standby
            PartSys.sources[0].source.ttl--;
            if (PartSys.sources[0].source.ttl == 0) { // time is up, relaunch

                if (hw_random8() < parameters.custom1) // randomly choose direction according to slider, fire at start of segment if true
                    aux0 = 1;
                else
                    aux0 = 0;

                PartSys.sources[0].sourceFlags.custom1 = 0; //flag used for rocket state
                PartSys.sources[0].source.hue = hw_random16();
                PartSys.sources[0].var = 10; // emit variation
                PartSys.sources[0].v = -10; // emit speed
                PartSys.sources[0].minLife = 100;
                PartSys.sources[0].maxLife = 300;
                PartSys.sources[0].source.x = 0; // start from bottom
                uint32_t speed = sqrt((gravity * ((PartSys.maxX >> 2) + hw_random16(PartSys.maxX >> 1))) >> 4); // set speed such that rocket explods in frame
                PartSys.sources[0].source.vx = min(speed, (uint32_t)127);
                PartSys.sources[0].source.ttl = 4000;
                PartSys.sources[0].sat = 30; // low saturation exhaust
                PartSys.sources[0].size = 0; // default size
                PartSys.sources[0].sourceFlags.reversegrav = false ; // normal gravity

                if (aux0) { // inverted rockets launch from end
                    PartSys.sources[0].sourceFlags.reversegrav = true;
                    PartSys.sources[0].source.x = PartSys.maxX; // start from top
                    PartSys.sources[0].source.vx = -PartSys.sources[0].source.vx; // revert direction
                    PartSys.sources[0].v = -PartSys.sources[0].v; // invert exhaust emit speed
                }
            }
        }
        else { // rocket is launched
            int32_t rocketgravity = -gravity;
            int32_t speed = PartSys.sources[0].source.vx;
            if (aux0) { // negative speed rocket
                rocketgravity = -rocketgravity;
                speed = -speed;
            }
            PartSys.applyForce(PartSys.sources[0].source, rocketgravity, forcecounter);
            PartSys.particleMoveUpdate(PartSys.sources[0].source, PartSys.sources[0].sourceFlags);
            PartSys.particleMoveUpdate(PartSys.sources[0].source, PartSys.sources[0].sourceFlags); // increase speed by calling the move function twice, also ages twice
            uint32_t rocketheight = aux0 ? PartSys.maxX - PartSys.sources[0].source.x : PartSys.sources[0].source.x;

            if (speed < 0 && PartSys.sources[0].source.ttl > 50) // reached apogee
                PartSys.sources[0].source.ttl = min((uint32_t)50, rocketheight >> (PS_P_RADIUS_SHIFT_1D + 3)); // alive for a few more frames

            if (PartSys.sources[0].source.ttl < 2) { // explode
                PartSys.sources[0].sourceFlags.custom1 = 1; // set standby state
                PartSys.sources[0].var = 5 + ((((PartSys.maxX >> 1) + rocketheight) * (200 + parameters.intensity)) / (PartSys.maxX << 2)); // set explosion particle speed
                PartSys.sources[0].minLife = 600;
                PartSys.sources[0].maxLife = 1300;
                PartSys.sources[0].source.ttl = 100 + hw_random16(64 - (parameters.speed >> 2)); // standby time til next launch
                PartSys.sources[0].sat = 7 + (parameters.custom3 << 3); //color saturation  TODO: replace saturation with something more useful?
                PartSys.sources[0].size = hw_random16(64); // random particle size in explosion
                uint32_t explosionsize = 8 + ((coordinate.width - 1) >> 2) + (PartSys.sources[0].source.x >> (PS_P_RADIUS_SHIFT_1D - 1));
                explosionsize += hw_random16((explosionsize * parameters.intensity) >> 8);
                for (uint32_t e = 0; e < explosionsize; e++) { // emit explosion particles
                    if (parameters.check2)
                        PartSys.sources[0].source.hue = hw_random16(); //random color for each particle
                    PartSys.sprayEmit(PartSys.sources[0]); // emit a particle
                }
            }
        }
        if ((parameters.call & 0x01) == 0 && PartSys.sources[0].sourceFlags.custom1 == false) // every second frame and not in standby
            PartSys.sprayEmit(PartSys.sources[0]); // emit exhaust particle
        if ((parameters.call & 0x03) == 0) // every fourth frame
            PartSys.applyFriction(1); // apply friction to all particles

        PartSys.update(buffer); // update and render

        for (uint32_t i = 0; i < PartSys.usedParticles; i++) {
            if (PartSys.particles[i].ttl > 10) PartSys.particles[i].ttl -= 10; //ttl is linked to brightness, this allows to use higher brightness but still a short spark lifespan
            else PartSys.particles[i].ttl = 0;
        }
        return true;
    }

private:
    uint8_t forcecounter{};
    uint16_t aux0{};
};


#endif //WLED_DISABLE_PARTICLESYSTEM1D
