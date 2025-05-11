#pragma once
#ifndef WLED_DISABLE_PARTICLESYSTEM1D

#include "../../FX.h"
#include "../Effect.h"
#include "Particle1dEffect.h"

/*
  Particle based balance: particles move back and forth (1D pendent to 2D particle box)
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
class ParticleBalanceEffect : public BaseEffect<ParticleBalanceEffect, Particle1dEffect<ParticleBalanceEffect>> {
private:
    using Self = ParticleBalanceEffect;
    using Base = BaseEffect<Self, Particle1dEffect<Self>>;

public:
    static constexpr const char metaData[] PROGMEM = "PS 1D Balance@!,!,Hardness,Blur,Tilt,Position Color,Wrap,Random;,!;!;1;pal=18,c2=0,c3=4,o1=1";
    static constexpr const uint8_t effectId = FX_MODE_PSBALANCE;

    using Base::Base;

    bool init(const EffectCoordinate& coordinate) {
        if (!Base::init(coordinate, 1, 128, false)) {
            return false;
        }

        //PartSys.setKillOutOfBounds(true);
        PartSys.setParticleSize(1);
        return true;
    }

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        uint32_t i;

        // Particle System settings
        PartSys.updateSystem(coordinate.width); // update system properties (dimensions and data pointers)
        PartSys.setMotionBlur(parameters.custom2); // enable motion blur
        PartSys.setBounce(!parameters.check2);
        PartSys.setWrap(parameters.check2);
        uint8_t hardness = parameters.custom1 > 0 ? map(parameters.custom1, 0, 255, 50, 250) : 200; // set hardness,  make the walls hard if collisions are disabled
        PartSys.enableParticleCollisions(parameters.custom1, hardness); // enable collisions if custom1 > 0
        PartSys.setWallHardness(200);
        PartSys.setUsedParticles(map(parameters.intensity, 0, 255, 10, 255));
        if (PartSys.usedParticles > aux1) { // more particles, reinitialize
            for (i = 0; i < PartSys.usedParticles; i++) {
                PartSys.particles[i].x = i * PS_P_RADIUS_1D;
                PartSys.particles[i].ttl = 300;
                PartSys.particleFlags[i].perpetual = true;
                PartSys.particleFlags[i].collide = true;
            }
        }
        aux1 = PartSys.usedParticles;

        // re-order particles in case collisions flipped particles
        for (i = 0; i < PartSys.usedParticles - 1; i++) {
            if (PartSys.particles[i].x > PartSys.particles[i+1].x) {
                if (parameters.check2) { // check for wrap around
                    if (PartSys.particles[i].x - PartSys.particles[i+1].x > 3 * PS_P_RADIUS_1D)
                        continue;
                }
                std::swap(PartSys.particles[i].x, PartSys.particles[i+1].x);
            }
        }

        if (parameters.call % (((255 - parameters.speed) >> 6) + 1) == 0) { // how often the force is applied depends on speed setting
            int32_t xgravity;
            int32_t increment = (parameters.speed >> 6) + 1;
            aux0 += increment;
            if (parameters.check3) // random, use perlin noise
                xgravity = ((int16_t)perlin8(aux0) - 128);
            else // sinusoidal
                xgravity = (int16_t)cos8_t(aux0) - 128;//((int32_t)(parameters.custom3 << 2) * cos8(aux0)
            // scale the force
            xgravity = (xgravity * ((parameters.custom3+1) << 2)) / 128; // xgravity: -127 to +127
            PartSys.applyForce(xgravity);
        }

        uint32_t randomindex = hw_random16(PartSys.usedParticles);
        PartSys.particles[randomindex].vx = ((int32_t)PartSys.particles[randomindex].vx * 200) / 255;  // apply friction to random particle to reduce clumping (without collisions)

        //if (parameters.check2 && (parameters.call & 0x07) == 0) // no walls, apply friction to smooth things out
        if ((parameters.call & 0x0F) == 0 && parameters.custom3 > 4) // apply friction every 16th frame to smooth things out (except for low tilt)
            PartSys.applyFriction(1); // apply friction to all particles

        //update colors
        PartSys.setColorByPosition(parameters.check1);
        if (!parameters.check1) {
            for (i = 0; i < PartSys.usedParticles; i++) {
                    PartSys.particles[i].hue = (1024 * i) / PartSys.usedParticles; // color by particle index
            }
        }
        PartSys.update(buffer); // update and render
        return true;
    }

private:
    uint16_t aux0{};
    uint16_t aux1{};
};


#endif //WLED_DISABLE_PARTICLESYSTEM1D
