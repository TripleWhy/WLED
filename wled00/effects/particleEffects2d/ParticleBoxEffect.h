#pragma once
#ifndef WLED_DISABLE_2D
#ifndef WLED_DISABLE_PARTICLESYSTEM2D

#include "../../FX.h"
#include "../Effect.h"
#include "Particle2dEffect.h"

/*
  Particle Box, applies gravity to particles in either a random direction or random but only downwards (sloshing)
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
class ParticleBoxEffect : public BaseEffect<ParticleBoxEffect, Particle2dEffect<ParticleBoxEffect>> {
private:
    using Self = ParticleBoxEffect;
    using Base = BaseEffect<Self, Particle2dEffect<Self>>;

public:
    static constexpr const char metaData[] PROGMEM = "PS Box@!,Particles,Tilt,Hardness,Size,Random,Washing Machine,Sloshing;;!;2;pal=53,ix=50,c3=1,o1=1";
    static constexpr const uint8_t effectId = FX_MODE_PARTICLEBOX;

    using Base::Base;

    bool init(const EffectCoordinate& coordinate) {
        if (!Base::init(coordinate, 1, false, false)) {
            return false;
        }

        PartSys.setBounceX(true);
        PartSys.setBounceY(true);
        return true;
    }

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        uint32_t i;

        PartSys.updateSystem(coordinate.width, coordinate.height); // update system properties (dimensions and data pointers)
        PartSys.setParticleSize(SEGMENT.custom3<<3);
        PartSys.setWallHardness(min(SEGMENT.custom2, (uint8_t)200)); // wall hardness is 200 or more
        PartSys.enableParticleCollisions(true, max(2, (int)SEGMENT.custom2)); // enable collisions and set particle collision hardness
        PartSys.setUsedParticles(map(SEGMENT.intensity, 0, 255, 2, 153)); // 1% - 60%
        // add in new particles if amount has changed
        for (i = 0; i < PartSys.usedParticles; i++) {
            if (PartSys.particles[i].ttl < 260) { // initialize handed over particles and dead particles
                PartSys.particles[i].ttl = 260; // full brigthness
                PartSys.particles[i].x = hw_random16(PartSys.maxX);
                PartSys.particles[i].y = hw_random16(PartSys.maxY);
                PartSys.particles[i].hue = hw_random8(); // make it colorful
                PartSys.particleFlags[i].perpetual = true; // never die
                PartSys.particleFlags[i].collide = true; // all particles colllide
                break; // only spawn one particle per frame for less chaotic transitions
            }
        }

        if (SEGMENT.call % (((255 - SEGMENT.speed) >> 6) + 1) == 0 && SEGMENT.speed > 0) { // how often the force is applied depends on speed setting
            int32_t xgravity;
            int32_t ygravity;
            int32_t increment = (SEGMENT.speed >> 6) + 1;

            if (SEGMENT.check2) { // washing machine
                int speed = tristate_square8(strip.now >> 7, 90, 15) / ((400 - SEGMENT.speed) >> 3);
                aux0 += speed;
                if (speed == 0) aux0 = 190; //down (= 270°)
            }
            else
                aux0 -= increment;

            if (SEGMENT.check1) { // random, use perlin noise
                xgravity = ((int16_t)perlin8(aux0) - 127);
                ygravity = ((int16_t)perlin8(aux0 + 10000) - 127);
                // scale the gravity force
                xgravity = (xgravity * SEGMENT.custom1) / 128;
                ygravity = (ygravity * SEGMENT.custom1) / 128;
            }
            else { // go in a circle
                xgravity = ((int32_t)(SEGMENT.custom1) * cos16_t(aux0 << 8)) / 0xFFFF;
                ygravity = ((int32_t)(SEGMENT.custom1) * sin16_t(aux0 << 8)) / 0xFFFF;
            }
            if (SEGMENT.check3) { // sloshing, y force is always downwards
                if (ygravity > 0)
                    ygravity = -ygravity;
            }

            PartSys.applyForce(xgravity, ygravity);
        }

        if ((SEGMENT.call & 0x0F) == 0) // every 16th frame
            PartSys.applyFriction(1);

        PartSys.update(buffer);   // update and render
        return true;
    }

private:
    uint16_t aux0{hw_random16()};
};


#endif //WLED_DISABLE_2D
#endif //WLED_DISABLE_PARTICLESYSTEM2D
