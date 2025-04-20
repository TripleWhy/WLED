#pragma once
#ifndef WLED_DISABLE_2D
#ifndef WLED_DISABLE_PARTICLESYSTEM2D

#include "../../FX.h"
#include "../Effect.h"
#include "Particle2dEffect.h"

/*
  Fuzzy Noise: Perlin noise 'gravity' mapping as in particles on 'noise hills' viewed from above
  calculates slope gradient at the particle positions and applies 'downhill' force, resulting in a fuzzy perlin noise display
  by DedeHai (Damian Schneider)
*/
class ParticlePerlinEffect : public BaseEffect<ParticlePerlinEffect, Particle2dEffect<ParticlePerlinEffect>> {
private:
    using Self = ParticlePerlinEffect;
    using Base = BaseEffect<Self, Particle2dEffect<Self>>;

public:
    static constexpr const char metaData[] PROGMEM = "PS Fuzzy Noise@Speed,Particles,Bounce,Friction,Scale,Cylinder,Smear,Collide;;!;2;pal=64,sx=50,ix=200,c1=130,c2=30,c3=5,o3=1";
    static constexpr const uint8_t effectId = FX_MODE_PARTICLEPERLIN;

    using Base::Base;

    bool init(const EffectCoordinate& coordinate) {
        if (!Base::init(coordinate, 1, true, false)) {
            return false;
        }

        PartSys.setKillOutOfBounds(true); // should never happen, but lets make sure there are no stray particles
        PartSys.setMotionBlur(230); // anable motion blur
        PartSys.setBounceY(true);
        return true;
    }

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        uint32_t i;

        PartSys.updateSystem(coordinate.width, coordinate.height); // update system properties (dimensions and data pointers)
        PartSys.setWrapX(SEGMENT.check1);
        PartSys.setBounceX(!SEGMENT.check1);
        PartSys.setWallHardness(SEGMENT.custom1); // wall hardness
        PartSys.enableParticleCollisions(SEGMENT.check3, SEGMENT.custom1); // enable collisions and set particle collision hardness
        PartSys.setUsedParticles(map(SEGMENT.intensity, 0, 255, 25, 128)); // min is 10%, max is 50%
        PartSys.setSmearBlur(SEGMENT.check2 * 15); // enable 2D blurring (smearing)

        // apply 'gravity' from a 2D perlin noise map
        aux0 += 1 + (SEGMENT.speed >> 5); // noise z-position
        // update position in noise
        for (i = 0; i < PartSys.usedParticles; i++) {
            if (PartSys.particles[i].ttl == 0) { // revive dead particles (do not keep them alive forever, they can clump up, need to reseed)
                PartSys.particles[i].ttl = hw_random16(500) + 200;
                PartSys.particles[i].x = hw_random(PartSys.maxX);
                PartSys.particles[i].y = hw_random(PartSys.maxY);
                PartSys.particleFlags[i].collide = true; // particle colllides
            }
            uint32_t scale = 16 - ((31 - SEGMENT.custom3) >> 1);
            uint16_t xnoise = PartSys.particles[i].x / scale; // position in perlin noise, scaled by slider
            uint16_t ynoise = PartSys.particles[i].y / scale;
            int16_t baseheight = perlin8(xnoise, ynoise, aux0); // noise value at particle position
            PartSys.particles[i].hue = baseheight; // color particles to perlin noise value
            if (SEGMENT.call % 8 == 0) { // do not apply the force every frame, is too chaotic
                int8_t xslope = (baseheight + (int16_t)perlin8(xnoise - 10, ynoise, aux0));
                int8_t yslope = (baseheight + (int16_t)perlin8(xnoise, ynoise - 10, aux0));
                PartSys.applyForce(i, xslope, yslope);
            }
        }

        if (SEGMENT.call % (16 - (SEGMENT.custom2 >> 4)) == 0)
            PartSys.applyFriction(2);

        PartSys.update(buffer); // update and render
        return true;
    }

private:
    uint16_t aux0{static_cast<uint16_t>(rand())}; //TODO replace with different rng
};


#endif //WLED_DISABLE_2D
#endif //WLED_DISABLE_PARTICLESYSTEM2D
