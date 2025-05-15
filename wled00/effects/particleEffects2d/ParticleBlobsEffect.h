#pragma once
#ifndef WLED_DISABLE_2D
#ifndef WLED_DISABLE_PARTICLESYSTEM2D

#include "../../FX.h"
#include "../Effect.h"
#include "Particle2dEffect.h"

/*
  PS Blobs: large particles bouncing around, changing size and form
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
class ParticleBlobsEffect : public BaseEffect<ParticleBlobsEffect, Particle2dEffect<ParticleBlobsEffect>> {
private:
    using Self = ParticleBlobsEffect;
    using Base = BaseEffect<Self, Particle2dEffect<Self>>;

public:
    static constexpr const char metaData[] PROGMEM = "PS Blobs@Speed,Blobs,Size,Life,Blur,Wobble,Collide,Pulsate;;!;2v;sx=30,ix=64,c1=200,c2=130,c3=0,o3=1";
    static constexpr const uint8_t effectId = FX_MODE_PARTICLEBLOBS;

    using Base::Base;

    bool init(const EffectCoordinate& coordinate) {
        if (!Base::init(coordinate, 1, true, true)) {
            return false;
        }

        PartSys.setBounceX(true);
        PartSys.setBounceY(true);
        PartSys.setWallHardness(255);
        PartSys.setWallRoughness(255);
        PartSys.setCollisionHardness(255);
        return true;
    }

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        PartSys.updateSystem(coordinate.width, coordinate.height); // update system properties (dimensions and data pointers)
        PartSys.setUsedParticles(map(parameters.intensity, 0, 255, 25, 128)); // minimum 10%, maximum 50% of available particles (note: PS ensures at least 1)
        PartSys.enableParticleCollisions(parameters.check2);

        for (uint32_t i = 0; i < PartSys.usedParticles; i++) { // update particles
            if (aux0 != parameters.speed || PartSys.particles[i].ttl == 0) { // speed changed or dead
                PartSys.particles[i].vx = (int8_t)hw_random16(parameters.speed >> 1) - (parameters.speed >> 2); // +/- speed/4
                PartSys.particles[i].vy = (int8_t)hw_random16(parameters.speed >> 1) - (parameters.speed >> 2);
            }
            if (aux1 != parameters.custom1 || PartSys.particles[i].ttl == 0) // size changed or dead
                PartSys.advPartSize[i].maxsize = 60 + (parameters.custom1 >> 1) + hw_random16((parameters.custom1 >> 2)); // set each particle to slightly randomized size

            //PartSys.particles[i].perpetual = parameters.check2; //infinite life if set
            if (PartSys.particles[i].ttl == 0) { // find dead particle, renitialize
                PartSys.particles[i].ttl = 300 + hw_random16(((uint16_t)parameters.custom2 << 3) + 100);
                PartSys.particles[i].x = hw_random(PartSys.maxX);
                PartSys.particles[i].y = hw_random16(PartSys.maxY);
                PartSys.particles[i].hue = hw_random16(); // set random color
                PartSys.particleFlags[i].collide = true; // enable collision for particle
                PartSys.advPartProps[i].size = 0; // start out small
                PartSys.advPartSize[i].asymmetry = hw_random16(220);
                PartSys.advPartSize[i].asymdir = hw_random16(255);
                // set advanced size control properties
                PartSys.advPartSize[i].grow = true;
                PartSys.advPartSize[i].growspeed = 1 + hw_random16(9);
                PartSys.advPartSize[i].shrinkspeed = 1 + hw_random16(9);
                PartSys.advPartSize[i].wobblespeed = 1 + hw_random16(3);
            }
            //PartSys.advPartSize[i].asymmetry++;
            PartSys.advPartSize[i].pulsate = parameters.check3;
            PartSys.advPartSize[i].wobble = parameters.check1;
        }
        aux0 = parameters.speed; //write state back
        aux1 = parameters.custom1;

        #ifdef USERMOD_AUDIOREACTIVE
        um_data_t *um_data;
        if (UsermodManager::getUMData(&um_data, USERMOD_ID_AUDIOREACTIVE)) { // get AR data, do not use simulated data
            uint8_t volumeSmth = (uint8_t)(*(float*)um_data->u_data[0]);
            for (uint32_t i = 0; i < PartSys.usedParticles; i++) { // update particles
                if (parameters.check3) //pulsate selected
                    PartSys.advPartProps[i].size = volumeSmth;
            }
        }
        #endif

        PartSys.setMotionBlur(((parameters.custom3) << 3) + 7);
        PartSys.update(buffer, parameters); // update and render
        return true;
    }

private:
    uint16_t aux0{};
    uint16_t aux1{};
};

#endif //WLED_DISABLE_2D
#endif //WLED_DISABLE_PARTICLESYSTEM2D
