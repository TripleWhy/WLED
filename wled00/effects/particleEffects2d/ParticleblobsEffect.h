#pragma once
#ifndef WLED_DISABLE_2D
#ifndef WLED_DISABLE_PARTICLESYSTEM2D

#include "../../FX.h"
#include "../../FXparticleSystem.h"
#include "../BufferedEffect.h"
#include "../Effect.h"

/*
  PS Blobs: large particles bouncing around, changing size and form
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
class ParticleblobsEffect : public BaseEffect<ParticleblobsEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = ParticleblobsEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "PS Blobs@Speed,Blobs,Size,Life,Blur,Wobble,Collide,Pulsate;;!;2v;sx=30,ix=64,c1=200,c2=130,c3=0,o3=1";
    static constexpr const uint8_t effectId = FX_MODE_PARTICLEBLOBS;

    explicit ParticleblobsEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        ParticleSystem2D *PartSys = nullptr;

        if (SEGMENT.call == 0) {
            if (!initParticleSystem2D(PartSys, 1, 0, true, true)) //init, request one source, no additional bytes, advanced size & size control (actually dont really need one TODO: test if using zero sources also works)
                return mode_static(); // allocation failed or not 2D
            PartSys->setBounceX(true);
            PartSys->setBounceY(true);
            PartSys->setWallHardness(255);
            PartSys->setWallRoughness(255);
            PartSys->setCollisionHardness(255);
        }
        else
            PartSys = reinterpret_cast<ParticleSystem2D *>(SEGENV.data); // if not first call, just set the pointer to the PS

        if (PartSys == nullptr)
            return mode_static(); // something went wrong, no data!

        PartSys->updateSystem(); // update system properties (dimensions and data pointers)
        PartSys->setUsedParticles(map(SEGMENT.intensity, 0, 255, 25, 128)); // minimum 10%, maximum 50% of available particles (note: PS ensures at least 1)
        PartSys->enableParticleCollisions(SEGMENT.check2);

        for (uint32_t i = 0; i < PartSys->usedParticles; i++) { // update particles
            if (aux0 != SEGMENT.speed || PartSys->particles[i].ttl == 0) { // speed changed or dead
                PartSys->particles[i].vx = (int8_t)hw_random16(SEGMENT.speed >> 1) - (SEGMENT.speed >> 2); // +/- speed/4
                PartSys->particles[i].vy = (int8_t)hw_random16(SEGMENT.speed >> 1) - (SEGMENT.speed >> 2);
            }
            if (aux1 != SEGMENT.custom1 || PartSys->particles[i].ttl == 0) // size changed or dead
                PartSys->advPartSize[i].maxsize = 60 + (SEGMENT.custom1 >> 1) + hw_random16((SEGMENT.custom1 >> 2)); // set each particle to slightly randomized size

            //PartSys->particles[i].perpetual = SEGMENT.check2; //infinite life if set
            if (PartSys->particles[i].ttl == 0) { // find dead particle, renitialize
                PartSys->particles[i].ttl = 300 + hw_random16(((uint16_t)SEGMENT.custom2 << 3) + 100);
                PartSys->particles[i].x = hw_random(PartSys->maxX);
                PartSys->particles[i].y = hw_random16(PartSys->maxY);
                PartSys->particles[i].hue = hw_random16(); // set random color
                PartSys->particleFlags[i].collide = true; // enable collision for particle
                PartSys->advPartProps[i].size = 0; // start out small
                PartSys->advPartSize[i].asymmetry = hw_random16(220);
                PartSys->advPartSize[i].asymdir = hw_random16(255);
                // set advanced size control properties
                PartSys->advPartSize[i].grow = true;
                PartSys->advPartSize[i].growspeed = 1 + hw_random16(9);
                PartSys->advPartSize[i].shrinkspeed = 1 + hw_random16(9);
                PartSys->advPartSize[i].wobblespeed = 1 + hw_random16(3);
            }
            //PartSys->advPartSize[i].asymmetry++;
            PartSys->advPartSize[i].pulsate = SEGMENT.check3;
            PartSys->advPartSize[i].wobble = SEGMENT.check1;
        }
        aux0 = SEGMENT.speed; //write state back
        aux1 = SEGMENT.custom1;

        #ifdef USERMOD_AUDIOREACTIVE
        um_data_t *um_data;
        if (UsermodManager::getUMData(&um_data, USERMOD_ID_AUDIOREACTIVE)) { // get AR data, do not use simulated data
            uint8_t volumeSmth = (uint8_t)(*(float*)um_data->u_data[0]);
            for (uint32_t i = 0; i < PartSys->usedParticles; i++) { // update particles
                if (SEGMENT.check3) //pulsate selected
                    PartSys->advPartProps[i].size = volumeSmth;
            }
        }
        #endif

        PartSys->setMotionBlur(((SEGMENT.custom3) << 3) + 7);
        PartSys->update(); // update and render
        return true;
    }

private:
    uint16_t aux0{};
    uint16_t aux1{};
};

#endif //WLED_DISABLE_2D
#endif //WLED_DISABLE_PARTICLESYSTEM2D
