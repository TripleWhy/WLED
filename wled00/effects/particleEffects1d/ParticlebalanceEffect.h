#pragma once
#ifndef WLED_DISABLE_PARTICLESYSTEM1D

#include "../../FX.h"
#include "../../FXparticleSystem.h"
#include "../BufferedEffect.h"
#include "../Effect.h"

/*
  Particle based balance: particles move back and forth (1D pendent to 2D particle box)
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
class ParticlebalanceEffect : public BaseEffect<ParticlebalanceEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = ParticlebalanceEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char* const metaData = "PS 1D Balance@!,!,Hardness,Blur,Tilt,Position Color,Wrap,Random;,!;!;1;pal=18,c2=0,c3=4,o1=1";
    static constexpr const uint8_t effectId = FX_MODE_PS_BALANCE;

    explicit ParticlebalanceEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        ParticleSystem1D *PartSys = nullptr;
        uint32_t i;

        if (SEGMENT.call == 0) { // initialization
            if (!initParticleSystem1D(PartSys, 1, 128)) // init, no additional data needed, use half of max particles
                return mode_static(); // allocation failed or is single pixel
            //PartSys->setKillOutOfBounds(true);
            PartSys->setParticleSize(1);
            aux0 = 0;
            aux1 = 0; //TODO: really need to set to zero or is it calloc'd?
        }
        else
            PartSys = reinterpret_cast<ParticleSystem1D *>(SEGENV.data); // if not first call, just set the pointer to the PS
        if (PartSys == nullptr)
            return mode_static(); // something went wrong, no data!

        // Particle System settings
        PartSys->updateSystem(); // update system properties (dimensions and data pointers)
        PartSys->setMotionBlur(SEGMENT.custom2); // anable motion blur
        PartSys->setBounce(!SEGMENT.check2);
        PartSys->setWrap(SEGMENT.check2);
        uint8_t hardness = SEGMENT.custom1 > 0 ? map(SEGMENT.custom1, 0, 255, 50, 250) : 200; // set hardness,  make the walls hard if collisions are disabled
        PartSys->enableParticleCollisions(SEGMENT.custom1, hardness); // enable collisions if custom1 > 0
        PartSys->setWallHardness(200);
        PartSys->setUsedParticles(map(SEGMENT.intensity, 0, 255, 10, 255));
        if (PartSys->usedParticles > aux1) { // more particles, reinitialize
            for (i = 0; i < PartSys->usedParticles; i++) {
                PartSys->particles[i].x = i * PS_P_RADIUS_1D;
                PartSys->particles[i].ttl = 300;
                PartSys->particleFlags[i].perpetual = true;
                PartSys->particleFlags[i].collide = true;
            }
        }
        aux1 = PartSys->usedParticles;

        if (SEGMENT.call % (((255 - SEGMENT.speed) >> 6) + 1) == 0) { // how often the force is applied depends on speed setting
            int32_t xgravity;
            int32_t increment = (SEGMENT.speed >> 6) + 1;
            aux0 += increment;
            if (SEGMENT.check3) // random, use perlin noise
                xgravity = ((int16_t)inoise8(aux0) - 128);
            else // sinusoidal
                xgravity = (int16_t)cos8(aux0) - 128;//((int32_t)(SEGMENT.custom3 << 2) * cos8(aux0)
            // scale the force
            xgravity = (xgravity * ((SEGMENT.custom3+1) << 2)) / 128; // xgravity: -127 to +127
            PartSys->applyForce(xgravity);
        }

        uint32_t randomindex = hw_random16(PartSys->usedParticles);
        PartSys->particles[randomindex].vx = ((int32_t)PartSys->particles[randomindex].vx * 200) / 255;  // apply friction to random particle to reduce clumping (without collisions)

        //if (SEGMENT.check2 && (SEGMENT.call & 0x07) == 0) // no walls, apply friction to smooth things out
        if ((SEGMENT.call & 0x0F) == 0 && SEGMENT.custom3 > 4) // apply friction every 16th frame to smooth things out (except for low tilt)
            PartSys->applyFriction(1); // apply friction to all particles

        //update colors
        PartSys->setColorByPosition(SEGMENT.check1);
        if (!SEGMENT.check1) {
            for (i = 0; i < PartSys->usedParticles; i++) {
                    PartSys->particles[i].hue = (1024 * i) / PartSys->usedParticles; // color by particle index
            }
        }
        PartSys->update(); // update and render
    }

private:
    uint16_t aux0{};
    uint16_t aux1{};
};


#endif //WLED_DISABLE_PARTICLESYSTEM1D
