#pragma once
#ifndef WLED_DISABLE_PARTICLESYSTEM1D

#include "../../FX.h"
#include "../../FXparticleSystem.h"
#include "../BufferedEffect.h"
#include "../Effect.h"

///////////////////////////
// 1D Particle System FX //
///////////////////////////

/*
  Particle version of Drip and Rain
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
class ParticledripEffect : public BaseEffect<ParticledripEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = ParticledripEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "PS DripDrop@Speed,!,Splash,Blur,Gravity,Rain,PushSplash,Smooth;,!;!;1;pal=0,sx=150,ix=25,c1=220,c2=30,c3=21";
    static constexpr const uint8_t effectId = FX_MODE_PARTICLEDRIP;

    explicit ParticledripEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        ParticleSystem1D *PartSys = nullptr;
        //uint8_t numSprays;
        if (SEGMENT.call == 0) { // initialization
            if (!initParticleSystem1D(PartSys, 4)) // init
                return mode_static(); // allocation failed or single pixel
            PartSys->setKillOutOfBounds(true); // out of bounds particles dont return (except on top, taken care of by gravity setting)
            PartSys->sources[0].source.hue = hw_random16();
            aux1 = 0xFFFF; // invalidate
        }
        else
            PartSys = reinterpret_cast<ParticleSystem1D *>(SEGENV.data); // if not first call, just set the pointer to the PS

        if (PartSys == nullptr)
            return mode_static(); // something went wrong, no data!

        // Particle System settings
        PartSys->updateSystem(); // update system properties (dimensions and data pointers)
        PartSys->setBounce(true);
        PartSys->setWallHardness(50);

        PartSys->setMotionBlur(SEGMENT.custom2); // anable motion blur
        PartSys->setGravity(SEGMENT.custom3 >> 1); // set gravity (8 is default strength)
        PartSys->setParticleSize(SEGMENT.check3); // 1 or 2 pixel rendering

        if (SEGMENT.check2) { //collisions enabled
            PartSys->enableParticleCollisions(true); //enable, full hardness
        }
        else
            PartSys->enableParticleCollisions(false);

        PartSys->sources[0].sourceFlags.collide = false; //drops do not collide

        if (SEGMENT.check1) { //rain mode, emit at random position, short life (3-8 seconds at 50fps)
            if (SEGMENT.custom1 == 0) //splash disabled, do not bounce raindrops
                PartSys->setBounce(false);
            PartSys->sources[0].var = 5;
            PartSys->sources[0].v = -(8 + (SEGMENT.speed >> 2)); //speed + var must be < 128, inverted speed (=down)
            // lifetime in frames
            PartSys->sources[0].minLife = 30;
            PartSys->sources[0].maxLife = 200;
            PartSys->sources[0].source.x = hw_random(PartSys->maxX); //random emit position
        }
        else { //drip
            PartSys->sources[0].var = 0;
            PartSys->sources[0].v = -(SEGMENT.speed >> 1); //speed + var must be < 128, inverted speed (=down)
            PartSys->sources[0].minLife = 3000;
            PartSys->sources[0].maxLife = 3000;
            PartSys->sources[0].source.x = PartSys->maxX - PS_P_RADIUS_1D;
        }

        if (aux1 != SEGMENT.intensity) //slider changed
            aux0 = 1; //must not be zero or "% 0" happens below which crashes on ESP32

        aux1 = SEGMENT.intensity; // save state

        // every nth frame emit a particle
        if (SEGMENT.call % aux0 == 0) {
            int32_t interval = 300 / ((SEGMENT.intensity) + 1);
            aux0 = interval + hw_random(interval + 5);
            // if (SEGMENT.check1) // rain mode
            //   PartSys->sources[0].source.hue = 0;
            // else
            PartSys->sources[0].source.hue = hw_random8(); //set random color  TODO: maybe also not random but color cycling? need another slider or checkmark for this.
            PartSys->sprayEmit(PartSys->sources[0]);
        }

        for (uint32_t i = 0; i < PartSys->usedParticles; i++) { //check all particles
            if (PartSys->particles[i].ttl && PartSys->particleFlags[i].collide == false) { // use collision flag to identify splash particles
                if (SEGMENT.custom1 > 0 && PartSys->particles[i].x < (PS_P_RADIUS_1D << 1)) { //splash enabled and reached bottom
                    PartSys->particles[i].ttl = 0; //kill origin particle
                    PartSys->sources[0].maxLife = 80;
                    PartSys->sources[0].minLife = 20;
                    PartSys->sources[0].var = 10 + (SEGMENT.custom1 >> 3);
                    PartSys->sources[0].v = 0;
                    PartSys->sources[0].source.hue = PartSys->particles[i].hue;
                    PartSys->sources[0].source.x = PS_P_RADIUS_1D;
                    PartSys->sources[0].sourceFlags.collide = true; //splashes do collide if enabled
                    for (int j = 0; j < 2 + (SEGMENT.custom1 >> 2); j++) {
                        PartSys->sprayEmit(PartSys->sources[0]);
                    }
                }
            }

            if (SEGMENT.check1) { //rain mode, fade hue to max
                if (PartSys->particles[i].hue < 245)
                    PartSys->particles[i].hue += 8;
            }
            //increase speed on high settings by calling the move function twice
            if (SEGMENT.speed > 200)
                PartSys->particleMoveUpdate(PartSys->particles[i], PartSys->particleFlags[i]);
        }

        PartSys->update(); // update and render
        return true;
    }

private:
    uint16_t aux0{};
    uint16_t aux1{};
};


#endif //WLED_DISABLE_PARTICLESYSTEM1D
