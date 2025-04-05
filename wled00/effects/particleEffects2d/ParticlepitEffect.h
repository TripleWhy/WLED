#pragma once
#ifndef WLED_DISABLE_2D
#ifndef WLED_DISABLE_PARTICLESYSTEM2D

#include "../../FX.h"
#include "../../FXparticleSystem.h"
#include "../BufferedEffect.h"
#include "../Effect.h"

/*
  PS Ballpit: particles falling down, user can enable these three options: X-wraparound, side bounce, ground bounce
  sliders control falling speed, intensity (number of particles spawned), inter-particle collision hardness (0 means no particle collisions) and render saturation
  this is quite versatile, can be made to look like rain or snow or confetti etc.
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
class ParticlepitEffect : public BaseEffect<ParticlepitEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = ParticlepitEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char* const metaData = "PS Ballpit@Speed,Intensity,Size,Hardness,Saturation,Cylinder,Walls,Ground;;!;2;pal=11,sx=100,ix=220,c1=120,c2=130,c3=31,o3=1";
    static constexpr const uint8_t effectId = FX_MODE_PARTICLEPIT;

    explicit ParticlepitEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        ParticleSystem2D *PartSys = nullptr;

        if (SEGMENT.call == 0) { // initialization
            if (!initParticleSystem2D(PartSys, 1, 0, true, false)) // init, request one source (actually dont really need one TODO: test if using zero sources also works)
                return mode_static(); // allocation failed or not 2D
            PartSys->setKillOutOfBounds(true);
            PartSys->setGravity(); // enable with default gravity
            PartSys->setUsedParticles(170); // use 75% of available particles
        }
        else
            PartSys = reinterpret_cast<ParticleSystem2D *>(SEGENV.data); // if not first call, just set the pointer to the PS
        if (PartSys == nullptr)
            return mode_static(); // something went wrong, no data!

        PartSys->updateSystem(); // update system properties (dimensions and data pointers)

        PartSys->setWrapX(SEGMENT.check1);
        PartSys->setBounceX(SEGMENT.check2);
        PartSys->setBounceY(SEGMENT.check3);
        PartSys->setWallHardness(min(SEGMENT.custom2, (uint8_t)150)); // limit to 100 min (if collisions are disabled, still want bouncy)
        if (SEGMENT.custom2 > 0)
            PartSys->enableParticleCollisions(true, SEGMENT.custom2); // enable collisions and set particle collision hardness
        else
            PartSys->enableParticleCollisions(false);

        uint32_t i;
        if (SEGMENT.call % (128 - (SEGMENT.intensity >> 1)) == 0 && SEGMENT.intensity > 0) { // every nth frame emit particles, stop emitting if set to zero
            for (i = 0; i < PartSys->usedParticles; i++) { // emit particles
                if (PartSys->particles[i].ttl == 0) { // find a dead particle
                    // emit particle at random position over the top of the matrix (random16 is not random enough)
                    PartSys->particles[i].ttl = 1500 - (SEGMENT.speed << 2) + hw_random16(500); // if speed is higher, make them die sooner
                    PartSys->particles[i].x = hw_random(PartSys->maxX); //random(PartSys->maxX >> 1) + (PartSys->maxX >> 2);
                    PartSys->particles[i].y = (PartSys->maxY << 1); // particles appear somewhere above the matrix, maximum is double the height
                    PartSys->particles[i].vx = (int16_t)hw_random16(SEGMENT.speed >> 1) - (SEGMENT.speed >> 2); // side speed is +/-
                    PartSys->particles[i].vy = map(SEGMENT.speed, 0, 255, -5, -100); // downward speed
                    PartSys->particles[i].hue = hw_random16(); // set random color
                    PartSys->particleFlags[i].collide = true; // enable collision for particle
                    PartSys->particles[i].sat = ((SEGMENT.custom3) << 3) + 7;
                    // set particle size
                    if (SEGMENT.custom1 == 255) {
                        PartSys->setParticleSize(1); // set global size to 1 for advanced rendering
                        PartSys->advPartProps[i].size = hw_random16(SEGMENT.custom1); // set each particle to random size
                    } else {
                        PartSys->setParticleSize(SEGMENT.custom1); // set global size
                        PartSys->advPartProps[i].size = 0; // use global size
                    }
                    break; // emit only one particle per round
                }
            }
        }

        uint32_t frictioncoefficient = 1 + SEGMENT.check1; //need more friction if wrapX is set, see below note
        if (SEGMENT.speed < 50) // for low speeds, apply more friction
            frictioncoefficient = 50 - SEGMENT.speed;

        if (SEGMENT.call % 6 == 0)// (3 + max(3, (SEGMENT.speed >> 2))) == 0) // note: if friction is too low, hard particles uncontrollably 'wander' left and right if wrapX is enabled
            PartSys->applyFriction(frictioncoefficient);

        PartSys->update(); // update and render
    }

private:
};


#endif //WLED_DISABLE_2D
#endif //WLED_DISABLE_PARTICLESYSTEM2D
