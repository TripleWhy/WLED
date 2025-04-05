#pragma once
#ifndef WLED_DISABLE_PARTICLESYSTEM1D

#include "../../FX.h"
#include "../../FXparticleSystem.h"
#include "../BufferedEffect.h"
#include "../Effect.h"

/*
  Particle Replacement for original Dancing Shadows:
  "Spotlights moving back and forth that cast dancing shadows.
  Shine this through tree branches/leaves or other close-up objects that cast
  interesting shadows onto a ceiling or tarp.
  By Steve Pomeroy @xxv"
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
class ParticledancingshadowsEffect : public BaseEffect<ParticledancingshadowsEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = ParticledancingshadowsEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char* const metaData = "PS Dancing Shadows@Speed,!,Blur,Color Cycle,,Smear,Position Color,Smooth;,!;!;1;sx=100,ix=180,c1=0,c2=0";
    static constexpr const uint8_t effectId = FX_MODE_PARTICLEDANCINGSHADOWS;

    explicit ParticledancingshadowsEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        ParticleSystem1D *PartSys = nullptr;

        if (SEGMENT.call == 0) { // initialization
            if (!initParticleSystem1D(PartSys, 1)) // init, one source
                return mode_static(); // allocation failed or is single pixel
            PartSys->sources[0].maxLife = 1000; //set long life (kill out of bounds is done in custom way)
            PartSys->sources[0].minLife = PartSys->sources[0].maxLife;
        }
        else {
            PartSys = reinterpret_cast<ParticleSystem1D *>(SEGENV.data); // if not first call, just set the pointer to the PS
        }

        if (PartSys == nullptr)
            return mode_static(); // something went wrong, no data!

        // Particle System settings
        PartSys->updateSystem(); // update system properties (dimensions and data pointers)
        PartSys->setMotionBlur(SEGMENT.custom1);
        if (SEGMENT.check1)
            PartSys->setSmearBlur(120); // enable smear blur
        else
            PartSys->setSmearBlur(0); // disable smear blur
        PartSys->setParticleSize(SEGMENT.check3); // 1 or 2 pixel rendering
        PartSys->setColorByPosition(SEGMENT.check2); // color fixed by position
        PartSys->setUsedParticles(map(SEGMENT.intensity, 0, 255, 10, 255)); // set percentage of particles to use

        uint32_t deadparticles = 0;
        //kill out of bounds and moving away plus change color
        for (uint32_t i = 0; i < PartSys->usedParticles; i++) {
            if (((SEGMENT.call & 0x07) == 0) && PartSys->particleFlags[i].outofbounds) { //check if out of bounds particle move away from strip, only update every 8th frame
                if ((int32_t)PartSys->particles[i].vx * PartSys->particles[i].x > 0) PartSys->particles[i].ttl = 0; //particle is moving away, kill it
            }
            PartSys->particleFlags[i].perpetual = true; //particles do not age
            if (SEGMENT.call % (32 / (1 + (SEGMENT.custom2 >> 3))) == 0)
                 PartSys->particles[i].hue += 2 + (SEGMENT.custom2 >> 5);
            //note: updating speed on the fly is not accurately possible, since it is unknown which particles are assigned to which spot
            if (aux0 != SEGMENT.speed) { //speed changed
                //update all particle speed by setting them to current value
                 PartSys->particles[i].vx = PartSys->particles[i].vx > 0 ? SEGMENT.speed >> 3 : -SEGMENT.speed >> 3;
            }
            if (PartSys->particles[i].ttl == 0) deadparticles++; // count dead particles
        }
        aux0 = SEGMENT.speed;

        //generate a spotlight: generates particles just outside of view
        if (deadparticles > 5 && (SEGMENT.call & 0x03) == 0) {
            //random color, random type
            uint32_t type = hw_random16(SPOT_TYPES_COUNT);
            int8_t speed = 2 + hw_random16(2 + (SEGMENT.speed >> 1)) + (SEGMENT.speed >> 4);
            int32_t width = hw_random16(1, 10);
            uint32_t ttl = 300; //ttl is particle brightness (below perpetual is set so it does not age, i.e. ttl stays at this value)
            int32_t position;
            //choose random start position, left and right from the segment
            if (hw_random() & 0x01) {
                position = PartSys->maxXpixel;
                speed = -speed;
            }
            else
                position = -width;

            PartSys->sources[0].v = speed; //emitted particle speed
            PartSys->sources[0].source.hue = hw_random8(); //random spotlight color
            for (int32_t i = 0; i < width; i++) {
                if (width > 1) {
                    switch (type) {
                        case SPOT_TYPE_SOLID:
                            //nothing to do
                            break;

                        case SPOT_TYPE_GRADIENT:
                            ttl = cubicwave8(map(i, 0, width - 1, 0, 255));
                            ttl = ttl*ttl >> 8; //make gradient more pronounced
                            break;

                        case SPOT_TYPE_2X_GRADIENT:
                            ttl = cubicwave8(2 * map(i, 0, width - 1, 0, 255));
                            ttl = ttl*ttl >> 8;
                            break;

                        case SPOT_TYPE_2X_DOT:
                            if (i > 0) position++; //skip one pixel
                            i++;
                            break;

                        case SPOT_TYPE_3X_DOT:
                            if (i > 0) position += 2; //skip two pixels
                            i+=2;
                            break;

                        case SPOT_TYPE_4X_DOT:
                            if (i > 0) position += 3; //skip three pixels
                            i+=3;
                            break;
                    }
                }
                //emit particle
                //set the particle source position:
                PartSys->sources[0].source.x = position * PS_P_RADIUS_1D;
                uint32_t partidx = PartSys->sprayEmit(PartSys->sources[0]);
                PartSys->particles[partidx].ttl = ttl;
                position++; //do the next pixel
            }
        }

        PartSys->update(); // update and render
    }

private:
    uint16_t aux0{};
};


#endif //WLED_DISABLE_PARTICLESYSTEM1D
