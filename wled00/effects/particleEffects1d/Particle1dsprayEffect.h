#pragma once
#ifndef WLED_DISABLE_PARTICLESYSTEM1D

#include "../../FX.h"
#include "../../FXparticleSystem.h"
#include "../BufferedEffect.h"
#include "../Effect.h"

/*
  Particle based Spray effect (like a volcano, possible replacement for popcorn)
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
class Particle1dsprayEffect : public BaseEffect<Particle1dsprayEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = Particle1dsprayEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "PS Spray 1D@Speed(+/-),!,Position,Blur,Gravity(+/-),AgeColor,Bounce,Position Color;,!;!;1;sx=200,ix=220,c1=0,c2=0";
    static constexpr const uint8_t effectId = FX_MODE_PS_1DSPRAY;

    explicit Particle1dsprayEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        ParticleSystem1D *PartSys = nullptr;

        if (SEGMENT.call == 0) { // initialization
            if (!initParticleSystem1D(PartSys, 1))
                return mode_static(); // allocation failed or is single pixel
            PartSys->setKillOutOfBounds(true);
            PartSys->setWallHardness(150);
            PartSys->setParticleSize(1);
        }
        else
            PartSys = reinterpret_cast<ParticleSystem1D *>(SEGENV.data); // if not first call, just set the pointer to the PS
        if (PartSys == nullptr)
            return mode_static(); // something went wrong, no data!

        // Particle System settings
        PartSys->updateSystem(); // update system properties (dimensions and data pointers)
        PartSys->setBounce(SEGMENT.check2);
        PartSys->setMotionBlur(SEGMENT.custom2); // anable motion blur
        int32_t gravity = -((int32_t)SEGMENT.custom3 - 16);  // gravity setting, 0-15 is positive (down), 17 - 31 is negative (up)
        PartSys->setGravity(abs(gravity)); // use reversgrav setting to invert gravity (for proper 'floor' and out of bounce handling)

        PartSys->sources[0].source.hue = SEGMENT.aux0; // hw_random16();
        PartSys->sources[0].var = 20;
        PartSys->sources[0].minLife = 200;
        PartSys->sources[0].maxLife = 400;
        PartSys->sources[0].source.x = map(SEGMENT.custom1, 0 , 255, 0, PartSys->maxX); // spray position
        PartSys->sources[0].v = map(SEGMENT.speed, 0 , 255, -127 + PartSys->sources[0].var, 127 - PartSys->sources[0].var); // particle emit speed
        PartSys->sources[0].sourceFlags.reversegrav = gravity < 0 ? true : false;

        if (hw_random()  % (1 + ((255 - SEGMENT.intensity) >> 3)) == 0) {
            PartSys->sprayEmit(PartSys->sources[0]); // emit a particle
            SEGMENT.aux0++; // increment hue
        }

        //update color settings
        PartSys->setColorByAge(SEGMENT.check1); // overruled by 'color by position'
        PartSys->setColorByPosition(SEGMENT.check3);
        for (uint i = 0; i < PartSys->usedParticles; i++) {
            PartSys->particleFlags[i].reversegrav = PartSys->sources[0].sourceFlags.reversegrav; // update gravity direction
        }
        PartSys->update(); // update and render
        return true;
    }

private:
};


#endif //WLED_DISABLE_PARTICLESYSTEM1D
