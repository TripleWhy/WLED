#pragma once
#ifndef WLED_DISABLE_PARTICLESYSTEM1D

#include "../../FX.h"
#include "../Effect.h"
#include "Particle1dEffect.h"

/*
  Particle based Spray effect (like a volcano, possible replacement for popcorn)
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
class Particle1dSprayEffect : public BaseEffect<Particle1dSprayEffect, Particle1dEffect<Particle1dSprayEffect>> {
private:
    using Self = Particle1dSprayEffect;
    using Base = BaseEffect<Self, Particle1dEffect<Self>>;

public:
    static constexpr const char metaData[] PROGMEM = "PS Spray 1D@Speed(+/-),!,Position,Blur,Gravity(+/-),AgeColor,Bounce,Position Color;,!;!;1;sx=200,ix=220,c1=0,c2=0";
    static constexpr const uint8_t effectId = FX_MODE_PS1DSPRAY;

    using Base::Base;

    bool init(const EffectCoordinate& coordinate) {
        if (!Base::init(coordinate, 1, 255, false)) {
            return false;
        }

        PartSys.setKillOutOfBounds(true);
        PartSys.setWallHardness(150);
        PartSys.setParticleSize(1);
        return true;
    }

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        // Particle System settings
        PartSys.updateSystem(coordinate.width); // update system properties (dimensions and data pointers)
        PartSys.setBounce(parameters.check2);
        PartSys.setMotionBlur(parameters.custom2); // anable motion blur
        int32_t gravity = -((int32_t)parameters.custom3 - 16);  // gravity setting, 0-15 is positive (down), 17 - 31 is negative (up)
        PartSys.setGravity(abs(gravity)); // use reversgrav setting to invert gravity (for proper 'floor' and out of bounce handling)

        PartSys.sources[0].source.hue = aux0; // hw_random16();
        PartSys.sources[0].var = 20;
        PartSys.sources[0].minLife = 200;
        PartSys.sources[0].maxLife = 400;
        PartSys.sources[0].source.x = map(parameters.custom1, 0 , 255, 0, PartSys.maxX); // spray position
        PartSys.sources[0].v = map(parameters.speed, 0 , 255, -127 + PartSys.sources[0].var, 127 - PartSys.sources[0].var); // particle emit speed
        PartSys.sources[0].sourceFlags.reversegrav = gravity < 0 ? true : false;

        if (hw_random()  % (1 + ((255 - parameters.intensity) >> 3)) == 0) {
            PartSys.sprayEmit(PartSys.sources[0]); // emit a particle
            aux0++; // increment hue
        }

        //update color settings
        PartSys.setColorByAge(parameters.check1); // overruled by 'color by position'
        PartSys.setColorByPosition(parameters.check3);
        for (uint i = 0; i < PartSys.usedParticles; i++) {
            PartSys.particleFlags[i].reversegrav = PartSys.sources[0].sourceFlags.reversegrav; // update gravity direction
        }
        PartSys.update(buffer); // update and render
        return true;
    }

private:
    uint16_t aux0{};
};


#endif //WLED_DISABLE_PARTICLESYSTEM1D
