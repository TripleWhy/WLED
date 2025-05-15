#pragma once
#ifndef WLED_DISABLE_2D
#ifndef WLED_DISABLE_PARTICLESYSTEM2D

#include "../../FX.h"
#include "../Effect.h"
#include "Particle2dEffect.h"

/*
  Particle Waterfall
  Uses palette for particle color, spray source at top emitting particles, many config options
  by DedeHai (Damian Schneider)
*/
class ParticleWaterfallEffect : public BaseEffect<ParticleWaterfallEffect, Particle2dEffect<ParticleWaterfallEffect>> {
private:
    using Self = ParticleWaterfallEffect;
    using Base = BaseEffect<Self, Particle2dEffect<Self>>;

public:
    static constexpr const char metaData[] PROGMEM = "PS Waterfall@Speed,Intensity,Variation,Collide,Position,Cylinder,Walls,Ground;;!;2;pal=9,sx=15,ix=200,c1=32,c2=160,o3=1";
    static constexpr const uint8_t effectId = FX_MODE_PARTICLEWATERFALL;

    using Base::Base;

    bool init(const EffectCoordinate& coordinate) {
        if (!Base::init(coordinate, 12, false, false)) {
            return false;
        }

        PartSys.setGravity();  // enable with default gforce
        PartSys.setKillOutOfBounds(true); // out of bounds particles dont return (except on top, taken care of by gravity setting)
        PartSys.setMotionBlur(190); // anable motion blur
        PartSys.setSmearBlur(30); // enable 2D blurring (smearing)
        for (uint32_t i = 0; i < PartSys.sources.size(); i++) {
            PartSys.sources[i].source.hue = i*90;
            PartSys.sources[i].sourceFlags.collide = true; // seeded particles will collide
        #ifdef ESP8266
            PartSys.sources[i].maxLife = 250; // lifetime in frames (ESP8266 has less particles, make them short lived to keep the water flowing)
            PartSys.sources[i].minLife = 100;
        #else
            PartSys.sources[i].maxLife = 400; // lifetime in frames
            PartSys.sources[i].minLife = 150;
        #endif
        }
        return true;
    }

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        uint8_t numSprays;
        uint32_t i = 0;

        // Particle System settings
        PartSys.updateSystem(coordinate.width, coordinate.height); // update system properties (dimensions and data pointers)
        PartSys.setWrapX(parameters.check1);   // cylinder
        PartSys.setBounceX(parameters.check2); // walls
        PartSys.setBounceY(parameters.check3); // ground
        PartSys.setWallHardness(parameters.custom2);
        numSprays = min((int32_t)PartSys.sources.size(), max((int32_t)(coordinate.width - 1) / 6, (int32_t)2)); // number of sprays depends on segment width
        if (parameters.custom2 > 0) // collisions enabled
            PartSys.enableParticleCollisions(true, parameters.custom2); // enable collisions and set particle collision hardness
        else {
            PartSys.enableParticleCollisions(false);
            PartSys.setWallHardness(120); // set hardness (for ground bounce) to fixed value if not using collisions
        }

        for (i = 0; i < numSprays; i++) {
                PartSys.sources[i].source.hue += 1 + hw_random16(parameters.custom1>>1); // change hue of spray source
        }

        if (parameters.call % (12 - (parameters.intensity >> 5)) == 0 && parameters.intensity > 0) { // every nth frame, emit particles, do not emit if intensity is zero
            for (i = 0; i < numSprays; i++) {
                PartSys.sources[i].vy = -parameters.speed >> 3; // emitting speed, down
                //PartSys.sources[i].source.x = map(parameters.custom3, 0, 31, 0, ((coordinate.width - 1) - numSprays * 2) * PS_P_RADIUS) + i * PS_P_RADIUS * 2; // emitter position
                PartSys.sources[i].source.x = map(parameters.custom3, 0, 31, 0, ((coordinate.width - 1) - numSprays) * PS_P_RADIUS) + i * PS_P_RADIUS * 2; // emitter position
                PartSys.sources[i].source.y = PartSys.maxY + (PS_P_RADIUS * ((i<<2) + 4)); // source y position, few pixels above the top to increase spreading before entering the matrix
                PartSys.sources[i].var = (parameters.custom1 >> 3); // emiting variation 0-32
                PartSys.sprayEmit(PartSys.sources[i]);
            }
        }

        if (parameters.call % 20 == 0)
            PartSys.applyFriction(1); // add just a tiny amount of friction to help smooth things

        PartSys.update(buffer, parameters);   // update and render
        return true;
    }

private:
};


#endif //WLED_DISABLE_2D
#endif //WLED_DISABLE_PARTICLESYSTEM2D
