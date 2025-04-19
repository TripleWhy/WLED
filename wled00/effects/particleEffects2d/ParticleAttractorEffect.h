#pragma once
#ifndef WLED_DISABLE_2D
#ifndef WLED_DISABLE_PARTICLESYSTEM2D

#include "../../FX.h"
#include "../Effect.h"
#include "Particle2dEffect.h"

/*
  Particle Attractor, a particle attractor sits in the matrix center, a spray bounces around and seeds particles
  uses inverse square law like in planetary motion
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
class ParticleAttractorEffect : public BaseEffect<ParticleAttractorEffect, Particle2dEffect<ParticleAttractorEffect>> {
private:
    using Self = ParticleAttractorEffect;
    using Base = BaseEffect<Self, Particle2dEffect<Self>>;

public:
    static constexpr const char metaData[] PROGMEM = "PS Attractor@Mass,Particles,Size,Collide,Friction,AgeColor,Move,Swallow;;!;2;pal=9,sx=100,ix=82,c1=2,c2=0";
    static constexpr const uint8_t effectId = FX_MODE_PARTICLEATTRACTOR;

    using Base::Base;

    bool init() {
        if (!Base::init(1, true, false)) {
            return false;
        }

        PartSys.sources[0].source.hue = hw_random16();
        PartSys.sources[0].source.vx = -7; // will collied with wall and get random bounce direction
        PartSys.sources[0].sourceFlags.collide = true; // seeded particles will collide
        PartSys.sources[0].sourceFlags.perpetual = true; //source does not age
        #ifdef ESP8266
        PartSys.sources[0].maxLife = 200; // lifetime in frames (ESP8266 has less particles)
        PartSys.sources[0].minLife = 30;
        #else
        PartSys.sources[0].maxLife = 350; // lifetime in frames
        PartSys.sources[0].minLife = 50;
        #endif
        PartSys.sources[0].var = 4; // emiting variation
        PartSys.setWallHardness(255);  //bounce forever
        PartSys.setWallRoughness(200); //randomize wall bounce
        return true;
    }

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        PSsettings2D sourcesettings;
        sourcesettings.asByte = 0b00001100; // PS settings for bounceY, bounceY used for source movement (it always bounces whereas particles do not)
        PSparticleFlags attractorFlags;
        attractorFlags.asByte = 0; // no flags set

        // Particle System settings
        PartSys.updateSystem(coordinate.width, coordinate.height); // update system properties (dimensions and data pointers)

        PartSys.setColorByAge(SEGMENT.check1);
        PartSys.setParticleSize(SEGMENT.custom1 >> 1); //set size globally
        PartSys.setUsedParticles(map(SEGMENT.intensity, 0, 255, 25, 190));

        if (SEGMENT.custom2 > 0) // collisions enabled
            PartSys.enableParticleCollisions(true, map(SEGMENT.custom2, 1, 255, 120, 255)); // enable collisions and set particle collision hardness
        else
            PartSys.enableParticleCollisions(false);

        if (SEGMENT.call == 0) {
            attractor.vx = PartSys.sources[0].source.vy; // set to spray movemement but reverse x and y
            attractor.vy = PartSys.sources[0].source.vx;
        }

        // set attractor properties
        attractor.ttl = 100; // never dies
        if (SEGMENT.check2) {
            if ((SEGMENT.call % 3) == 0) // move slowly
                PartSys.particleMoveUpdate(attractor, attractorFlags, &sourcesettings); // move the attractor
        }
        else {
            attractor.x = PartSys.maxX >> 1; // set to center
            attractor.y = PartSys.maxY >> 1;
        }

        if (SEGMENT.call % 5 == 0)
            PartSys.sources[0].source.hue++;

        aux0 += 256; // emitting angle, one full turn in 255 frames (0xFFFF is 360°)
        if (SEGMENT.call % 2 == 0) // alternate direction of emit
            PartSys.angleEmit(PartSys.sources[0], aux0, 12);
        else
            PartSys.angleEmit(PartSys.sources[0], aux0 + 0x7FFF, 12); // emit at 180° as well
        // apply force
        uint32_t strength = SEGMENT.speed;
        #ifdef USERMOD_AUDIOREACTIVE
        um_data_t *um_data;
        if (UsermodManager::getUMData(&um_data, USERMOD_ID_AUDIOREACTIVE)) { // AR active, do not use simulated data
            uint32_t volumeSmth = (uint32_t)(*(float*) um_data->u_data[0]); // 0-255
            strength = (SEGMENT.speed * volumeSmth) >> 8;
        }
        #endif
        for (uint32_t i = 0; i < PartSys.usedParticles; i++) {
            PartSys.pointAttractor(i, attractor, strength, SEGMENT.check3);
        }


        if (SEGMENT.call % (33 - SEGMENT.custom3) == 0)
            PartSys.applyFriction(2);
        PartSys.particleMoveUpdate(PartSys.sources[0].source, PartSys.sources[0].sourceFlags, &sourcesettings); // move the source
        PartSys.update(buffer); // update and render
        return true;
    }

private:
    PSparticle attractor{};
    uint16_t aux0{};
};


#endif //WLED_DISABLE_2D
#endif //WLED_DISABLE_PARTICLESYSTEM2D
