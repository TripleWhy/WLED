#pragma once
#ifndef WLED_DISABLE_2D
#ifndef WLED_DISABLE_PARTICLESYSTEM2D

#include "../../FX.h"
#include "../Effect.h"
#include "Particle2dEffect.h"

/*
  Particle Volcano
  Particles are sprayed from below, spray moves back and forth if option is set
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
class ParticleVolcanoEffect : public BaseEffect<ParticleVolcanoEffect, Particle2dEffect<ParticleVolcanoEffect>> {
private:
    using Self = ParticleVolcanoEffect;
    using Base = BaseEffect<Self, Particle2dEffect<Self>>;

    static constexpr uint32_t NUMBEROFSOURCES = 1;

public:
    static constexpr const char metaData[] PROGMEM = "PS Volcano@Speed,Intensity,Move,Bounce,Spread,AgeColor,Walls,Collide;;!;2;pal=35,sx=100,ix=190,c1=0,c2=160,c3=6,o1=1";
    static constexpr const uint8_t effectId = FX_MODE_PARTICLEVOLCANO;

    using Base::Base;

    bool init() {
        if (!Base::init(NUMBEROFSOURCES, false, false)) {
            return false;
        }

        PartSys.setBounceY(true);
        PartSys.setGravity(); // enable with default gforce
        PartSys.setKillOutOfBounds(true); // out of bounds particles dont return (except on top, taken care of by gravity setting)
        PartSys.setMotionBlur(230); // anable motion blur

        const uint8_t numSprays = min(PartSys.sources.size(), (uint32_t)NUMBEROFSOURCES); // number of sprays
        for (uint32_t i = 0; i < numSprays; i++) {
            PartSys.sources[i].source.hue = hw_random16();
            PartSys.sources[i].source.x = PartSys.maxX / (numSprays + 1) * (i + 1); // distribute evenly
            PartSys.sources[i].maxLife = 300; // lifetime in frames
            PartSys.sources[i].minLife = 250;
            PartSys.sources[i].sourceFlags.collide = true; // seeded particles will collide (if enabled)
            PartSys.sources[i].sourceFlags.perpetual = true; // source never dies
        }
        return true;
    }

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        PSsettings2D volcanosettings;
        volcanosettings.asByte = 0b00000100; // PS settings for volcano movement: bounceX is enabled
        uint8_t numSprays; // note: so far only one tested but more is possible
        uint32_t i = 0;

        numSprays = min(PartSys.sources.size(), (uint32_t)NUMBEROFSOURCES); // number of volcanoes

        // change source emitting color from time to time, emit one particle per spray
        if (SEGMENT.call % (11 - (SEGMENT.intensity / 25)) == 0) { // every nth frame, cycle color and emit particles (and update the sources)
            for (i = 0; i < numSprays; i++) {
                PartSys.sources[i].source.y = PS_P_RADIUS + 5; // reset to just above the lower edge that is allowed for bouncing particles, if zero, particles already 'bounce' at start and loose speed.
                PartSys.sources[i].source.vy = 0; //reset speed (so no extra particlesettin is required to keep the source 'afloat')
                PartSys.sources[i].source.hue++; // = hw_random16(); //change hue of spray source (note: random does not look good)
                PartSys.sources[i].source.vx = PartSys.sources[i].source.vx > 0 ? (SEGMENT.custom1 >> 2) : -(SEGMENT.custom1 >> 2); // set moving speed but keep the direction given by PS
                PartSys.sources[i].vy = SEGMENT.speed >> 2; // emitting speed (upwards)
                PartSys.sources[i].vx = 0;
                PartSys.sources[i].var = SEGMENT.custom3 >> 1; // emiting variation = nozzle size (custom 3 goes from 0-31)
                PartSys.sprayEmit(PartSys.sources[i]);
                PartSys.setWallHardness(255); // full hardness for source bounce
                PartSys.particleMoveUpdate(PartSys.sources[i].source, PartSys.sources[i].sourceFlags, &volcanosettings); //move the source
            }
        }

        // Particle System settings
        PartSys.updateSystem(coordinate.width, coordinate.height); // update system properties (dimensions and data pointers)
        PartSys.setColorByAge(SEGMENT.check1);
        PartSys.setBounceX(SEGMENT.check2);
        PartSys.setWallHardness(SEGMENT.custom2);

        if (SEGMENT.check3) // collisions enabled
            PartSys.enableParticleCollisions(true, SEGMENT.custom2); // enable collisions and set particle collision hardness
        else
            PartSys.enableParticleCollisions(false);

        PartSys.update(buffer); // update and render
        return true;
    }

private:
};


#endif //WLED_DISABLE_2D
#endif //WLED_DISABLE_PARTICLESYSTEM2D
