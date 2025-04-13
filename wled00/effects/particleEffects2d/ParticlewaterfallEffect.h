#pragma once
#ifndef WLED_DISABLE_2D
#ifndef WLED_DISABLE_PARTICLESYSTEM2D

#include "../../FX.h"
#include "../../FXparticleSystem.h"
#include "../BufferedEffect.h"
#include "../Effect.h"

/*
  Particle Waterfall
  Uses palette for particle color, spray source at top emitting particles, many config options
  by DedeHai (Damian Schneider)
*/
class ParticlewaterfallEffect : public BaseEffect<ParticlewaterfallEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = ParticlewaterfallEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "PS Waterfall@Speed,Intensity,Variation,Collide,Position,Cylinder,Walls,Ground;;!;2;pal=9,sx=15,ix=200,c1=32,c2=160,o3=1";
    static constexpr const uint8_t effectId = FX_MODE_PARTICLEWATERFALL;

    explicit ParticlewaterfallEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        ParticleSystem2D *PartSys = nullptr;
        uint8_t numSprays;
        uint32_t i = 0;

        if (SEGMENT.call == 0) { // initialization TODO: make this a PSinit function, this is needed in every particle FX but first, get this working.
            if (!initParticleSystem2D(PartSys, 12)) // init, request 12 sources, no additional data needed
                return mode_static(); // allocation failed or not 2D

            PartSys.setGravity();  // enable with default gforce
            PartSys.setKillOutOfBounds(true); // out of bounds particles dont return (except on top, taken care of by gravity setting)
            PartSys.setMotionBlur(190); // anable motion blur
            PartSys.setSmearBlur(30); // enable 2D blurring (smearing)
            for (i = 0; i < PartSys.numSources; i++) {
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
        }
        else
            PartSys = reinterpret_cast<ParticleSystem2D *>(SEGENV.data); // if not first call, just set the pointer to the PS
        if (PartSys == nullptr)
            return mode_static(); // something went wrong, no data! (TODO: ask how to handle this so it always works)

        // Particle System settings
        PartSys.updateSystem(); // update system properties (dimensions and data pointers)
        PartSys.setWrapX(SEGMENT.check1);   // cylinder
        PartSys.setBounceX(SEGMENT.check2); // walls
        PartSys.setBounceY(SEGMENT.check3); // ground
        PartSys.setWallHardness(SEGMENT.custom2);
        numSprays = min((int32_t)PartSys.numSources, max(PartSys.maxXpixel / 6, (int32_t)2)); // number of sprays depends on segment width
        if (SEGMENT.custom2 > 0) // collisions enabled
            PartSys.enableParticleCollisions(true, SEGMENT.custom2); // enable collisions and set particle collision hardness
        else {
            PartSys.enableParticleCollisions(false);
            PartSys.setWallHardness(120); // set hardness (for ground bounce) to fixed value if not using collisions
        }

        for (i = 0; i < numSprays; i++) {
                PartSys.sources[i].source.hue += 1 + hw_random16(SEGMENT.custom1>>1); // change hue of spray source
        }

        if (SEGMENT.call % (12 - (SEGMENT.intensity >> 5)) == 0 && SEGMENT.intensity > 0) { // every nth frame, emit particles, do not emit if intensity is zero
            for (i = 0; i < numSprays; i++) {
                PartSys.sources[i].vy = -SEGMENT.speed >> 3; // emitting speed, down
                //PartSys.sources[i].source.x = map(SEGMENT.custom3, 0, 31, 0, (PartSys.maxXpixel - numSprays * 2) * PS_P_RADIUS) + i * PS_P_RADIUS * 2; // emitter position
                PartSys.sources[i].source.x = map(SEGMENT.custom3, 0, 31, 0, (PartSys.maxXpixel - numSprays) * PS_P_RADIUS) + i * PS_P_RADIUS * 2; // emitter position
                PartSys.sources[i].source.y = PartSys.maxY + (PS_P_RADIUS * ((i<<2) + 4)); // source y position, few pixels above the top to increase spreading before entering the matrix
                PartSys.sources[i].var = (SEGMENT.custom1 >> 3); // emiting variation 0-32
                PartSys.sprayEmit(PartSys.sources[i]);
            }
        }

        if (SEGMENT.call % 20 == 0)
            PartSys.applyFriction(1); // add just a tiny amount of friction to help smooth things

        PartSys.update();   // update and render
        return true;
    }

private:
};


#endif //WLED_DISABLE_2D
#endif //WLED_DISABLE_PARTICLESYSTEM2D
