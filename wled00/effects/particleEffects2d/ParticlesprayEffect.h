#pragma once
#ifndef WLED_DISABLE_2D
#ifndef WLED_DISABLE_PARTICLESYSTEM2D

#include "../../FX.h"
#include "../../FXparticleSystem.h"
#include "../BufferedEffect.h"
#include "../Effect.h"

/*
  Particle Spray, just a particle spray with many parameters
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
class ParticlesprayEffect : public BaseEffect<ParticlesprayEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = ParticlesprayEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "PS Spray@Speed,!,Left/Right,Up/Down,Angle,Gravity,Cylinder/Square,Collide;;!;2v;pal=0,sx=150,ix=150,c1=220,c2=30,c3=21";
    static constexpr const uint8_t effectId = FX_MODE_PARTICLESPRAY;

    explicit ParticlesprayEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        ParticleSystem2D *PartSys = nullptr;
        const uint8_t hardness = 200; // collision hardness is fixed

        if (SEGMENT.call == 0) { // initialization
            if (!initParticleSystem2D(PartSys, 1)) // init, no additional data needed
                return mode_static(); // allocation failed or not 2D
            PartSys.setKillOutOfBounds(true); // out of bounds particles dont return (except on top, taken care of by gravity setting)
            PartSys.setBounceY(true);
            PartSys.setMotionBlur(200); // anable motion blur
            PartSys.setSmearBlur(10); // anable motion blur
            PartSys.sources[0].source.hue = hw_random16();
            PartSys.sources[0].sourceFlags.collide = true; // seeded particles will collide (if enabled)
            PartSys.sources[0].var = 3;
        }
        else
            PartSys = reinterpret_cast<ParticleSystem2D *>(SEGENV.data); // if not first call, just set the pointer to the PS

        if (PartSys == nullptr)
            return mode_static(); // something went wrong, no data!

        // Particle System settings
        PartSys.updateSystem(); // update system properties (dimensions and data pointers)
        PartSys.setBounceX(!SEGMENT.check2);
        PartSys.setWrapX(SEGMENT.check2);
        PartSys.setWallHardness(hardness);
        PartSys.setGravity(8 * SEGMENT.check1); // enable gravity if checked (8 is default strength)
        //numSprays = min(PartSys.numSources, (uint8_t)1); // number of sprays

        if (SEGMENT.check3) // collisions enabled
            PartSys.enableParticleCollisions(true, hardness); // enable collisions and set particle collision hardness
        else
            PartSys.enableParticleCollisions(false);

        //position according to sliders
        PartSys.sources[0].source.x = map(SEGMENT.custom1, 0, 255, 0, PartSys.maxX);
        PartSys.sources[0].source.y = map(SEGMENT.custom2, 0, 255, 0, PartSys.maxY);
        uint16_t angle = (256 - (((int32_t)SEGMENT.custom3 + 1) << 3)) << 8;

        #ifdef USERMOD_AUDIOREACTIVE
        um_data_t *um_data;
        if (UsermodManager::getUMData(&um_data, USERMOD_ID_AUDIOREACTIVE)) { // get AR data, do not use simulated data
            uint32_t volumeSmth  = (uint8_t)(*(float*)   um_data->u_data[0]); //0 to 255
            uint32_t volumeRaw    = *(int16_t*)um_data->u_data[1]; //0 to 255
            PartSys.sources[0].minLife = 30;

            if (SEGMENT.call % 20 == 0 || SEGMENT.call % (11 - volumeSmth / 25) == 0) { // defines interval of particle emit
                PartSys.sources[0].maxLife = (volumeSmth >> 1) + (SEGMENT.intensity >> 1); // lifetime in frames
                PartSys.sources[0].var = 1 + ((volumeRaw * SEGMENT.speed)  >> 12);
                uint32_t emitspeed = (SEGMENT.speed >> 2) + (volumeRaw >> 3);
                PartSys.sources[0].source.hue += volumeSmth/30;
                PartSys.angleEmit(PartSys.sources[0], angle, emitspeed);
            }
        }
        else { //no AR data, fall back to normal mode
            // change source properties
            if (SEGMENT.call % (11 - (SEGMENT.intensity / 25)) == 0) { // every nth frame, cycle color and emit particles
                PartSys.sources[0].maxLife = 300 + SEGMENT.intensity; // lifetime in frames
                PartSys.sources[0].minLife = 150 + SEGMENT.intensity;
                PartSys.sources[0].source.hue++; // = hw_random16(); //change hue of spray source
                PartSys.angleEmit(PartSys.sources[0], angle, SEGMENT.speed >> 2);
            }
        }
        #else
        // change source properties
        if (SEGMENT.call % (11 - (SEGMENT.intensity / 25)) == 0) { // every nth frame, cycle color and emit particles
            PartSys.sources[0].maxLife = 300; // lifetime in frames. note: could be done in init part, but AR moderequires this to be dynamic
            PartSys.sources[0].minLife = 100;
            PartSys.sources[0].source.hue++; // = hw_random16(); //change hue of spray source
            // PartSys.sources[i].var = SEGMENT.custom3; // emiting variation = nozzle size (custom 3 goes from 0-32)
            // spray[j].source.hue = hw_random16(); //set random color for each particle (using palette)
            PartSys.angleEmit(PartSys.sources[0], angle, SEGMENT.speed >> 2);
        }
        #endif

        PartSys.update(); // update and render
        return true;
    }

private:
};


#endif //WLED_DISABLE_2D
#endif //WLED_DISABLE_PARTICLESYSTEM2D
