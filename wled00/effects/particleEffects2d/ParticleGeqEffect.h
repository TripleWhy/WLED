#pragma once
#ifndef WLED_DISABLE_2D
#ifndef WLED_DISABLE_PARTICLESYSTEM2D

#include "../../FX.h"
#include "../Effect.h"
#include "Particle2dEffect.h"

/*
  Particle base Graphical Equalizer
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
class ParticleGeqEffect : public BaseEffect<ParticleGeqEffect, Particle2dEffect<ParticleGeqEffect>> {
private:
    using Self = ParticleGeqEffect;
    using Base = BaseEffect<Self, Particle2dEffect<Self>>;

public:
    static constexpr const char metaData[] PROGMEM = "PS GEQ 2D@Speed,Intensity,Diverge,Bounce,Gravity,Cylinder,Walls,Floor;;!;2f;pal=0,sx=155,ix=200,c1=0";
    static constexpr const uint8_t effectId = FX_MODE_PARTICLESGEQ;

    using Base::Base;

    bool init(const EffectCoordinate& coordinate) {
        if (!Base::init(coordinate, 1, false, false)) {
            return false;
        }

        PartSys.setKillOutOfBounds(true);
        PartSys.setUsedParticles(170); // use 2/3 of available particles
        return true;
    }

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        uint32_t i;
        // set particle system properties
        PartSys.updateSystem(coordinate.width, coordinate.height); // update system properties (dimensions and data pointers)
        PartSys.setWrapX(parameters.check1);
        PartSys.setBounceX(parameters.check2);
        PartSys.setBounceY(parameters.check3);
        //PartSys.enableParticleCollisions(false);
        PartSys.setWallHardness(parameters.custom2);
        PartSys.setGravity(parameters.custom3 << 2); // set gravity strength

        um_data_t *um_data = getAudioData();
        uint8_t *fftResult = (uint8_t *)um_data->u_data[2]; // 16 bins with FFT data, log mapped already, each band contains frequency amplitude 0-255

        //map the bands into 16 positions on x axis, emit some particles according to frequency loudness
        i = 0;
        uint32_t binwidth = (PartSys.maxX + 1)>>4; //emit poisition variation for one bin (+/-) is equal to width/16 (for 16 bins)
        uint32_t threshold = 300 - parameters.intensity;
        uint32_t emitparticles = 0;

        for (uint32_t bin = 0; bin < 16; bin++) {
            uint32_t xposition = binwidth*bin + (binwidth>>1); // emit position according to frequency band
            uint8_t emitspeed = ((uint32_t)fftResult[bin] * (uint32_t)parameters.speed) >> 9; // emit speed according to loudness of band (127 max!)
            emitparticles = 0;

            if (fftResult[bin] > threshold) {
                emitparticles = 1;// + (fftResult[bin]>>6);
            }
            else if (fftResult[bin] > 0) { // band has low volue
                uint32_t restvolume = ((threshold - fftResult[bin])>>2) + 2;
                if (hw_random16() % restvolume == 0)
                    emitparticles = 1;
            }

            while (i < PartSys.usedParticles && emitparticles > 0) { // emit particles if there are any left, low frequencies take priority
                if (PartSys.particles[i].ttl == 0) { // find a dead particle
                    //set particle properties TODO: could also use the spray...
                    PartSys.particles[i].ttl = 20 + map(parameters.intensity, 0,255, emitspeed>>1, emitspeed + hw_random16(emitspeed)) ; // set particle alive, particle lifespan is in number of frames
                    PartSys.particles[i].x = xposition + hw_random16(binwidth) - (binwidth>>1); // position randomly, deviating half a bin width
                    PartSys.particles[i].y = PS_P_RADIUS; // start at the bottom (PS_P_RADIUS is minimum position a particle is fully in frame)
                    PartSys.particles[i].vx = hw_random16(parameters.custom1>>1)-(parameters.custom1>>2) ; //x-speed variation: +/- custom1/4
                    PartSys.particles[i].vy = emitspeed;
                    PartSys.particles[i].hue = (bin<<4) + hw_random16(17) - 8; // color from palette according to bin
                    emitparticles--;
                }
                i++;
            }
        }

        PartSys.update(buffer); // update and render
        return true;
    }

private:
};


#endif //WLED_DISABLE_2D
#endif //WLED_DISABLE_PARTICLESYSTEM2D
