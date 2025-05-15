#pragma once
#ifndef WLED_DISABLE_2D
#ifndef WLED_DISABLE_PARTICLESYSTEM2D

#include "../../FX.h"
#include "../Effect.h"
#include "Particle2dEffect.h"

/*
  Particle rotating GEQ
  Particles sprayed from center with rotating spray
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
class ParticleCenterGeqEffect : public BaseEffect<ParticleCenterGeqEffect, Particle2dEffect<ParticleCenterGeqEffect>> {
private:
    using Self = ParticleCenterGeqEffect;
    using Base = BaseEffect<Self, Particle2dEffect<Self>>;

    static constexpr uint32_t NUMBEROFSOURCES = 16;

public:
    static constexpr const char metaData[] PROGMEM = "PS GEQ Nova@Speed,Intensity,Rotation Speed,Color Change,Nozzle,,Direction;;!;2f;pal=13,ix=180,c1=0,c2=0,c3=8";
    static constexpr const uint8_t effectId = FX_MODE_PARTICLECENTERGEQ;

    using Base::Base;

    bool init(const EffectCoordinate& coordinate) {
        if (!Base::init(coordinate, NUMBEROFSOURCES, false, false)) {
            return false;
        }

        uint8_t const numSprays = min(PartSys.sources.size(), (uint32_t)NUMBEROFSOURCES);
        for (uint32_t i = 0; i < numSprays; i++) {
            PartSys.sources[i].source.x = (PartSys.maxX + 1) >> 1; // center
            PartSys.sources[i].source.y = (PartSys.maxY + 1) >> 1; // center
            PartSys.sources[i].source.hue = i * 16; // even color distribution
            PartSys.sources[i].maxLife = 400;
            PartSys.sources[i].minLife = 200;
        }
        PartSys.setKillOutOfBounds(true);
        return true;
    }

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        uint8_t numSprays;
        uint32_t i;

        PartSys.updateSystem(coordinate.width, coordinate.height); // update system properties (dimensions and data pointers)
        numSprays = min(PartSys.sources.size(), (uint32_t)NUMBEROFSOURCES);

        um_data_t *um_data = getAudioData();
        uint8_t *fftResult = (uint8_t *)um_data->u_data[2]; // 16 bins with FFT data, log mapped already, each band contains frequency amplitude 0-255
        uint32_t threshold = 300 - parameters.intensity;

        if (parameters.check2)
            aux0 += parameters.custom1 << 2;
        else
            aux0 -= parameters.custom1 << 2;

        uint16_t angleoffset = (uint16_t)0xFFFF / (uint16_t)numSprays;
        uint32_t j = hw_random16(numSprays); // start with random spray so all get a chance to emit a particle if maximum number of particles alive is reached.
        for (i = 0; i < numSprays; i++) {
            if (parameters.call % (32 - (parameters.custom2 >> 3)) == 0 && parameters.custom2 > 0)
                PartSys.sources[j].source.hue += 1 + (parameters.custom2 >> 4);

            PartSys.sources[j].var = parameters.custom3 >> 2;
            int8_t emitspeed = 5 + (((uint32_t)fftResult[j] * ((uint32_t)parameters.speed + 20)) >> 10); // emit speed according to loudness of band
            uint16_t emitangle = j * angleoffset + aux0;

            uint32_t emitparticles = 0;
            if (fftResult[j] > threshold)
                emitparticles = 1;
            else if (fftResult[j] > 0) { // band has low value
                uint32_t restvolume = ((threshold - fftResult[j]) >> 2) + 2;
                if (hw_random16() % restvolume == 0)
                    emitparticles = 1;
            }
            if (emitparticles)
                PartSys.angleEmit(PartSys.sources[j], emitangle, emitspeed);

            j = (j + 1) % numSprays;
        }
        PartSys.update(buffer, parameters); // update and render
        return true;
    }

private:
    uint16_t aux0{};
};


#endif //WLED_DISABLE_2D
#endif //WLED_DISABLE_PARTICLESYSTEM2D
