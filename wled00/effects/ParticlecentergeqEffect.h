#pragma once
#ifndef WLED_DISABLE_2D
#ifndef WLED_DISABLE_PARTICLESYSTEM2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"
#include "effectUtils.h"

/*
  Particle rotating GEQ
  Particles sprayed from center with rotating spray
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
#define NUMBEROFSOURCES 16
class ParticlecentergeqEffect : public BaseEffect<ParticlecentergeqEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = ParticlecentergeqEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char* const metaData = "PS GEQ Nova@Speed,Intensity,Rotation Speed,Color Change,Nozzle,,Direction;;!;2f;pal=13,ix=180,c1=0,c2=0,c3=8";
    static constexpr const uint8_t effectId = FX_MODE_PARTICLECIRCULARGEQ;

    explicit ParticlecentergeqEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        ParticleSystem2D *PartSys = nullptr;
        uint8_t numSprays;
        uint32_t i;

        if (SEGMENT.call == 0) { // initialization
            if (!initParticleSystem2D(PartSys, NUMBEROFSOURCES))  // init, request 16 sources
                return mode_static(); // allocation failed or not 2D

            numSprays = min(PartSys->numSources, (uint32_t)NUMBEROFSOURCES);
            for (i = 0; i < numSprays; i++) {
                PartSys->sources[i].source.x = (PartSys->maxX + 1) >> 1; // center
                PartSys->sources[i].source.y = (PartSys->maxY + 1) >> 1; // center
                PartSys->sources[i].source.hue = i * 16; // even color distribution
                PartSys->sources[i].maxLife = 400;
                PartSys->sources[i].minLife = 200;
            }
            PartSys->setKillOutOfBounds(true);
        }
        else
            PartSys = reinterpret_cast<ParticleSystem2D *>(SEGENV.data); // if not first call, just set the pointer to the PS

        if (PartSys == nullptr)
            return mode_static(); // something went wrong, no data!

        PartSys->updateSystem(); // update system properties (dimensions and data pointers)
        numSprays = min(PartSys->numSources, (uint32_t)NUMBEROFSOURCES);

        um_data_t *um_data = getAudioData();
        uint8_t *fftResult = (uint8_t *)um_data->u_data[2]; // 16 bins with FFT data, log mapped already, each band contains frequency amplitude 0-255
        uint32_t threshold = 300 - SEGMENT.intensity;

        if (SEGMENT.check2)
            aux0 += SEGMENT.custom1 << 2;
        else
            aux0 -= SEGMENT.custom1 << 2;

        uint16_t angleoffset = (uint16_t)0xFFFF / (uint16_t)numSprays;
        uint32_t j = hw_random16(numSprays); // start with random spray so all get a chance to emit a particle if maximum number of particles alive is reached.
        for (i = 0; i < numSprays; i++) {
            if (SEGMENT.call % (32 - (SEGMENT.custom2 >> 3)) == 0 && SEGMENT.custom2 > 0)
                PartSys->sources[j].source.hue += 1 + (SEGMENT.custom2 >> 4);

            PartSys->sources[j].var = SEGMENT.custom3 >> 2;
            int8_t emitspeed = 5 + (((uint32_t)fftResult[j] * ((uint32_t)SEGMENT.speed + 20)) >> 10); // emit speed according to loudness of band
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
                PartSys->angleEmit(PartSys->sources[j], emitangle, emitspeed);

            j = (j + 1) % numSprays;
        }
        PartSys->update(); // update and render
    }

private:
    uint16_t aux0{};
};


#endif //WLED_DISABLE_2D
#endif //WLED_DISABLE_PARTICLESYSTEM2D
