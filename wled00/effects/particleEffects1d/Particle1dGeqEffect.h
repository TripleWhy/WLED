#pragma once
#ifndef WLED_DISABLE_PARTICLESYSTEM1D

#include "../../FX.h"
#include "../../FXparticleSystem.h"
#include "../BufferedEffect.h"
#include "../Effect.h"

/*
  Particle based 1D GEQ effect, each frequency bin gets an emitter, distributed over the strip
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
class Particle1dGeqEffect : public BaseEffect<Particle1dGeqEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = Particle1dGeqEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "PS GEQ 1D@Speed,!,Size,Blur,,,,;,!;!;1f;pal=0,sx=50,ix=200,c1=0,c2=0,c3=0,o1=1,o2=1";
    static constexpr const uint8_t effectId = FX_MODE_PS1DGEQ;

    explicit Particle1dGeqEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        ParticleSystem1D *PartSys = nullptr;
        uint32_t numSources;
        uint32_t i;

        if (SEGMENT.call == 0) { // initialization
            if (!initParticleSystem1D(PartSys, 16, 255, 0, true)) // init, no additional data needed
                return mode_static(); // allocation failed or is single pixel
        }
        else
            PartSys = reinterpret_cast<ParticleSystem1D *>(SEGENV.data); // if not first call, just set the pointer to the PS
        if (PartSys == nullptr)
            return mode_static(); // something went wrong, no data!

        // Particle System settings
        PartSys->updateSystem(); // update system properties (dimensions and data pointers)
        numSources = PartSys->numSources;
        PartSys->setMotionBlur(SEGMENT.custom2); // anable motion blur

        uint32_t spacing = PartSys->maxX / numSources;
        for (i = 0; i < numSources; i++) {
            PartSys->sources[i].source.hue = i * 16; // hw_random16();   //TODO: make adjustable, maybe even colorcycle?
            PartSys->sources[i].var = SEGMENT.speed >> 2;
            PartSys->sources[i].minLife = 180 + (SEGMENT.intensity >> 1);
            PartSys->sources[i].maxLife = 240 + SEGMENT.intensity;
            PartSys->sources[i].sat = 255;
            PartSys->sources[i].size = SEGMENT.custom1;
            PartSys->setParticleSize(SEGMENT.custom1);
            PartSys->sources[i].source.x = (spacing >> 1) + spacing * i; //distribute evenly
        }

        for (i = 0; i < PartSys->usedParticles; i++) {
            if (PartSys->particles[i].ttl > 20) PartSys->particles[i].ttl -= 20; //ttl is linked to brightness, this allows to use higher brightness but still a short lifespan
            else PartSys->particles[i].ttl = 0;
        }

        um_data_t *um_data = getAudioData();
        uint8_t *fftResult = (uint8_t *)um_data->u_data[2]; // 16 bins with FFT data, log mapped already, each band contains frequency amplitude 0-255

        //map the bands into 16 positions on x axis, emit some particles according to frequency loudness
        i = 0;
        uint32_t bin = hw_random16(numSources); //current bin , start with random one to distribute available particles fairly
        uint32_t threshold = 300 - SEGMENT.intensity;

        for (i = 0; i < numSources; i++) {
            bin++;
            bin = bin % numSources;
            uint32_t emitparticle = 0;
            // uint8_t emitspeed = ((uint32_t)fftResult[bin] * (uint32_t)SEGMENT.speed) >> 10; // emit speed according to loudness of band (127 max!)
            if (fftResult[bin] > threshold) {
                emitparticle = 1;
            }
            else if (fftResult[bin] > 0) { // band has low volue
                uint32_t restvolume = ((threshold - fftResult[bin]) >> 2) + 2;
                if (hw_random() % restvolume == 0) {
                    emitparticle = 1;
                }
            }

            if (emitparticle)
                PartSys->sprayEmit(PartSys->sources[bin]);
        }
        //TODO: add color control?

        PartSys->update(); // update and render
        return true;
    }

private:
};


#endif //WLED_DISABLE_PARTICLESYSTEM1D
