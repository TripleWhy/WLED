#pragma once
#ifndef WLED_DISABLE_PARTICLESYSTEM1D

#include "../../FX.h"
#include "../../FXparticleSystem.h"
#include "../BufferedEffect.h"
#include "../Effect.h"

/*
  Particle based Fire effect
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
class Particlefire1dEffect : public BaseEffect<Particlefire1dEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = Particlefire1dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "PS Fire 1D@!,!,Cooling,Blur;,!;!;1;pal=35,sx=100,ix=50,c1=80,c2=100,c3=28,o1=1,o2=1";
    static constexpr const uint8_t effectId = FX_MODE_PS_FIRE1D;

    explicit Particlefire1dEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        ParticleSystem1D *PartSys = nullptr;

        if (SEGMENT.call == 0) { // initialization
            if (!initParticleSystem1D(PartSys, 5)) // init
                return mode_static(); // allocation failed or is single pixel
            PartSys->setKillOutOfBounds(true);
            PartSys->setParticleSize(1);
        }
        else
            PartSys = reinterpret_cast<ParticleSystem1D *>(SEGENV.data); // if not first call, just set the pointer to the PS
        if (PartSys == nullptr)
            return mode_static(); // something went wrong, no data!

        // Particle System settings
        PartSys->updateSystem(); // update system properties (dimensions and data pointers)
        PartSys->setMotionBlur(128 + (SEGMENT.custom2 >> 1)); // enable motion blur
        PartSys->setColorByAge(true);
        uint32_t emitparticles = 1;
        uint32_t j = hw_random16();
        for (uint i = 0; i < 3; i++) { // 3 base flames TODO: check if this is ok or needs adjustments
            if (PartSys->sources[i].source.ttl > 50)
                PartSys->sources[i].source.ttl -= 10; // TODO: in 2D making the source fade out slow results in much smoother flames, need to check if it can be done the same
            else
                PartSys->sources[i].source.ttl = 100 + hw_random16(200);
        }
        for (uint i = 0; i < PartSys->numSources; i++) {
            j = (j + 1) % PartSys->numSources;
            PartSys->sources[j].source.x = 0;
            PartSys->sources[j].var = 2 + (SEGMENT.speed >> 4);
            // base flames
            if (j > 2) {
                PartSys->sources[j].minLife = 150 + SEGMENT.intensity + (j << 2); // TODO: in 2D, min life is maxlife/2 and that looks very nice
                PartSys->sources[j].maxLife = 200 + SEGMENT.intensity + (j << 3);
                PartSys->sources[j].v = (SEGMENT.speed >> (2 + (j << 1)));
                if (emitparticles) {
                    emitparticles--;
                    PartSys->sprayEmit(PartSys->sources[j]); // emit a particle
                }
            }
            else {
                PartSys->sources[j].minLife = PartSys->sources[j].source.ttl + SEGMENT.intensity; // TODO: in 2D, emitted particle ttl depends on source TTL, mimic here the same way? OR: change 2D to the same way it is done here and ditch special fire treatment in emit?
                PartSys->sources[j].maxLife = PartSys->sources[j].minLife + 50;
                PartSys->sources[j].v = SEGMENT.speed >> 2;
                if (SEGENV.call & 0x01) // every second frame
                    PartSys->sprayEmit(PartSys->sources[j]); // emit a particle
            }
        }

        for (uint i = 0; i < PartSys->usedParticles; i++) {
            PartSys->particles[i].x += PartSys->particles[i].ttl >> 7; // 'hot' particles are faster, apply some extra velocity
            if (PartSys->particles[i].ttl > 3 + ((255 - SEGMENT.custom1) >> 1))
                PartSys->particles[i].ttl -= map(SEGMENT.custom1, 0, 255, 1, 3); // age faster
        }

        PartSys->update(); // update and render
        return true;
    }

private:
};


#endif //WLED_DISABLE_PARTICLESYSTEM1D
