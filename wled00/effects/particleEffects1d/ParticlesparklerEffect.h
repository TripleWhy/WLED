#pragma once
#ifndef WLED_DISABLE_PARTICLESYSTEM1D

#include "../../FX.h"
#include "../../FXparticleSystem.h"
#include "../BufferedEffect.h"
#include "../Effect.h"

/*
  Particle based Sparkle effect
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
class ParticlesparklerEffect : public BaseEffect<ParticlesparklerEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = ParticlesparklerEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "PS Sparkler@Move,!,Saturation,Blur,Sparklers,Slide,Bounce,Large;,!;!;1;pal=0,sx=255,c1=0,c2=0,c3=6";
    static constexpr const uint8_t effectId = FX_MODE_PS_SPARKLER;

    explicit ParticlesparklerEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        ParticleSystem1D *PartSys = nullptr;
        uint32_t numSparklers;
        PSsettings1D sparklersettings;
        sparklersettings.asByte = 0; // PS settings for sparkler (set below)

        if (SEGMENT.call == 0) { // initialization
            if (!initParticleSystem1D(PartSys, 16, 128 ,0, true)) // init, no additional data needed
                return mode_static(); // allocation failed or is single pixel
        } else
            PartSys = reinterpret_cast<ParticleSystem1D *>(SEGENV.data); // if not first call, just set the pointer to the PS
        if (PartSys == nullptr)
            return mode_static(); // something went wrong, no data!

        // Particle System settings
        PartSys->updateSystem(); // update system properties (dimensions and data pointers)

        sparklersettings.wrap = !SEGMENT.check2;
        sparklersettings.bounce = SEGMENT.check2; // note: bounce always takes priority over wrap

        numSparklers = PartSys->numSources;
        PartSys->setMotionBlur(SEGMENT.custom2); // anable motion blur/overlay
        //PartSys->setSmearBlur(SEGMENT.custom2); // anable smearing blur

        for (uint32_t i = 0; i < numSparklers; i++) {
            PartSys->sources[i].source.hue = hw_random16();
            PartSys->sources[i].var = 0; // sparks stationary
            PartSys->sources[i].minLife = 150 + SEGMENT.intensity;
            PartSys->sources[i].maxLife = 250 + (SEGMENT.intensity << 1);
            uint32_t speed = SEGMENT.speed >> 1;
            if (SEGMENT.check1) // sparks move (slide option)
                PartSys->sources[i].var = SEGMENT.intensity >> 3;
            PartSys->sources[i].source.vx = speed; // update speed, do not change direction
            PartSys->sources[i].source.ttl = 400; // replenish its life (setting it perpetual uses more code)
            PartSys->sources[i].sat = SEGMENT.custom1; // color saturation
            PartSys->sources[i].size = SEGMENT.check3 ? 120 : 0;
            if (SEGMENT.speed == 255) // random position at highest speed setting
                PartSys->sources[i].source.x = hw_random16(PartSys->maxX);
            else
                PartSys->particleMoveUpdate(PartSys->sources[i].source, PartSys->sources[i].sourceFlags, &sparklersettings); //move sparkler
        }

        numSparklers = min(1 + (SEGMENT.custom3 >> 1), (int)numSparklers);  // set used sparklers, 1 to 16

        if (aux0 != SEGMENT.custom3) { //number of used sparklers changed, redistribute
            for (uint32_t i = 1; i < numSparklers; i++) {
                PartSys->sources[i].source.x = (PartSys->sources[0].source.x + (PartSys->maxX / numSparklers) * i ) % PartSys->maxX; //distribute evenly
            }
        }
        aux0 = SEGMENT.custom3;

        for (uint32_t i = 0; i < numSparklers; i++) {
            if (hw_random()  % (((271 - SEGMENT.intensity) >> 4)) == 0)
                PartSys->sprayEmit(PartSys->sources[i]); //emit a particle
        }

        PartSys->update(); // update and render

        for (uint32_t i = 0; i < PartSys->usedParticles; i++) {
            if (PartSys->particles[i].ttl > (64 - (SEGMENT.intensity >> 2))) PartSys->particles[i].ttl -= (64 - (SEGMENT.intensity >> 2)); //ttl is linked to brightness, this allows to use higher brightness but still a short spark lifespan
            else PartSys->particles[i].ttl = 0;
        }
        return true;
    }

private:
    uint16_t aux0{};
};


#endif //WLED_DISABLE_PARTICLESYSTEM1D
