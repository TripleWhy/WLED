#pragma once
#ifndef WLED_DISABLE_PARTICLESYSTEM1D

#include "../../FX.h"
#include "../Effect.h"
#include "Particle1dEffect.h"

/*
  Particle based Sparkle effect
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
class ParticleSparklerEffect : public BaseEffect<ParticleSparklerEffect, Particle1dEffect<ParticleSparklerEffect>> {
private:
    using Self = ParticleSparklerEffect;
    using Base = BaseEffect<Self, Particle1dEffect<Self>>;

public:
    static constexpr const char metaData[] PROGMEM = "PS Sparkler@Move,!,Saturation,Blur,Sparklers,Slide,Bounce,Large;,!;!;1;pal=0,sx=255,c1=0,c2=0,c3=6";
    static constexpr const uint8_t effectId = FX_MODE_PSSPARKLER;

    using Base::Base;

    bool init(const EffectCoordinate& coordinate) {
        return Base::init(coordinate, 16, 128, true);
    }

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        uint32_t numSparklers;
        PSsettings1D sparklersettings;
        sparklersettings.asByte = 0; // PS settings for sparkler (set below)

        // Particle System settings
        PartSys.updateSystem(coordinate.width); // update system properties (dimensions and data pointers)

        sparklersettings.wrap = !parameters.check2;
        sparklersettings.bounce = parameters.check2; // note: bounce always takes priority over wrap

        numSparklers = PartSys.sources.size();
        PartSys.setMotionBlur(parameters.custom2); // anable motion blur/overlay
        //PartSys.setSmearBlur(parameters.custom2); // anable smearing blur

        for (uint32_t i = 0; i < numSparklers; i++) {
            PartSys.sources[i].source.hue = hw_random16();
            PartSys.sources[i].var = 0; // sparks stationary
            PartSys.sources[i].minLife = 150 + parameters.intensity;
            PartSys.sources[i].maxLife = 250 + (parameters.intensity << 1);
            uint32_t speed = parameters.speed >> 1;
            if (parameters.check1) // sparks move (slide option)
                PartSys.sources[i].var = parameters.intensity >> 3;
            PartSys.sources[i].source.vx = speed; // update speed, do not change direction
            PartSys.sources[i].source.ttl = 400; // replenish its life (setting it perpetual uses more code)
            PartSys.sources[i].sat = parameters.custom1; // color saturation
            PartSys.sources[i].size = parameters.check3 ? 120 : 0;
            if (parameters.speed == 255) // random position at highest speed setting
                PartSys.sources[i].source.x = hw_random16(PartSys.maxX);
            else
                PartSys.particleMoveUpdate(PartSys.sources[i].source, PartSys.sources[i].sourceFlags, &sparklersettings); //move sparkler
        }

        numSparklers = min(1 + (parameters.custom3 >> 1), (int)numSparklers);  // set used sparklers, 1 to 16

        if (aux0 != parameters.custom3) { //number of used sparklers changed, redistribute
            for (uint32_t i = 1; i < numSparklers; i++) {
                PartSys.sources[i].source.x = (PartSys.sources[0].source.x + (PartSys.maxX / numSparklers) * i ) % PartSys.maxX; //distribute evenly
            }
        }
        aux0 = parameters.custom3;

        for (uint32_t i = 0; i < numSparklers; i++) {
            if (hw_random()  % (((271 - parameters.intensity) >> 4)) == 0)
                PartSys.sprayEmit(PartSys.sources[i]); //emit a particle
        }

        PartSys.update(buffer, parameters); // update and render

        for (uint32_t i = 0; i < PartSys.usedParticles; i++) {
            if (PartSys.particles[i].ttl > (64 - (parameters.intensity >> 2))) PartSys.particles[i].ttl -= (64 - (parameters.intensity >> 2)); //ttl is linked to brightness, this allows to use higher brightness but still a short spark lifespan
            else PartSys.particles[i].ttl = 0;
        }
        return true;
    }

private:
    uint16_t aux0{};
};


#endif //WLED_DISABLE_PARTICLESYSTEM1D
