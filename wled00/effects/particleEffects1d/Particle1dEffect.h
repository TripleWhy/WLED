#pragma once
#ifndef WLED_DISABLE_PARTICLESYSTEM1D

#include "../../FX.h"
#include "../../FXparticleSystem.h"
#include "../BufferedEffect.h"
#include "../Effect.h"

class Particle1dEffect : public BufferedEffect<EffectDimensionality::d1> {
private:
    using Self = Particle1dEffect;
    using Base = BufferedEffect<EffectDimensionality::d1>;

public:
    explicit Particle1dEffect(const EffectInformation& ei, const uint32_t requestedsources, const uint8_t fractionofparticles, const bool advanced)
        : Base{ei, false},
          PartSys{ei.effectId, Segment::getEffectWidth<dimensionality>(), requestedsources, fractionofparticles, advanced}
    {
    }

protected:
    ParticleSystem1D PartSys;
};


#endif //WLED_DISABLE_PARTICLESYSTEM1D
