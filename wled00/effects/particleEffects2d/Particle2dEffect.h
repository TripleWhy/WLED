#pragma once
#ifndef WLED_DISABLE_PARTICLESYSTEM2D

#include "../../FX.h"
#include "../../FXparticleSystem.h"
#include "../BufferedEffect.h"
#include "../Effect.h"

class Particle2dEffect : public BufferedEffect<EffectDimensionality::d2> {
private:
    using Self = Particle2dEffect;
    using Base = BufferedEffect<EffectDimensionality::d2>;

public:
    explicit Particle2dEffect(const EffectInformation& ei, const uint32_t requestedsources, const bool advanced, const bool sizecontrol)
        : Base{ei, false},
          PartSys{ei.effectId, Segment::getEffectWidth<dimensionality>(), Segment::getEffectHeight<dimensionality>(), requestedsources, advanced, sizecontrol}
    {
    }

protected:
    ParticleSystem2D PartSys;
};


#endif //WLED_DISABLE_PARTICLESYSTEM2D
