#pragma once
#ifndef WLED_DISABLE_PARTICLESYSTEM2D

#include "../../FX.h"
#include "../../FXparticleSystem.h"
#include "../BufferedEffect.h"
#include "../Effect.h"

template<typename T>
class Particle2dEffect : public BufferedEffect<EffectDimensionality::d2> {
private:
    using Self = Particle2dEffect;
    using Base = BufferedEffect<EffectDimensionality::d2>;

public:
    explicit Particle2dEffect(const EffectInformation& ei)
        : Base{ei, false},
          PartSys{ei.effectId}
    {
    }

    inline bool init(const uint32_t requestedsources, const bool advanced, const bool sizecontrol) {
        return PartSys.init(Segment::getEffectWidth<dimensionality>(), Segment::getEffectHeight<dimensionality>(), requestedsources, advanced, sizecontrol);
    }

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }
        if (PartSys.isInitialized()) {
            return true;
        }
        return static_cast<T*>(this)->init();
    }

protected:
    ParticleSystem2D PartSys;
};


#endif //WLED_DISABLE_PARTICLESYSTEM2D
