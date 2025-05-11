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

    inline bool init(const EffectCoordinate& coordinate, const uint32_t requestedsources, const bool advanced, const bool sizecontrol) {
        return PartSys.init(coordinate.width, coordinate.height, requestedsources, advanced, sizecontrol);
    }

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }
        if (PartSys.isInitialized()) {
            return true;
        }
        return static_cast<T*>(this)->init(coordinate);
    }

protected:
    ParticleSystem2D PartSys;
};


#endif //WLED_DISABLE_PARTICLESYSTEM2D
