#pragma once
#ifndef WLED_DISABLE_PARTICLESYSTEM1D

#include "../../FX.h"
#include "../../FXparticleSystem.h"
#include "../BufferedEffect.h"
#include "../Effect.h"

template<typename T>
class Particle1dEffect : public BufferedEffect<EffectDimensionality::d1> {
private:
    using Self = Particle1dEffect;
    using Base = BufferedEffect<EffectDimensionality::d1>;

public:
    explicit Particle1dEffect(const EffectInformation& ei)
        : Base{ei, false},
          PartSys{ei.effectId}
    {
    }

    inline bool init(const uint32_t requestedsources, const uint8_t fractionofparticles, const bool advanced) {
        return PartSys.init(Segment::getEffectWidth<dimensionality>(), requestedsources, fractionofparticles, advanced);
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
    ParticleSystem1D PartSys;
};


#endif //WLED_DISABLE_PARTICLESYSTEM1D
