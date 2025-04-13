#pragma once
#ifndef WLED_DISABLE_PARTICLESYSTEM1D

#include "../../FX.h"
#include "../Effect.h"
#include "Particle1dEffect.h"

/*
  Particle Fireworks Starburst replacement (smoother rendering, more settings)
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
class ParticleStarburstEffect : public BaseEffect<ParticleStarburstEffect, Particle1dEffect> {
private:
    using Self = ParticleStarburstEffect;
    using Base = BaseEffect<Self, Particle1dEffect>;

public:
    static constexpr const char metaData[] PROGMEM = "PS Starburst@Chance,Fragments,Size,Blur,Cooling,Gravity,Colorful,Push;,!;!;1;pal=52,sx=150,ix=150,c1=120,c2=0,c3=21";
    static constexpr const uint8_t effectId = FX_MODE_PSSTARBURST;

    explicit ParticleStarburstEffect(const EffectInformation& ei)
        : Base{ei, 1, 200, true}
    {
        PartSys.setKillOutOfBounds(true);
        PartSys.enableParticleCollisions(true, 200);
        PartSys.sources[0].source.ttl = 1; // set initial stanby time
        PartSys.sources[0].sat = 0; // emitted particles start out white
    }

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        // Particle System settings
        PartSys.updateSystem(coordinate.width); // update system properties (dimensions and data pointers)
        PartSys.setMotionBlur(SEGMENT.custom2); // anable motion blur
        PartSys.setGravity(SEGMENT.check1 * 8); // enable gravity

        if (PartSys.sources[0].source.ttl-- == 0) { // stanby time elapsed TODO: make it a timer?
            uint32_t explosionsize = 4 + hw_random16(SEGMENT.intensity >> 2);
            PartSys.sources[0].source.hue = hw_random16();
            PartSys.sources[0].var = 10 + (explosionsize << 1);
            PartSys.sources[0].minLife = 250;
            PartSys.sources[0].maxLife = 300;
            PartSys.sources[0].source.x = hw_random(PartSys.maxX); //random explosion position
            PartSys.sources[0].source.ttl = 10 + hw_random16(255 - SEGMENT.speed);
            PartSys.sources[0].size = SEGMENT.custom1; // Fragment size
            PartSys.setParticleSize(SEGMENT.custom1); // enable advanced size rendering
            PartSys.sources[0].sourceFlags.collide = SEGMENT.check3;
            for (uint32_t e = 0; e < explosionsize; e++) { // emit particles
                if (SEGMENT.check2)
                    PartSys.sources[0].source.hue = hw_random16(); //random color for each particle
                PartSys.sprayEmit(PartSys.sources[0]); //emit a particle
            }
        }
        //shrink all particles
        for (uint32_t i = 0; i < PartSys.usedParticles; i++) {
            if (PartSys.advPartProps[i].size)
                PartSys.advPartProps[i].size--;
            if (PartSys.advPartProps[i].sat < 251)
                PartSys.advPartProps[i].sat += 1 + (SEGMENT.custom3 >> 2); //note: it should be >> 3, the >> 2 creates overflows resulting in blinking if custom3 > 27, which is a bonus feature
        }

        if (SEGMENT.call % 5 == 0) {
            PartSys.applyFriction(1); //slow down particles
        }

        PartSys.update(buffer); // update and render
        return true;
    }

private:
};


#endif //WLED_DISABLE_PARTICLESYSTEM1D
