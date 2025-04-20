#pragma once
#ifndef WLED_DISABLE_PARTICLESYSTEM1D

#include "../../FX.h"
#include "../Effect.h"
#include "Particle1dEffect.h"

/*
Particle based Chase effect
Uses palette for particle color
by DedeHai (Damian Schneider)
*/
class ParticleChaseEffect : public BaseEffect<ParticleChaseEffect, Particle1dEffect<ParticleChaseEffect>> {
private:
    using Self = ParticleChaseEffect;
    using Base = BaseEffect<Self, Particle1dEffect<Self>>;

public:
    static constexpr const char metaData[] PROGMEM = "PS Chase@!,Density,Size,Hue,Blur,,,Position Color;,!;!;1;pal=11,sx=50,c2=5,c3=0";
    static constexpr const uint8_t effectId = FX_MODE_PSCHASE;

    using Base::Base;

    bool init(const EffectCoordinate& coordinate) {
        return Base::init(coordinate, 1, 255, true);
    }

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        // Particle System settings
        PartSys.updateSystem(coordinate.width); // update system properties (dimensions and data pointers)
        PartSys.setColorByPosition(SEGMENT.check3);
        PartSys.setMotionBlur(8 + ((SEGMENT.custom3) << 3)); // anable motion blur

        uint32_t settingssum = SEGMENT.speed + SEGMENT.intensity + SEGMENT.custom1 + SEGMENT.custom2 + SEGMENT.check1 + SEGMENT.check2 + SEGMENT.check3 + PartSys.getAvailableParticles(); // note: getAvailableParticles is used to enforce update during transitions
        if (aux0 != settingssum) { // settings changed changed, update
            uint32_t numParticles = map(SEGMENT.intensity, 0, 255, 2, 255 / (1 + (SEGMENT.custom1 >> 6))); // depends on intensity and particle size (custom1)
            if (numParticles == 0) numParticles = 1; // minimum 1 particle
            PartSys.setUsedParticles(numParticles);
            step = (PartSys.maxX + (PS_P_RADIUS_1D << 5)) / PartSys.usedParticles; // spacing between particles
            for (int32_t i = 0; i < (int32_t)PartSys.usedParticles; i++) {
                PartSys.advPartProps[i].sat = 255;
                PartSys.particles[i].x = (i - 1) * step; // distribute evenly (starts out of frame for i=0)
                PartSys.particles[i].vx =  SEGMENT.speed >> 1;
                PartSys.advPartProps[i].size = SEGMENT.custom1;
                if (SEGMENT.custom2 < 255)
                    PartSys.particles[i].hue = (i * (SEGMENT.custom2 << 3)) / PartSys.usedParticles; // gradient distribution
                else
                    PartSys.particles[i].hue = hw_random16();
            }
            aux0 = settingssum;
        }

        int32_t huestep = (((uint32_t)SEGMENT.custom2 << 19) / PartSys.usedParticles) >> 16; // hue increment

        // wrap around (cannot use particle system wrap if distributing colors manually, it also wraps rendering which does not look good)
        for (int32_t i = (int32_t)PartSys.usedParticles - 1; i >= 0; i--) { // check from the back, last particle wraps first, multiple particles can overrun per frame
            if (PartSys.particles[i].x > PartSys.maxX + PS_P_RADIUS_1D + PartSys.advPartProps[i].size) { // wrap it around
                uint32_t nextindex = (i + 1) % PartSys.usedParticles;
                PartSys.particles[i].x =  PartSys.particles[nextindex].x - (int)step;
                if (SEGMENT.custom2 < 255)
                    PartSys.particles[i].hue = PartSys.particles[nextindex].hue - huestep;
                else
                    PartSys.particles[i].hue = hw_random16();
            }
            PartSys.particles[i].ttl = 300; // reset ttl, cannot use perpetual because memmanager can change pointer at any time
        }

        PartSys.setParticleSize(SEGMENT.custom1); // if custom1 == 0 this sets rendering size to one pixel
        PartSys.update(buffer); // update and render
        return true;
    }

private:
    uint32_t step{};
    uint16_t aux0{0xFFFF};
};


#endif //WLED_DISABLE_PARTICLESYSTEM1D
