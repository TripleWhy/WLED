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
    static constexpr const char metaData[] PROGMEM = "PS Chase@!,Density,Size,Hue,Blur,Playful,,Position Color;,!;!;1;pal=11,sx=50,c2=5,c3=0";
    static constexpr const uint8_t effectId = FX_MODE_PSCHASE;

    using Base::Base;

    bool init(const EffectCoordinate& coordinate) {
        return Base::init(coordinate, 1, 255, true);
    }

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        // Particle System settings
        PartSys.updateSystem(coordinate.width); // update system properties (dimensions and data pointers)
        PartSys.setColorByPosition(parameters.check3);
        PartSys.setMotionBlur(7 + ((parameters.custom3) << 3)); // anable motion blur
        uint32_t numParticles = 1 + map(parameters.intensity, 0, 255, 2, 255 / (1 + (parameters.custom1 >> 6))); // depends on intensity and particle size (custom1), minimum 1
        numParticles = min(numParticles, PartSys.usedParticles); // limit to available particles
        int32_t huestep = 1 + ((((uint32_t)parameters.custom2 << 19) / numParticles) >> 16); // hue increment
        uint32_t settingssum = parameters.speed + parameters.intensity + parameters.custom1 + parameters.custom2 + parameters.check1 + parameters.check2 + parameters.check3 + PartSys.getAvailableParticles(); // note: getAvailableParticles is used to enforce update during transitions
        if (aux0 != settingssum) { // settings changed changed, update
            if (parameters.check1)
                step = PartSys.advPartProps[0].size / 2 + (PartSys.maxX / numParticles);
            else
                step = (PartSys.maxX + (PS_P_RADIUS_1D << 5)) / numParticles; // spacing between particles
            for (int32_t i = 0; i < (int32_t)PartSys.usedParticles; i++) {
                PartSys.advPartProps[i].sat = 255;
                PartSys.particles[i].x = (i - 1) * step; // distribute evenly (starts out of frame for i=0)
                PartSys.particles[i].vx =  parameters.speed >> 2;
                PartSys.advPartProps[i].size = parameters.custom1;
                if (parameters.custom2 < 255)
                    PartSys.particles[i].hue = i * huestep; // gradient distribution
                else
                    PartSys.particles[i].hue = hw_random16();
            }
            aux0 = settingssum;
        }

        if(parameters.check1) {
            huestep = 1 + (max((int)huestep, 3)  * ((int(sin16_t(strip.now * 3) + 32767))) >> 15); // changes gradient spread (scale hue step)
        }

        // wrap around (cannot use particle system wrap if distributing colors manually, it also wraps rendering which does not look good)
        for (int32_t i = (int32_t)PartSys.usedParticles - 1; i >= 0; i--) { // check from the back, last particle wraps first, multiple particles can overrun per frame
            if (PartSys.particles[i].x > PartSys.maxX + PS_P_RADIUS_1D + PartSys.advPartProps[i].size) { // wrap it around
                uint32_t nextindex = (i + 1) % PartSys.usedParticles;
                PartSys.particles[i].x = PartSys.particles[nextindex].x - (int)step;
                if(parameters.check1) // playful mode, vary size
                    PartSys.advPartProps[i].size = max(1 + (parameters.custom1 >> 1), ((int(sin16_t(strip.now << 1) + 32767)) >> 8)); // cycle size
                if (parameters.custom2 < 255)
                    PartSys.particles[i].hue = PartSys.particles[nextindex].hue - huestep;
                else
                    PartSys.particles[i].hue = hw_random16();
            }
            PartSys.particles[i].ttl = 300; // reset ttl, cannot use perpetual because memmanager can change pointer at any time
        }

        if (parameters.check1) { // playful mode, changes hue, size, speed, density dynamically
            if(stepdir == 0) stepdir = 1; // initialize directions
            if(huedir == 0) huedir = 1;
            if (step >= (PartSys.advPartProps[0].size + PS_P_RADIUS_1D * 4) + PartSys.maxX / numParticles)
                stepdir = -1; // increase density (decrease space between particles)
            else if (step <= (PartSys.advPartProps[0].size >> 1) + ((PartSys.maxX / numParticles)))
                stepdir = 1; // decrease density
            if (aux1 > 512)
                huedir = -1;
            else if (aux1 < 50)
                huedir = 1;
            if (parameters.call % (1024 / (1 + (parameters.speed >> 2))) == 0)
                aux1 += huedir;
            int8_t globalhuestep = 0; // global hue increment
            if (parameters.call % (1 + (int(sin16_t(strip.now) + 32767) >> 12))  == 0)
                globalhuestep = 2; // global hue change to add some color variation
            if ((parameters.call & 0x1F) == 0)
                step += stepdir; // change density
            for(int32_t i = 0; i < PartSys.usedParticles; i++) {
                PartSys.particles[i].hue -= globalhuestep; // shift global hue (both directions)
                PartSys.particles[i].vx = 1 + (parameters.speed >> 2) + ((int32_t(sin16_t(strip.now >> 1) + 32767) * (parameters.speed >> 2)) >> 16);
            }
        }

        PartSys.setParticleSize(parameters.custom1); // if custom1 == 0 this sets rendering size to one pixel
        PartSys.update(buffer, parameters); // update and render
        return true;
    }

private:
    int8_t huedir{};
    int8_t stepdir{};

    uint32_t step{};
    uint16_t aux0{0xFFFF};
    uint16_t aux1{};
};


#endif //WLED_DISABLE_PARTICLESYSTEM1D
