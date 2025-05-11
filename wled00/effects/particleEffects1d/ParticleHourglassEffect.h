#pragma once
#ifndef WLED_DISABLE_PARTICLESYSTEM1D

#include "../../FX.h"
#include "../Effect.h"
#include "Particle1dEffect.h"

/*
  Particle based Hourglass, particles falling at defined intervals
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
class ParticleHourglassEffect : public BaseEffect<ParticleHourglassEffect, Particle1dEffect<ParticleHourglassEffect>> {
private:
    using Self = ParticleHourglassEffect;
    using Base = BaseEffect<Self, Particle1dEffect<Self>>;

public:
    static constexpr const char metaData[] PROGMEM = "PS Hourglass@Interval,!,Color,Blur,Gravity,Colorflip,Start,Fast Reset;,!;!;1;pal=34,sx=50,ix=200,c1=140,c2=80,c3=4,o1=1,o2=1,o3=1";
    static constexpr const uint8_t effectId = FX_MODE_PSHOURGLASS;

    using Base::Base;

    bool init(const EffectCoordinate& coordinate) {
        if (!Base::init(coordinate, 0, 255, false)) {
            return false;
        }

        PartSys.setBounce(true);
        PartSys.setWallHardness(100);
        return true;
    }

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        constexpr int positionOffset = PS_P_RADIUS_1D / 2; // resting position offset

        // Particle System settings
        PartSys.updateSystem(coordinate.width); // update system properties (dimensions and data pointers)
        PartSys.setUsedParticles(1 + ((parameters.intensity * 255) >> 8));
        PartSys.setMotionBlur(parameters.custom2); // anable motion blur
        PartSys.setGravity(map(parameters.custom3, 0, 31, 1, 30));
        PartSys.enableParticleCollisions(true, 32); // hardness value found by experimentation on different settings

        uint32_t colormode = parameters.custom1 >> 5; // 0-7

        if ((parameters.intensity | (PartSys.getAvailableParticles() << 8)) != settingTracker) { // initialize, getAvailableParticles changes while in FX transition
            settingTracker = parameters.intensity | (PartSys.getAvailableParticles() << 8);
            for (uint32_t i = 0; i < PartSys.usedParticles; i++) {
                PartSys.particleFlags[i].reversegrav = true; // resting particles dont fall
                direction = 0; // down
                aux1 = 1; // initialize below
            }
            aux0 = PartSys.usedParticles - 1; // initial state, start with highest number particle
        }

        // calculate target position depending on direction
        auto calcTargetPos = [&](size_t i) {
            return PartSys.particleFlags[i].reversegrav ?
                    PartSys.maxX - i * PS_P_RADIUS_1D - positionOffset
                : (PartSys.usedParticles - i) * PS_P_RADIUS_1D - positionOffset;
        };

        for (uint32_t i = 0; i < PartSys.usedParticles; i++) { // check if particle reached target position after falling
            if (PartSys.particleFlags[i].fixed == false && abs(PartSys.particles[i].vx) < 5) {
                int32_t targetposition = calcTargetPos(i);
                bool closeToTarget = abs(targetposition - PartSys.particles[i].x) < 3 * PS_P_RADIUS_1D;
                if (closeToTarget) { // close to target and slow speed
                    PartSys.particles[i].x = targetposition; // set exact position
                    PartSys.particleFlags[i].fixed = true;   // pin particle
                }
            }
            if (colormode == 7)
                PartSys.setColorByPosition(true); // color fixed by position
            else {
                PartSys.setColorByPosition(false);
                uint8_t basehue = ((parameters.custom1 & 0x1F) << 3); // use 5 LSBs to select color
                switch(colormode) {
                    case 0: PartSys.particles[i].hue = 120; break; // fixed at 120, if flip is activated, this can make red and green (use palette 34)
                    case 1: PartSys.particles[i].hue = basehue; break; // fixed selectable color
                    case 2: // 2 colors inverleaved (same code as 3)
                    case 3: PartSys.particles[i].hue = ((parameters.custom1 & 0x1F) << 1) + (i % colormode)*74; break; // interleved colors (every 2 or 3 particles)
                    case 4: PartSys.particles[i].hue = basehue + (i * 255) / PartSys.usedParticles;  break; // gradient palette colors
                    case 5: PartSys.particles[i].hue = basehue + (i * 1024) / PartSys.usedParticles;  break; // multi gradient palette colors
                    case 6: PartSys.particles[i].hue = i + (strip.now >> 3);  break; // disco! moving color gradient
                    default: break;
                }
            }
            if (parameters.check1 && !PartSys.particleFlags[i].reversegrav) // flip color when fallen
                PartSys.particles[i].hue += 120;
        }
        // re-order particles in case collisions flipped particles (highest number index particle is on the "bottom")
        for (int i = 0; i < PartSys.usedParticles - 1; i++) {
            if (PartSys.particles[i].x < PartSys.particles[i+1].x && PartSys.particleFlags[i].fixed == false && PartSys.particleFlags[i+1].fixed == false) {
                std::swap(PartSys.particles[i].x, PartSys.particles[i+1].x);
            }
        }


        if (aux1 == 1) { // last countdown call before dropping starts, reset all particles
            for (uint32_t i = 0; i < PartSys.usedParticles; i++) {
                PartSys.particleFlags[i].collide = true;
                PartSys.particleFlags[i].perpetual = true;
                PartSys.particles[i].ttl = 260;
                PartSys.particles[i].x = calcTargetPos(i);
                PartSys.particleFlags[i].fixed = true;
            }
        }

        if (aux1 == 0) { // countdown passed, run
            if (strip.now >= step) { // drop a particle, do not drop more often than every second frame or particles tangle up quite badly
                // set next drop time
                if (parameters.check3 && direction) // fast reset
                    step = strip.now + 100; // drop one particle every 100ms
                else // normal interval
                    step = strip.now + max(20, parameters.speed * 20); // map speed slider from 0.1s to 5s
                if (aux0 < PartSys.usedParticles) {
                    PartSys.particleFlags[aux0].reversegrav = direction; // let this particle fall or rise
                    PartSys.particleFlags[aux0].fixed = false; // unpin
                }
                else { // overflow
                    direction = !(direction); // flip direction
                    aux1 = coordinate.width + 100; // set countdown
                }
                if (direction == false) // down, start dropping the highest number particle
                    aux0--; // next particle
                else
                    aux0++;
            }
        }
        else if (parameters.check2) // auto reset
            aux1--; // countdown

        PartSys.update(buffer); // update and render
        return true;
    }

private:
    uint32_t settingTracker{};
    bool direction;
    uint32_t step{};
    uint16_t aux0{};
    uint16_t aux1{};
};


#endif //WLED_DISABLE_PARTICLESYSTEM1D
