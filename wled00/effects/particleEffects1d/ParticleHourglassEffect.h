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
class ParticleHourglassEffect : public BaseEffect<ParticleHourglassEffect, Particle1dEffect> {
private:
    using Self = ParticleHourglassEffect;
    using Base = BaseEffect<Self, Particle1dEffect>;

public:
    static constexpr const char metaData[] PROGMEM = "PS Hourglass@Interval,!,Color,Blur,Gravity,Colorflip,Start,Fast Reset;,!;!;1;pal=34,sx=50,ix=200,c1=140,c2=80,c3=4,o1=1,o2=1,o3=1";
    static constexpr const uint8_t effectId = FX_MODE_PSHOURGLASS;

    explicit ParticleHourglassEffect(const EffectInformation& ei)
        : Base{ei, 0, 255, false}
    {
        PartSys.setBounce(true);
        PartSys.setWallHardness(100);
    }

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        constexpr int positionOffset = PS_P_RADIUS_1D / 2; // resting position offset

        // Particle System settings
        PartSys.updateSystem(coordinate.width); // update system properties (dimensions and data pointers)
        PartSys.setUsedParticles(map(SEGMENT.intensity, 0, 255, 1, 255));
        PartSys.setMotionBlur(SEGMENT.custom2); // anable motion blur
        PartSys.setGravity(map(SEGMENT.custom3, 0, 31, 1, 30));
        PartSys.enableParticleCollisions(true, 34); // hardness value found by experimentation on different settings

        uint32_t colormode = SEGMENT.custom1 >> 5; // 0-7

        if ((SEGMENT.intensity | (PartSys.getAvailableParticles() << 8)) != settingTracker) { // initialize, getAvailableParticles changes while in FX transition
            settingTracker = SEGMENT.intensity | (PartSys.getAvailableParticles() << 8);
            for (uint32_t i = 0; i < PartSys.usedParticles; i++) {
                PartSys.particleFlags[i].reversegrav = true;
                direction = 0; // down
                aux1 = 1; // initialize below
            }
            aux0 = PartSys.usedParticles - 1; // initial state, start with highest number particle
        }

        for (uint32_t i = 0; i < PartSys.usedParticles; i++) { // check if particle reached target position after falling
            int32_t targetposition;
            if (PartSys.particleFlags[i].fixed == false) { // && abs(PartSys.particles[i].vx) < 8) {
                // calculate target position depending on direction
                bool closeToTarget = false;
                bool reachedTarget = false;
                if (PartSys.particleFlags[i].reversegrav) { // up
                    targetposition = PartSys.maxX - (i * PS_P_RADIUS_1D) - positionOffset; // target resting position
                    if (targetposition - PartSys.particles[i].x <= 5 * PS_P_RADIUS_1D)
                        closeToTarget = true;
                    if (PartSys.particles[i].x >= targetposition) // particle has reached target position, pin it. if not pinned, they do not stack well on larger piles
                        reachedTarget = true;
                }
                else { // down, highest index particle drops first
                    targetposition = (PartSys.usedParticles - i) * PS_P_RADIUS_1D - positionOffset; // target resting position note: using -offset instead of -1 + offset
                    if (PartSys.particles[i].x - targetposition <= 5 * PS_P_RADIUS_1D)
                        closeToTarget = true;
                    if (PartSys.particles[i].x <= targetposition) // particle has reached target position, pin it. if not pinned, they do not stack well on larger piles
                        reachedTarget = true;
                }
                if (reachedTarget || (closeToTarget && abs(PartSys.particles[i].vx) < 10)) { // reached target or close to target and slow speed
                    PartSys.particles[i].x = targetposition; // set exact position
                    PartSys.particleFlags[i].fixed = true;   // pin particle
                }
            }
            if (colormode == 7)
                PartSys.setColorByPosition(true); // color fixed by position
            else {
                PartSys.setColorByPosition(false);
                uint8_t basehue = ((SEGMENT.custom1 & 0x1F) << 3); // use 5 LSBs to select color
                switch(colormode) {
                    case 0: PartSys.particles[i].hue = 120; break; // fixed at 120, if flip is activated, this can make red and green (use palette 34)
                    case 1: PartSys.particles[i].hue = basehue; break; // fixed selectable color
                    case 2: // 2 colors inverleaved (same code as 3)
                    case 3: PartSys.particles[i].hue = ((SEGMENT.custom1 & 0x1F) << 1) + (i % colormode)*74; break; // interleved colors (every 2 or 3 particles)
                    case 4: PartSys.particles[i].hue = basehue + (i * 255) / PartSys.usedParticles;  break; // gradient palette colors
                    case 5: PartSys.particles[i].hue = basehue + (i * 1024) / PartSys.usedParticles;  break; // multi gradient palette colors
                    case 6: PartSys.particles[i].hue = i + (strip.now >> 3);  break; // disco! moving color gradient
                    default: break;
                }
            }
            if (SEGMENT.check1 && !PartSys.particleFlags[i].reversegrav) // flip color when fallen
                PartSys.particles[i].hue += 120;
        }

        if (aux1 == 1) { // last countdown call before dropping starts, reset all particles
            for (uint32_t i = 0; i < PartSys.usedParticles; i++) {
                PartSys.particleFlags[i].collide = true;
                PartSys.particleFlags[i].perpetual = true;
                PartSys.particles[i].ttl = 260;
                uint32_t targetposition;
                //calculate target position depending on direction
                if (PartSys.particleFlags[i].reversegrav)
                     targetposition = PartSys.maxX - (i * PS_P_RADIUS_1D + positionOffset); // target resting position
                else
                    targetposition = (PartSys.usedParticles - i) * PS_P_RADIUS_1D - positionOffset; // target resting position  -5 - PS_P_RADIUS_1D/2

                PartSys.particles[i].x = targetposition;
                PartSys.particleFlags[i].fixed = true;
            }
        }

        if (aux1 == 0) { // countdown passed, run
            if (strip.now >= step) { // drop a particle, do not drop more often than every second frame or particles tangle up quite badly
                // set next drop time
                if (SEGMENT.check3 && direction) // fast reset
                    step = strip.now + 100; // drop one particle every 100ms
                else // normal interval
                    step = strip.now + max(20, SEGMENT.speed * 20); // map speed slider from 0.1s to 5s
                if (aux0 < PartSys.usedParticles) {
                    PartSys.particleFlags[aux0].reversegrav = direction; // let this particle fall or rise
                    PartSys.particleFlags[aux0].fixed = false; // unpin
                }
                else { // overflow
                    direction = !(direction); // flip direction
                    aux1 = SEGMENT.virtualLength() + 100; // set countdown
                }
                if (direction == false) // down, start dropping the highest number particle
                    aux0--; // next particle
                else
                    aux0++;
            }
        }
        else if (SEGMENT.check2) // auto reset
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
