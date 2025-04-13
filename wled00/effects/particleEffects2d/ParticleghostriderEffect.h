#pragma once
#ifndef WLED_DISABLE_2D
#ifndef WLED_DISABLE_PARTICLESYSTEM2D

#include "../../FX.h"
#include "../../FXparticleSystem.h"
#include "../BufferedEffect.h"
#include "../Effect.h"

/*
  Particle replacement of Ghost Rider by DedeHai (Damian Schneider), original FX by stepko adapted by Blaz Kristan (AKA blazoncek)
*/
#define MAXANGLESTEP 2200 //32767 means 180°
class ParticleghostriderEffect : public BaseEffect<ParticleghostriderEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = ParticleghostriderEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "PS Ghost Rider@Speed,Spiral,Blur,Color Cycle,Spread,AgeColor,Walls;;!;2;pal=1,sx=70,ix=0,c1=220,c2=30,c3=21,o1=1";
    static constexpr const uint8_t effectId = FX_MODE_PARTICLEGHOSTRIDER;

    explicit ParticleghostriderEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        ParticleSystem2D *PartSys = nullptr;
        PSsettings2D ghostsettings;
        ghostsettings.asByte = 0b0000011; //enable wrapX and wrapY

        if (SEGMENT.call == 0) { // initialization
            if (!initParticleSystem2D(PartSys, 1)) // init, no additional data needed
                return mode_static(); // allocation failed or not 2D
            PartSys.setKillOutOfBounds(true); // out of bounds particles dont return (except on top, taken care of by gravity setting)
            PartSys.sources[0].maxLife = 260; // lifetime in frames
            PartSys.sources[0].minLife = 250;
            PartSys.sources[0].source.x = hw_random16(PartSys.maxX);
            PartSys.sources[0].source.y = hw_random16(PartSys.maxY);
            step = hw_random16(MAXANGLESTEP) - (MAXANGLESTEP>>1); // angle increment
        }
        else {
            PartSys = reinterpret_cast<ParticleSystem2D *>(SEGENV.data); // if not first call, just set the pointer to the PS
        }

        if (PartSys == nullptr)
            return mode_static(); // something went wrong, no data!

        if (SEGMENT.intensity > 0) { // spiraling
            if (aux1) {
                step += SEGMENT.intensity>>3;
                if ((int32_t)step > MAXANGLESTEP)
                    aux1 = 0;
            }
            else {
                step -= SEGMENT.intensity>>3;
                if ((int32_t)step < -MAXANGLESTEP)
                    aux1 = 1;
            }
        }
        // Particle System settings
        PartSys.updateSystem(); // update system properties (dimensions and data pointers)
        PartSys.setMotionBlur(SEGMENT.custom1);
        PartSys.sources[0].var = SEGMENT.custom3 >> 1;

        // color by age (PS 'color by age' always starts with hue = 255, don't want that here)
        if (SEGMENT.check1) {
            for (uint32_t i = 0; i < PartSys.usedParticles; i++) {
                PartSys.particles[i].hue = PartSys.sources[0].source.hue + (PartSys.particles[i].ttl<<2);
            }
        }

        // enable/disable walls
        ghostsettings.bounceX = SEGMENT.check2;
        ghostsettings.bounceY = SEGMENT.check2;

        aux0 += (int32_t)step; // step is angle increment
        uint16_t emitangle = aux0 + 32767; // +180°
        int32_t speed = map(SEGMENT.speed, 0, 255, 12, 64);
        PartSys.sources[0].source.vx = ((int32_t)cos16_t(aux0) * speed) / (int32_t)32767;
        PartSys.sources[0].source.vy = ((int32_t)sin16_t(aux0) * speed) / (int32_t)32767;
        PartSys.sources[0].source.ttl = 500; // source never dies (note: setting 'perpetual' is not needed if replenished each frame)
        PartSys.particleMoveUpdate(PartSys.sources[0].source, PartSys.sources[0].sourceFlags, &ghostsettings);
        // set head (steal one of the particles)
        PartSys.particles[PartSys.usedParticles-1].x = PartSys.sources[0].source.x;
        PartSys.particles[PartSys.usedParticles-1].y = PartSys.sources[0].source.y;
        PartSys.particles[PartSys.usedParticles-1].ttl = 255;
        PartSys.particles[PartSys.usedParticles-1].sat = 0; //white
        // emit two particles
        PartSys.angleEmit(PartSys.sources[0], emitangle, speed);
        PartSys.angleEmit(PartSys.sources[0], emitangle, speed);
        if (SEGMENT.call % (11 - (SEGMENT.custom2 / 25)) == 0) { // every nth frame, cycle color and emit particles //TODO: make this a segment call % SEGMENT.custom2  for better control
            PartSys.sources[0].source.hue++;
        }
        if (SEGMENT.custom2 > 190) //fast color change
            PartSys.sources[0].source.hue += (SEGMENT.custom2 - 190) >> 2;

        PartSys.update(); // update and render
        return true;
    }

private:
    uint32_t step{};
    uint16_t aux0{};
    uint16_t aux1{};
};


#endif //WLED_DISABLE_2D
#endif //WLED_DISABLE_PARTICLESYSTEM2D
