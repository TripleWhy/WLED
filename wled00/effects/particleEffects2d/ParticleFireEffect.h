#pragma once
#ifndef WLED_DISABLE_2D
#ifndef WLED_DISABLE_PARTICLESYSTEM2D

#include "../../FX.h"
#include "../Effect.h"
#include "Particle2dEffect.h"

/*
  Particle Fire
  realistic fire effect using particles. heat based and using perlin-noise for wind
  by DedeHai (Damian Schneider)
*/
class ParticleFireEffect : public BaseEffect<ParticleFireEffect, Particle2dEffect<ParticleFireEffect>> {
private:
    using Self = ParticleFireEffect;
    using Base = BaseEffect<Self, Particle2dEffect<Self>>;

public:
    static constexpr const char metaData[] PROGMEM = "PS Fire@Speed,Intensity,Flame Height,Wind,Spread,Smooth,Cylinder,Turbulence;;!;2;pal=35,sx=110,c1=110,c2=50,c3=31,o1=1";
    static constexpr const uint8_t effectId = FX_MODE_PARTICLEFIRE;

    using Base::Base;

    bool init(const EffectCoordinate& coordinate) {
        return Base::init(coordinate, coordinate.width, false, false);
    }

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        uint32_t i; // index variable
        uint32_t numFlames; // number of flames: depends on fire width. for a fire width of 16 pixels, about 25-30 flames give good results

        PartSys.updateSystem(coordinate.width, coordinate.height); // update system properties (dimensions and data pointers)
        PartSys.setWrapX(parameters.check2);
        PartSys.setMotionBlur(parameters.check1 * 170); // anable/disable motion blur

        uint32_t firespeed = max((uint8_t)100, parameters.speed); //limit speed to 100 minimum, reduce frame rate to make it slower (slower speeds than 100 do not look nice)
        if (parameters.speed < 100) { //slow, limit FPS
            uint32_t period = strip.now - lastcall;
            if (period < (uint32_t)map(parameters.speed, 0, 99, 50, 10)) { // limit to 90FPS - 20FPS
                parameters.call--; //skipping a frame, decrement the counter (on call0, this is never executed as lastcall is 0, so its fine to not check if >0)
                //still need to render the frame or flickering will occur in transitions
                PartSys.updateFire(buffer, parameters.intensity, true); // render the fire without updating particles (render only)
                return false; //do not update this frame
            }
            lastcall = strip.now;
        }

        uint32_t spread = (PartSys.maxX >> 5) * (parameters.custom3 + 1); //fire around segment center (in subpixel points)
        numFlames = min((uint32_t)PartSys.sources.size(), (4 + ((spread / PS_P_RADIUS) << 1))); // number of flames used depends on spread with, good value is (fire width in pixel) * 2
        uint32_t percycle = (numFlames * 2) / 3; // maximum number of particles emitted per cycle (TODO: for ESP826 maybe use flames/2)

        // update the flame sprays:
        for (i = 0; i < numFlames; i++) {
            if (parameters.call & 1 && PartSys.sources[i].source.ttl > 0) { // every second frame
                PartSys.sources[i].source.ttl--;
            } else { // flame source is dead: initialize new flame: set properties of source
                PartSys.sources[i].source.x = (PartSys.maxX >> 1) - (spread >> 1) + hw_random(spread); // change flame position: distribute randomly on chosen width
                PartSys.sources[i].source.y = -(PS_P_RADIUS << 2); // set the source below the frame
                PartSys.sources[i].source.ttl = 20 + hw_random16((parameters.custom1 * parameters.custom1) >> 8) / (1 + (firespeed >> 5)); //'hotness' of fire, faster flames reduce the effect or flame height will scale too much with speed
                PartSys.sources[i].maxLife = hw_random16(coordinate.height >> 1) + 16; // defines flame height together with the vy speed, vy speed*maxlife/PS_P_RADIUS is the average flame height
                PartSys.sources[i].minLife = PartSys.sources[i].maxLife >> 1;
                PartSys.sources[i].vx = hw_random16(4) - 2; // emitting speed (sideways)
                PartSys.sources[i].vy = (coordinate.height >> 1) + (firespeed >> 4) + (parameters.custom1 >> 4); // emitting speed (upwards)
                PartSys.sources[i].var = 2 + hw_random16(2 + (firespeed >> 4)); // speed variation around vx,vy (+/- var)
            }
        }

        if (parameters.call % 3 == 0) { // update noise position and add wind
            aux0++; // position in the perlin noise matrix for wind generation
            if (parameters.call % 10 == 0)
                aux1++; // move in noise y direction so noise does not repeat as often
            // add wind force to all particles
            int8_t windspeed = ((int16_t)(perlin8(aux0, aux1) - 127) * parameters.custom2) >> 7;
            PartSys.applyForce(windspeed, 0);
        }
        step++;

        if (parameters.check3) { //add turbulance (parameters and algorithm found by experimentation)
            if (parameters.call % map(firespeed, 0, 255, 4, 15) == 0) {
                for (i = 0; i < PartSys.usedParticles; i++) {
                    if (PartSys.particles[i].y < PartSys.maxY / 4) { // do not apply turbulance everywhere -> bottom quarter seems a good balance
                        int32_t curl = ((int32_t)perlin8(PartSys.particles[i].x, PartSys.particles[i].y, step << 4) - 127);
                        PartSys.particles[i].vx += (curl * (firespeed + 10)) >> 9;
                    }
                }
            }
        }

        uint8_t j = hw_random16(); // start with a random flame (so each flame gets the chance to emit a particle if available particles is smaller than number of flames)
        for (i = 0; i < percycle; i++) {
            j = (j + 1) % numFlames;
            PartSys.flameEmit(PartSys.sources[j]);
        }

        PartSys.updateFire(buffer, parameters.intensity, false); // update and render the fire
        return true;
    }

private:
    uint32_t lastcall{};
    uint32_t step{};
    uint16_t aux0{hw_random16()}; // aux0 is wind position (index) in the perlin noise
    uint16_t aux1{};
};


#endif //WLED_DISABLE_2D
#endif //WLED_DISABLE_PARTICLESYSTEM2D
