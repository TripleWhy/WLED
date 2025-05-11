#pragma once
#ifndef WLED_DISABLE_2D
#ifndef WLED_DISABLE_PARTICLESYSTEM2D

#include "../../FX.h"
#include "../Effect.h"
#include "Particle2dEffect.h"

/*
  Particle smashing down like meteors and exploding as they hit the ground, has many parameters to play with
  by DedeHai (Damian Schneider)
*/
class ParticleImpactEffect : public BaseEffect<ParticleImpactEffect, Particle2dEffect<ParticleImpactEffect>> {
private:
    using Self = ParticleImpactEffect;
    using Base = BaseEffect<Self, Particle2dEffect<Self>>;

    static constexpr uint32_t NUMBEROFSOURCES = 8;

public:
    static constexpr const char metaData[] PROGMEM = "PS Impact@Launches,!,Force,Hardness,Blur,Cylinder,Walls,Collide;;!;2;pal=0,sx=32,ix=85,c1=70,c2=130,c3=0,o3=1";
    static constexpr const uint8_t effectId = FX_MODE_PARTICLEIMPACT;

    using Base::Base;

    bool init(const EffectCoordinate& coordinate) {
        if (!Base::init(coordinate, NUMBEROFSOURCES, false, false)) {
            return false;
        }

        PartSys.setKillOutOfBounds(true);
        PartSys.setGravity(); // enable default gravity
        PartSys.setBounceY(true); // always use ground bounce
        PartSys.setWallRoughness(220); // high roughness
        const uint8_t MaxNumMeteors = min(PartSys.sources.size(), (uint32_t)NUMBEROFSOURCES);
        for (uint32_t i = 0; i < MaxNumMeteors; i++) {
         // PartSys.sources[i].source.y = 500;
            PartSys.sources[i].source.ttl = hw_random16(10 * i); // set initial delay for meteors
            PartSys.sources[i].source.vy = 10; // at positive speeds, no particles are emitted and if particle dies, it will be relaunched
        }
        return true;
    }

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        uint32_t i = 0;
        uint8_t MaxNumMeteors;
        PSsettings2D meteorsettings;
        meteorsettings.asByte = 0b00101000; // PS settings for meteors: bounceY and gravity enabled

        // Particle System settings
        PartSys.updateSystem(coordinate.width, coordinate.height); // update system properties (dimensions and data pointers)
        PartSys.setWrapX(parameters.check1);
        PartSys.setBounceX(parameters.check2);
        PartSys.setMotionBlur(parameters.custom3<<3);
        uint8_t hardness = map(parameters.custom2, 0, 255, PS_P_MINSURFACEHARDNESS - 2, 255);
        PartSys.setWallHardness(hardness);
        PartSys.enableParticleCollisions(parameters.check3, hardness); // enable collisions and set particle collision hardness
        MaxNumMeteors = min(PartSys.sources.size(), (uint32_t)NUMBEROFSOURCES);
        uint8_t numMeteors = MaxNumMeteors; // TODO: clean this up   map(parameters.custom3, 0, 31, 1, MaxNumMeteors); // number of meteors to use for animation

        uint32_t emitparticles; // number of particles to emit for each rocket's state

        for (i = 0; i < numMeteors; i++) {
            // determine meteor state by its speed:
            if ( PartSys.sources[i].source.vy < 0) { // moving down, emit sparks
            #ifdef ESP8266
                emitparticles = 1;
            #else
                emitparticles = 2;
            #endif
            }
            else if ( PartSys.sources[i].source.vy > 0) // moving up means meteor is on 'standby'
                emitparticles = 0;
            else { // speed is zero, explode!
                PartSys.sources[i].source.vy = 10; // set source speed positive so it goes into timeout and launches again
            #ifdef ESP8266
                emitparticles = hw_random16(parameters.intensity >> 3) + 5; // defines the size of the explosion
            #else
                emitparticles = map(parameters.intensity, 0, 255, 10, hw_random16(PartSys.usedParticles>>2)); // defines the size of the explosion !!!TODO: check if this works on ESP8266, drop esp8266 def if it does
            #endif
            }
            for (int e = emitparticles; e > 0; e--) {
                    PartSys.sprayEmit(PartSys.sources[i]);
            }
        }

        // update the meteors, set the speed state
        for (i = 0; i < numMeteors; i++) {
            if (PartSys.sources[i].source.ttl) {
                PartSys.sources[i].source.ttl--; // note: this saves an if statement, but moving down particles age twice
                if (PartSys.sources[i].source.vy < 0) { // move down
                    PartSys.applyGravity(PartSys.sources[i].source);
                    PartSys.particleMoveUpdate(PartSys.sources[i].source, PartSys.sources[i].sourceFlags, &meteorsettings);

                    // if source reaches the bottom, set speed to 0 so it will explode on next function call (handled above)
                    if (PartSys.sources[i].source.y < PS_P_RADIUS<<1) { // reached the bottom pixel on its way down
                        PartSys.sources[i].source.vy = 0; // set speed zero so it will explode
                        PartSys.sources[i].source.vx = 0;
                        PartSys.sources[i].sourceFlags.collide = true;
                        #ifdef ESP8266
                        PartSys.sources[i].maxLife = 180;
                        PartSys.sources[i].minLife = 20;
                        #else
                        PartSys.sources[i].maxLife = 250;
                        PartSys.sources[i].minLife = 50;
                        #endif
                        PartSys.sources[i].source.ttl = hw_random16((512 - (parameters.speed << 1))) + 40; // standby time til next launch (in frames)
                        PartSys.sources[i].vy = (parameters.custom1 >> 2);  // emitting speed y
                        PartSys.sources[i].var = (parameters.custom1 >> 2); // speed variation around vx,vy (+/- var)
                    }
                }
            }
            else if (PartSys.sources[i].source.vy > 0) {  // meteor is exploded and time is up (ttl==0 and positive speed), relaunch it
                // reinitialize meteor
                PartSys.sources[i].source.y = PartSys.maxY + (PS_P_RADIUS << 2); // start 4 pixels above the top
                PartSys.sources[i].source.x = hw_random(PartSys.maxX);
                PartSys.sources[i].source.vy = -hw_random16(30) - 30; // meteor downward speed
                PartSys.sources[i].source.vx = hw_random16(50) - 25; // TODO: make this dependent on position so they do not move out of frame
                PartSys.sources[i].source.hue = hw_random16(); // random color
                PartSys.sources[i].source.ttl = 500; // long life, will explode at bottom
                PartSys.sources[i].sourceFlags.collide = false; // trail particles will not collide
                PartSys.sources[i].maxLife = 60; // spark particle life
                PartSys.sources[i].minLife = 20;
                PartSys.sources[i].vy = -9; // emitting speed (down)
                PartSys.sources[i].var = 3; // speed variation around vx,vy (+/- var)
            }
        }

        PartSys.update(buffer); // update and render
        return true;
    }

private:
};


#endif //WLED_DISABLE_2D
#endif //WLED_DISABLE_PARTICLESYSTEM2D
