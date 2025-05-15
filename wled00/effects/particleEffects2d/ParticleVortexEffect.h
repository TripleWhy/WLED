#pragma once
#ifndef WLED_DISABLE_2D
#ifndef WLED_DISABLE_PARTICLESYSTEM2D

#include "../../FX.h"
#include "../Effect.h"
#include "Particle2dEffect.h"

/*
  Particle System Vortex
  Particles sprayed from center with a rotating spray
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
class ParticleVortexEffect : public BaseEffect<ParticleVortexEffect, Particle2dEffect<ParticleVortexEffect>> {
private:
    using Self = ParticleVortexEffect;
    using Base = BaseEffect<Self, Particle2dEffect<Self>>;

    static constexpr uint32_t NUMBEROFSOURCES = 8;

public:
    static constexpr const char metaData[] PROGMEM = "PS Vortex@Rotation Speed,Particle Speed,Arms,Flip,Nozzle,Smear,Direction,Random Flip;;!;2;pal=27,c1=200,c2=0,c3=0";
    static constexpr const uint8_t effectId = FX_MODE_PARTICLEVORTEX;

    using Base::Base;

    bool init(const EffectCoordinate& coordinate) {
        if (!Base::init(coordinate, NUMBEROFSOURCES, false, false)) {
            return false;
        }

        #ifdef ESP8266
        PartSys.setMotionBlur(180);
        #else
        PartSys.setMotionBlur(130);
        #endif
        for (uint32_t i = 0; i < min(PartSys.sources.size(), (uint32_t)NUMBEROFSOURCES); i++) {
            PartSys.sources[i].source.x = (PartSys.maxX + 1) >> 1; // center
            PartSys.sources[i].source.y = (PartSys.maxY + 1) >> 1; // center
            PartSys.sources[i].maxLife = 900;
            PartSys.sources[i].minLife = 800;
        }
        PartSys.setKillOutOfBounds(true);
        return true;
    }

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        uint32_t i, j;

        PartSys.updateSystem(coordinate.width, coordinate.height); // update system properties (dimensions and data pointers)
        uint32_t spraycount = min(PartSys.sources.size(), (uint32_t)(1 + (parameters.custom1 >> 5))); // number of sprays to display, 1-8
        #ifdef ESP8266
        for (i = 1; i < 4; i++) { // need static particles in the center to reduce blinking (would be black every other frame without this hack), just set them there fixed
            int partindex = (int)PartSys.usedParticles - (int)i;
            if (partindex >= 0) {
                PartSys.particles[partindex].x = (PartSys.maxX + 1) >> 1; // center
                PartSys.particles[partindex].y = (PartSys.maxY + 1) >> 1; // center
                PartSys.particles[partindex].sat = 230;
                PartSys.particles[partindex].ttl = 256; //keep alive
            }
        }
        #endif

        if (parameters.check1)
            PartSys.setSmearBlur(90); // enable smear blur
        else
            PartSys.setSmearBlur(0); // disable smear blur

        // update colors of the sprays
        for (i = 0; i < spraycount; i++) {
            uint32_t coloroffset = 0xFF / spraycount;
            PartSys.sources[i].source.hue = coloroffset * i;
        }

        // set rotation direction and speed
        // can use direction flag to determine current direction
        bool direction = parameters.check2; //no automatic direction change, set it to flag
        int32_t currentspeed = (int32_t)step; // make a signed integer out of step

        if (parameters.custom2 > 0) { // automatic direction change enabled
            uint32_t changeinterval = 1040 - ((uint32_t)parameters.custom2 << 2);
            direction = aux1 & 0x01; //set direction according to flag

            if (parameters.check3) // random interval
                changeinterval = 20 + changeinterval + hw_random16(changeinterval);

            if (parameters.call % changeinterval == 0) { //flip direction on next frame
                aux1 |= 0x02; // set the update flag (for random interval update)
                if (direction)
                    aux1 &= ~0x01; // clear the direction flag
                else
                    aux1 |= 0x01; // set the direction flag
            }
        }

        int32_t targetspeed = (direction ? 1 : -1) * (parameters.speed << 3);
        int32_t speeddiff = targetspeed - currentspeed;
        int32_t speedincrement = speeddiff / 50;

        if (speedincrement == 0) { //if speeddiff is not zero, make the increment at least 1 so it reaches target speed
            if (speeddiff < 0)
                speedincrement = -1;
            else if (speeddiff > 0)
                speedincrement = 1;
        }

        currentspeed += speedincrement;
        aux0 += currentspeed;
        step = (uint32_t)currentspeed; //save it back

        uint16_t angleoffset = 0xFFFF / spraycount; // angle offset for an even distribution
        uint32_t skip = PS_P_HALFRADIUS / (parameters.intensity + 1) + 1; // intensity is emit speed, emit less on low speeds
        if (parameters.call % skip == 0) {
            j = hw_random16(spraycount); // start with random spray so all get a chance to emit a particle if maximum number of particles alive is reached.
            for (i = 0; i < spraycount; i++) { // emit one particle per spray (if available)
                PartSys.sources[j].var = (parameters.custom3 >> 1); //update speed variation
                #ifdef ESP8266
                if (parameters.call & 0x01) // every other frame, do not emit to save particles
                #endif
                PartSys.angleEmit(PartSys.sources[j], aux0 + angleoffset * j, (parameters.intensity >> 2)+1);
                j = (j + 1) % spraycount;
            }
        }
        PartSys.update(buffer, parameters); //update all particles and render to frame
        return true;
    }

private:
    uint32_t step{};
    uint16_t aux0{};
    uint16_t aux1{};
};


#endif //WLED_DISABLE_2D
#endif //WLED_DISABLE_PARTICLESYSTEM2D
