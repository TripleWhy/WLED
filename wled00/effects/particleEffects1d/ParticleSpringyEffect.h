#pragma once
#ifndef WLED_DISABLE_PARTICLESYSTEM1D

#include "../../FX.h"
#include "../Effect.h"
#include "Particle1dEffect.h"

/*
Particles bound by springs
by DedeHai (Damian Schneider)
*/
class ParticleSpringyEffect : public BaseEffect<ParticleSpringyEffect, Particle1dEffect<ParticleSpringyEffect>> {
private:
    using Self = ParticleSpringyEffect;
    using Base = BaseEffect<Self, Particle1dEffect<Self>>;

public:
    static constexpr const char* const metaData PROGMEM = "PS Springy@Stiffness,Damping,Density,Hue,Mode,Smear,XL,AR;,!;!;1f;pal=54,c2=0,c3=23";
    static constexpr const uint8_t effectId = FX_MODE_PS1DSPRINGY;

    using Base::Base;

    bool init(const EffectCoordinate& coordinate) {
        return Base::init(coordinate, 1, 128, true);
    }

    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(coordinate)) {
            return false;
        }

        // Particle System settings
        PartSys.updateSystem(); // update system properties (dimensions and data pointers)
        PartSys.setMotionBlur(220 * SEGMENT.check1); // anable motion blur
        PartSys.setSmearBlur(50); // smear a little
        PartSys.setUsedParticles(map(SEGMENT.custom1, 0, 255, 30 >> SEGMENT.check2, 255  >> (SEGMENT.check2*2))); // depends on density and particle size
 // PartSys.enableParticleCollisions(true, 140); // enable particle collisions, can not be set too hard or impulses will not strech the springs if soft.
        int32_t springlength = PartSys.maxX / (PartSys.usedParticles); // spring length (spacing between particles)
        int32_t springK = map(SEGMENT.speed, 0, 255, 5, 35); // spring constant (stiffness)

        uint32_t settingssum = SEGMENT.custom1 + SEGMENT.check2 + PartSys.getAvailableParticles(); // note: getAvailableParticles is used to enforce update during transitions
        if (aux0 != settingssum) { // number of particles changed, update distribution
            for (int32_t i = 0; i < (int32_t)PartSys.usedParticles; i++) {
                PartSys.advPartProps[i].sat = 255; // full saturation
                //PartSys.particleFlags[i].collide = true; // enable collision for particles
                PartSys.particles[i].x = (i+1) * ((PartSys.maxX) / (PartSys.usedParticles)); // distribute
                //PartSys.particles[i].vx = 0; //reset speed
                PartSys.advPartProps[i].size = SEGMENT.check2 ? 190 : 2; // set size, small or big
            }
            aux0 = settingssum;
        }
        int dxlimit = (2 + ((255 - SEGMENT.speed) >> 5)) * springlength; // limit for spring length to avoid overstretching

        int springforce[PartSys.usedParticles]; // spring forces
        memset(springforce, 0, PartSys.usedParticles * sizeof(int32_t)); // reset spring forces

        // calculate spring forces and limit particle positions
        if (PartSys.particles[0].x < -springlength)
            PartSys.particles[0].x = -springlength; // limit the spring length
        else if (PartSys.particles[0].x > dxlimit)
            PartSys.particles[0].x = dxlimit; // limit the spring length
        springforce[0] += ((springlength >> 1) - (PartSys.particles[0].x)) * springK; // first particle anchors to x=0

        for (int32_t i = 1; i < PartSys.usedParticles; i++) {
            // reorder particles if they are out of order to prevent chaos
            if (PartSys.particles[i].x < PartSys.particles[i-1].x)
                    std::swap(PartSys.particles[i].x, PartSys.particles[i-1].x); // swap particle positions to maintain order
            int dx = PartSys.particles[i].x - PartSys.particles[i-1].x; // distance, always positive
            if (dx > dxlimit) { // limit the spring length
                PartSys.particles[i].x = PartSys.particles[i-1].x + dxlimit;
                dx = dxlimit;
            }
            int dxleft = (springlength - dx); // offset from spring resting position
            springforce[i] += dxleft * springK;
            springforce[i-1] -= dxleft * springK;
            if (i == (PartSys.usedParticles - 1)) {
             if (PartSys.particles[i].x >= PartSys.maxX + springlength)
                    PartSys.particles[i].x = PartSys.maxX + springlength;
                int dxright = (springlength >> 1) - (PartSys.maxX - PartSys.particles[i].x); // last particle anchors to x=maxX
                springforce[i] -= dxright * springK;
            }
        }
        // apply spring forces to particles
        bool dampenoscillations = (SEGMENT.call % (9 - (SEGMENT.speed >> 5))) == 0; // dampen oscillation if particles are slow, more damping on stiffer springs
        for (int32_t i = 0; i < PartSys.usedParticles; i++) {
            springforce[i] = springforce[i] / 64; // scale spring force (cannot use shifts because of negative values)
            int maxforce = 120; // limit spring force
            springforce[i] = springforce[i] > maxforce ? maxforce : springforce[i] < -maxforce ? -maxforce : springforce[i]; // limit spring force
            PartSys.applyForce(PartSys.particles[i], springforce[i], PartSys.advPartProps[i].forcecounter);
            //dampen slow particles to avoid persisting oscillations on higher stiffness
            if (dampenoscillations) {
                if (abs(PartSys.particles[i].vx) < 3 && abs(springforce[i]) < (springK >> 2))
                    PartSys.particles[i].vx = (PartSys.particles[i].vx * 254) / 256; // take out some energy
            }
            PartSys.particles[i].ttl = 300; // reset ttl, cannot use perpetual
        }

        if (SEGMENT.call % ((65 - ((SEGMENT.intensity * (1 + (SEGMENT.speed>>3))) >> 7))) == 0) // more damping for higher stiffness
            PartSys.applyFriction((SEGMENT.intensity >> 2));

        // add a small resetting force so particles return to resting position even under high damping
        for (int32_t i = 1; i < PartSys.usedParticles - 1; i++) {
            int restposition = (springlength >> 1) + i * springlength; // resting position
            int dx = restposition - PartSys.particles[i].x; // distance, always positive
            PartSys.applyForce(PartSys.particles[i], dx > 0 ? 1 : (dx < 0 ? -1 : 0), PartSys.advPartProps[i].forcecounter);
        }

        // Modes
        if (SEGMENT.check3) { // use AR, custom 3 becomes frequency band to use, applies velocity to center particle according to loudness
            um_data_t *um_data = getAudioData();
            uint8_t *fftResult = (uint8_t *)um_data->u_data[2]; // 16 bins with FFT data, log mapped already, each band contains frequency amplitude 0-255
            uint32_t baseBin = map(SEGMENT.custom3, 0, 31, 0, 14);
            uint32_t loudness = fftResult[baseBin] + fftResult[baseBin+1];
            uint32_t threshold = 80; //150 - (SEGMENT.intensity >> 1);
            if (loudness > threshold) {
                    int offset = (PartSys.maxX >> 1) - PartSys.particles[PartSys.usedParticles>>1].x; // offset from center
                    if (abs(offset) < PartSys.maxX >> 5) // push particle around in center sector
                        PartSys.particles[PartSys.usedParticles>>1].vx = ((PartSys.particles[PartSys.usedParticles>>1].vx > 0 ? 1 : -1)) * (loudness >> 3);
            }
        }
        else{
            if (SEGMENT.custom3 <= 10) { // periodic pulse: 0-5 apply at start, 6-10 apply at center
                if (strip.now > SEGMENT.step) {
                    int speed = (SEGMENT.custom3 > 5) ? (SEGMENT.custom3 - 6) : SEGMENT.custom3;
                    SEGMENT.step = strip.now + 7500 - ((SEGMENT.speed << 3) + (speed << 10));
                    int amplitude = 40 + (SEGMENT.custom1 >> 2);
                    int index = (SEGMENT.custom3 > 5) ? (PartSys.usedParticles / 2) : 0; // center or start particle
                    PartSys.particles[index].vx += amplitude;
                }
            }
            else if (SEGMENT.custom3 <= 30) { // sinusoidal wave: 11-20 apply at start, 21-30 apply at center
                int index = (SEGMENT.custom3 > 20) ? (PartSys.usedParticles / 2) : 0; // center or start particle
                int restposition = 0;
                if (index > 0) restposition = PartSys.maxX >> 1; // center
                //int amplitude = 5 + (SEGMENT.speed >> 3) + (SEGMENT.custom1 >> 2); // amplitude depends on density
                int amplitude = 5 + (SEGMENT.custom1 >> 2); // amplitude depends on density
                int speed = SEGMENT.custom3 - 10 - (index ? 10 : 0); // map 11-20 and 21-30 to 1-10
                int phase = strip.now * ((1 + (SEGMENT.speed >> 4)) * speed);
                if (SEGMENT.check2) amplitude <<= 1; // double amplitude for XL particles
                //PartSys.applyForce(PartSys.particles[index], (sin16_t(phase) * amplitude) >> 15, PartSys.advPartProps[index].forcecounter); // apply acceleration
                PartSys.particles[index].x = restposition + ((sin16_t(phase) * amplitude) >> 12); // apply position
            }
            else {
                if (hw_random16() < 656) { // ~1% chance to add a pulse
                    int amplitude = 60;
                    if (SEGMENT.check2) amplitude <<= 1; // double amplitude for XL particles
                    PartSys.particles[PartSys.usedParticles >> 1].vx += hw_random16(amplitude << 1) - amplitude; // apply acceleration
                }
            }
        }

        for (int32_t i = 0; i < PartSys.usedParticles; i++) {
            if (SEGMENT.custom2 == 255) { // map speed to hue
                 int speedclr = ((int8_t(abs(PartSys.particles[i].vx))) >> 2) << 4; // scale for greater color variation, dump small values to avoid flickering
                 //int speed = PartSys.particles[i].vx << 2; // +/- 512
                 if (speedclr > 240) speedclr = 240; // limit color to non-wrapping part of palette
                 PartSys.particles[i].hue = speedclr;
            }
            else if (SEGMENT.custom2 > 0)
                PartSys.particles[i].hue = i * (SEGMENT.custom2 >> 2); // gradient distribution
            else {
                // map hue to particle density
                int deviation;
                if (i == 0) // First particle: measure density based on distance to anchor point
                    deviation = springlength/2 - PartSys.particles[i].x;
                else if (i == PartSys.usedParticles - 1) // Last particle: measure density based on distance to right boundary
                    deviation = springlength/2 - (PartSys.maxX - PartSys.particles[i].x);
                else {
                    // Middle particles: average of compression/expansion from both sides
                    int leftDx = PartSys.particles[i].x - PartSys.particles[i-1].x;
                    int rightDx = PartSys.particles[i+1].x - PartSys.particles[i].x;
                    int avgDistance = (leftDx + rightDx) >> 1;
                    if (avgDistance < 0) avgDistance = 0; // avoid negative distances (not sure why this happens)
                    deviation = (springlength - avgDistance);
                }
                deviation = constrain(deviation, -127, 112); // limit deviation to -127..112 (do not go intwo wrapping part of palette)
                PartSys.particles[i].hue = 127 + deviation; // map density to hue
            }
        }
        PartSys.update(); // update and render
    }

private:
    uint16_t aux0{0xFFFF};
    uint16_t aux1{0xFFFF};
};


#endif //WLED_DISABLE_PARTICLESYSTEM1D
