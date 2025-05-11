#pragma once
#ifndef WLED_DISABLE_PARTICLESYSTEM1D

#include "../../FX.h"
#include "../Effect.h"
#include "Particle1dEffect.h"

/*
  Particle based AR effect, swoop particles along the strip with selected frequency loudness
  by DedeHai (Damian Schneider)
*/
class Particle1dSonicStreamEffect : public BaseEffect<Particle1dSonicStreamEffect, Particle1dEffect<Particle1dSonicStreamEffect>> {
private:
    using Self = Particle1dSonicStreamEffect;
    using Base = BaseEffect<Self, Particle1dEffect<Self>>;

public:
    static constexpr const char metaData[] PROGMEM = "PS Sonic Stream@!,!,Color,Blur,Bin,Mod,Filter,Push;,!;!;1f;c3=0,o2=1";
    static constexpr const uint8_t effectId = FX_MODE_PS1DSONICSTREAM;

    using Base::Base;

    bool init(const EffectCoordinate& coordinate) {
        if (!Base::init(coordinate, 1, 255, true)) {
            return false;
        }

        PartSys.setKillOutOfBounds(true);
        PartSys.sources[0].source.x = 0; // at start
        //PartSys.sources[1].source.x = PartSys.maxX; // at end
        PartSys.sources[0].var = 0;//parameters.custom1 >> 3;
        return true;
    }

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        // Particle System settings
        PartSys.updateSystem(coordinate.width); // update system properties (dimensions and data pointers)
        PartSys.setMotionBlur(20 + (parameters.custom2 >> 1)); // anable motion blur
        PartSys.setSmearBlur(200); // smooth out the edges
        PartSys.sources[0].v = 5 + (parameters.speed >> 2);

        // FFT processing
        um_data_t *um_data = getAudioData();
        uint8_t *fftResult = (uint8_t *)um_data->u_data[2]; // 16 bins with FFT data, log mapped already, each band contains frequency amplitude 0-255
        uint32_t loudness;
        uint32_t baseBin = parameters.custom3 >> 1; // 0 - 15 map(parameters.custom3, 0, 31, 0, 14);

        loudness = fftResult[baseBin];// + fftResult[baseBin + 1];
        int mids = sqrt32_bw((int)fftResult[5] + (int)fftResult[6] + (int)fftResult[7] + (int)fftResult[8] + (int)fftResult[9] + (int)fftResult[10]); // average the mids, bin 5 is ~500Hz, bin 10 is ~2kHz (see audio_reactive.h)
        if (baseBin > 12)
            loudness = loudness << 2; // double loudness for high frequencies (better detecion)

        uint32_t threshold = 140 - (parameters.intensity >> 1);
        if (parameters.check2) { // enable low pass filter for dynamic threshold
            step = (step * 31500 + loudness * (32768 - 31500)) >> 15; // low pass filter for simple beat detection: add average to base threshold
            threshold = 20 + (threshold >> 1) + step; // add average to threshold
        }

        // color
        uint32_t hueincrement = (parameters.custom1 >> 3); // 0-31
        PartSys.sources[0].sat = parameters.custom1 > 0 ? 255 : 0; // color slider at zero: set to white
        PartSys.setColorByPosition(parameters.custom1 == 255);

        // particle manipulation
        for (uint32_t i = 0; i < PartSys.usedParticles; i++) {
            if (PartSys.sources[0].sourceFlags.perpetual == false) { // age faster if not perpetual
                if (PartSys.particles[i].ttl > 2) {
                    PartSys.particles[i].ttl -= 2; //ttl is linked to brightness, this allows to use higher brightness but still a short lifespan
                }
                else PartSys.particles[i].ttl = 0;
            }
            if (parameters.check1) { // modulate colors by mid frequencies
                int mids = sqrt32_bw((int)fftResult[5] + (int)fftResult[6] + (int)fftResult[7] + (int)fftResult[8] + (int)fftResult[9] + (int)fftResult[10]); // average the mids, bin 5 is ~500Hz, bin 10 is ~2kHz (see audio_reactive.h)
                PartSys.particles[i].hue += (mids * perlin8(PartSys.particles[i].x << 2, step << 2)) >> 9; // color by perlin noise from mid frequencies
            }
        }

        if (loudness > threshold) {
            aux0 += hueincrement; // change color
            PartSys.sources[0].minLife = 100 + (((unsigned)parameters.intensity * loudness * loudness) >> 13);
            PartSys.sources[0].maxLife = PartSys.sources[0].minLife;
            PartSys.sources[0].source.hue = aux0;
            PartSys.sources[0].size = parameters.speed;
            if (PartSys.particles[aux1].x > 3 * PS_P_RADIUS_1D || PartSys.particles[aux1].ttl == 0) { // only emit if last particle is far enough away or dead
                int partindex = PartSys.sprayEmit(PartSys.sources[0]); // emit a particle
                if (partindex >= 0) aux1 = partindex; // track last emitted particle
            }
        }
        else loudness = 0; // required for push mode

        PartSys.update(buffer); // update and render (needs to be done before manipulation for initial particle spacing to be right)

        if (parameters.check3) { // push mode
            PartSys.sources[0].sourceFlags.perpetual = true; // emitted particles dont age
            PartSys.applyFriction(1); //slow down particles
            int32_t movestep = (((int)parameters.speed + 2) * loudness) >> 10;
            if (movestep) {
                for (uint32_t i = 0; i < PartSys.usedParticles; i++) {
                    if (PartSys.particles[i].ttl) {
                        PartSys.particles[i].x += movestep; // push particles
                        PartSys.particles[i].vx = 10 + (parameters.speed >> 4) ; // give particles some speed for smooth movement (friction will slow them down)
                    }
                }
            }
        }
        else {
            PartSys.sources[0].sourceFlags.perpetual = false; // emitted particles age
            // move all particles (again) to allow faster speeds
            for (uint32_t i = 0; i < PartSys.usedParticles; i++) {
                if (PartSys.particles[i].vx == 0)
                    PartSys.particles[i].vx = PartSys.sources[0].v; // move static particles (after disabling push mode)
                PartSys.particleMoveUpdate(PartSys.particles[i], PartSys.particleFlags[i], nullptr, &PartSys.advPartProps[i]);
            }
        }
        return true;
    }

private:
    uint32_t step{};
    uint16_t aux0{};
    uint16_t aux1{};
};
#endif // WLED_DISABLE_PARTICLESYSTEM1D
