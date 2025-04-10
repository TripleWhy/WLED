#pragma once
#ifndef WLED_DISABLE_PARTICLESYSTEM1D

#include "../../FX.h"
#include "../../FXparticleSystem.h"
#include "../BufferedEffect.h"
#include "../Effect.h"

/*
  Particle based AR effect, swoop particles along the strip with selected frequency loudness
  Uses palette for particle color
  by DedeHai (Damian Schneider)
*/
class Particle1dsonicstreamEffect : public BaseEffect<Particle1dsonicstreamEffect, BufferedEffect<EffectDimensionality::d1>> {
private:
    using Self = Particle1dsonicstreamEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d1>>;

public:
    static constexpr const char metaData[] PROGMEM = "PS Sonic Stream@!,!,Color,Blur,Bin,Mod,Filter,Push;,!;!;1f;c3=0,o2=1";
    static constexpr const uint8_t effectId = FX_MODE_PS_SONICSTREAM;

    explicit Particle1dsonicstreamEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        ParticleSystem1D *PartSys = nullptr;

        if (SEGMENT.call == 0) { // initialization
            if (!initParticleSystem1D(PartSys, 1, 255, 0, true)) // init, no additional data needed
                return mode_static(); // allocation failed or is single pixel
            PartSys->setKillOutOfBounds(true);
            PartSys->sources[0].source.x = 0; // at start
            //PartSys->sources[1].source.x = PartSys->maxX; // at end
            PartSys->sources[0].var = 0;//SEGMENT.custom1 >> 3;
            PartSys->sources[0].sat = 255;
        }
        else
            PartSys = reinterpret_cast<ParticleSystem1D *>(SEGENV.data); // if not first call, just set the pointer to the PS
        if (PartSys == nullptr)
            return mode_static(); // something went wrong, no data!

        // Particle System settings
        PartSys->updateSystem(); // update system properties (dimensions and data pointers)
        PartSys->setMotionBlur(20 + (SEGMENT.custom2 >> 1)); // anable motion blur
        PartSys->setSmearBlur(200); // smooth out the edges

        PartSys->sources[0].v = 5 + (SEGMENT.speed >> 2);

        // FFT processing
        um_data_t *um_data = getAudioData();
        uint8_t *fftResult = (uint8_t *)um_data->u_data[2]; // 16 bins with FFT data, log mapped already, each band contains frequency amplitude 0-255
        uint32_t loudness;
        uint32_t baseBin = SEGMENT.custom3 >> 1; // 0 - 15 map(SEGMENT.custom3, 0, 31, 0, 14);

        loudness = fftResult[baseBin];// + fftResult[baseBin + 1];
        int mids = sqrt16((int)fftResult[5] + (int)fftResult[6] + (int)fftResult[7] + (int)fftResult[8] + (int)fftResult[9] + (int)fftResult[10]); // average the mids, bin 5 is ~500Hz, bin 10 is ~2kHz (see audio_reactive.h)
        if (baseBin > 12)
            loudness = loudness << 2; // double loudness for high frequencies (better detecion)

        uint32_t threshold = 150 - (SEGMENT.intensity >> 1);
        if (SEGMENT.check2) { // enable low pass filter for dynamic threshold
            SEGMENT.step = (SEGMENT.step * 31500 + loudness * (32768 - 31500)) >> 15; // low pass filter for simple beat detection: add average to base threshold
            threshold = 20 + (threshold >> 1) + SEGMENT.step; // add average to threshold
        }

        // color
        uint32_t hueincrement = (SEGMENT.custom1 >> 3); // 0-31
        PartSys->setColorByPosition(SEGMENT.custom1 == 255);

        // particle manipulation
        for (uint32_t i = 0; i < PartSys->usedParticles; i++) {
            if (PartSys->sources[0].sourceFlags.perpetual == false) { // age faster if not perpetual
                if (PartSys->particles[i].ttl > 2) {
                    PartSys->particles[i].ttl -= 2; //ttl is linked to brightness, this allows to use higher brightness but still a short lifespan
                }
                else PartSys->particles[i].ttl = 0;
            }
            if (SEGMENT.check1) // modulate colors by mid frequencies
                PartSys->particles[i].hue += (mids * inoise8(PartSys->particles[i].x << 2, SEGMENT.step << 2)) >> 9; // color by perlin noise from mid frequencies
        }

        if (loudness > threshold) {
            SEGMENT.aux0 += hueincrement; // change color
            PartSys->sources[0].minLife = 100 + (((unsigned)SEGMENT.intensity * loudness * loudness) >> 13);
            PartSys->sources[0].maxLife = PartSys->sources[0].minLife;
            PartSys->sources[0].source.hue = SEGMENT.aux0;
            PartSys->sources[0].size = SEGMENT.speed;
            if (PartSys->particles[SEGMENT.aux1].x > 3 * PS_P_RADIUS_1D || PartSys->particles[SEGMENT.aux1].ttl == 0) { // only emit if last particle is far enough away or dead
                int partindex = PartSys->sprayEmit(PartSys->sources[0]); // emit a particle
                if (partindex >= 0) SEGMENT.aux1 = partindex; // track last emitted particle
            }
        }
        else loudness = 0; // required for push mode

        PartSys->update(); // update and render (needs to be done before manipulation for initial particle spacing to be right)

        if (SEGMENT.check3) { // push mode
            PartSys->sources[0].sourceFlags.perpetual = true; // emitted particles dont age
            PartSys->applyFriction(1); //slow down particles
            int32_t movestep = (((int)SEGMENT.speed + 2) * loudness) >> 10;
            if (movestep) {
                for (uint32_t i = 0; i < PartSys->usedParticles; i++) {
                    if (PartSys->particles[i].ttl) {
                        PartSys->particles[i].x += movestep; // push particles
                        PartSys->particles[i].vx = 10 + (SEGMENT.speed >> 4) ; // give particles some speed for smooth movement (friction will slow them down)
                    }
                }
            }
        }
        else {
            PartSys->sources[0].sourceFlags.perpetual = false; // emitted particles age
            // move all particles (again) to allow faster speeds
            for (uint32_t i = 0; i < PartSys->usedParticles; i++) {
                if (PartSys->particles[i].vx == 0)
                    PartSys->particles[i].vx = PartSys->sources[0].v; // move static particles (after disabling push mode)
                PartSys->particleMoveUpdate(PartSys->particles[i], PartSys->particleFlags[i], nullptr, &PartSys->advPartProps[i]);
            }
        }
    }

private:
};
#endif // WLED_DISABLE_PARTICLESYSTEM1D
