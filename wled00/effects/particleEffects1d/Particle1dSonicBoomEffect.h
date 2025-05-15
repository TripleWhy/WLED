#pragma once
#ifndef WLED_DISABLE_PARTICLESYSTEM1D

#include "../../FX.h"
#include "../Effect.h"
#include "Particle1dEffect.h"

/*
  Particle based AR effect, creates exploding particles on beats
  by DedeHai (Damian Schneider)
*/
class Particle1dSonicBoomEffect : public BaseEffect<Particle1dSonicBoomEffect, Particle1dEffect<Particle1dSonicBoomEffect>> {
private:
    using Self = Particle1dSonicBoomEffect;
    using Base = BaseEffect<Self, Particle1dEffect<Self>>;

public:
    static constexpr const char* const metaData PROGMEM = "PS Sonic Boom@!,!,Color,Position,Bin,Mod,Filter,Blur;,!;!;1f;c2=63,c3=0,o2=1";
    static constexpr const uint8_t effectId = FX_MODE_PS1DSONICBOOM;

    using Base::Base;

    bool init(const EffectCoordinate& coordinate) {
        if (!Base::init(coordinate, 1, 255, true)) {
            return false;
        }

        PartSys.setKillOutOfBounds(true);
        return true;
    }

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        // Particle System settings
        PartSys.updateSystem(coordinate.width); // update system properties (dimensions and data pointers)
        PartSys.setMotionBlur(180 * parameters.check3);
        PartSys.setSmearBlur(64 * parameters.check3);
        PartSys.sources[0].var = map(parameters.speed, 0, 255, 10, 127);

        // FFT processing
        um_data_t *um_data = getAudioData();
        uint8_t *fftResult = (uint8_t *)um_data->u_data[2]; // 16 bins with FFT data, log mapped already, each band contains frequency amplitude 0-255
        uint32_t loudness;
        uint32_t baseBin = parameters.custom3 >> 1; // 0 - 15 map(parameters.custom3, 0, 31, 0, 14);
        loudness = fftResult[baseBin];// + fftResult[baseBin + 1];

        if (baseBin > 12)
            loudness = loudness << 2; // double loudness for high frequencies (better detecion)
        uint32_t threshold = 150 - (parameters.intensity >> 1);
        if (parameters.check2) { // enable low pass filter for dynamic threshold
            step = (step * 31500 + loudness * (32768 - 31500)) >> 15; // low pass filter for simple beat detection: add average to base threshold
            threshold = 20 + (threshold >> 1) + step; // add average to threshold
        }

        // particle manipulation
        for (uint32_t i = 0; i < PartSys.usedParticles; i++) {
            if (parameters.check1) { // modulate colors by mid frequencies
                int mids = sqrt32_bw((int)fftResult[5] + (int)fftResult[6] + (int)fftResult[7] + (int)fftResult[8] + (int)fftResult[9] + (int)fftResult[10]); // average the mids, bin 5 is ~500Hz, bin 10 is ~2kHz (see audio_reactive.h)
                PartSys.particles[i].hue += (mids * perlin8(PartSys.particles[i].x << 2, step << 2)) >> 9; // color by perlin noise from mid frequencies
            }
            if (PartSys.particles[i].ttl > 16) {
                PartSys.particles[i].ttl -= 16; //ttl is linked to brightness, this allows to use higher brightness but still a (very) short lifespan
            }
        }

        if (loudness > threshold) {
            if (aux1 == 0) { // edge detected, code only runs once per "beat"
                // update position
                if (parameters.custom2 < 128) // fixed position
                    PartSys.sources[0].source.x = map(parameters.custom2, 0, 127, 0, PartSys.maxX);
                else if (parameters.custom2 < 255) { // advances on each "beat"
                    int32_t step = PartSys.maxX / (((270 - parameters.custom2) >> 3)); // step: 2 - 33 steps for full segment width
                    PartSys.sources[0].source.x = (PartSys.sources[0].source.x + step) % PartSys.maxX;
                    if (PartSys.sources[0].source.x < step) // align to be symmetrical by making the first position half a step from start
                        PartSys.sources[0].source.x = step >> 1;
                }
                else // position set to max, use random postion per beat
                    PartSys.sources[0].source.x = hw_random(PartSys.maxX);

                // update color
                //PartSys.setColorByPosition(parameters.custom1 == 255);     // color slider at max: particle color by position
                PartSys.sources[0].sat = parameters.custom1 > 0 ? 255 : 0; // color slider at zero: set to white
                if (parameters.custom1 == 255) // emit color by position
                    aux0 = map(PartSys.sources[0].source.x , 0, PartSys.maxX, 0, 255);
                else if (parameters.custom1 > 0)
                    aux0 += (parameters.custom1 >> 1); // change emit color per "beat"
            }
            aux1 = 1; // track edge detection

            PartSys.sources[0].minLife = 200;
            PartSys.sources[0].maxLife = PartSys.sources[0].minLife + (((unsigned)parameters.intensity * loudness * loudness) >> 13);
            PartSys.sources[0].source.hue = aux0;
            PartSys.sources[0].size = 1; //parameters.speed>>3;
            uint32_t explosionsize = 4 + ((coordinate.width - 1) >> 2);
            explosionsize = hw_random16((explosionsize * loudness) >> 10);
            for (uint32_t e = 0; e < explosionsize; e++) { // emit explosion particles
                    PartSys.sprayEmit(PartSys.sources[0]); // emit a particle
                }
        }
        else
            aux1 = 0; // reset edge detection

        PartSys.update(buffer, parameters); // update and render (needs to be done before manipulation for initial particle spacing to be right)
        return true;
    }

private:
    uint32_t step{};
    uint16_t aux0{};
    uint16_t aux1{};
};


#endif //WLED_DISABLE_PARTICLESYSTEM1D
