#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"
#include "PopcornEffect.h"

/*
 * Exploding fireworks effect
 * adapted from: http://www.anirama.com/1000leds/1d-fireworks/
 * adapted for 2D WLED by blazoncek (Blaz Kristan (AKA blazoncek))
 */
class ExplodingFireworksEffect : public BaseEffect<ExplodingFireworksEffect, BufferedEffect<EffectDimensionality::d2VStrips>> {
private:
    using Self = ExplodingFireworksEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2VStrips>>;
    using Spark = PopcornEffect::Spark;

public:
    static constexpr const char metaData[] PROGMEM = "Fireworks 1D@Gravity,Firing side;!,!;!;12;pal=11,ix=128";
    static constexpr const uint8_t effectId = FX_MODE_EXPLODING_FIREWORKS;

    explicit ExplodingFireworksEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        //allocate segment data
        unsigned maxData = FAIR_DATA_PER_SEG; //ESP8266: 256 ESP32: 640
        unsigned segs = strip.getActiveSegmentsNum();
        if (segs <= (strip.getMaxSegments() /2)) maxData *= 2; //ESP8266: 512 if <= 8 segs ESP32: 1280 if <= 16 segs
        if (segs <= (strip.getMaxSegments() /4)) maxData *= 2; //ESP8266: 1024 if <= 4 segs ESP32: 2560 if <= 8 segs
        int maxSparks = maxData / sizeof(Spark); //ESP8266: max. 21/42/85 sparks/seg, ESP32: max. 53/106/213 sparks/seg

        unsigned numSparks = min(5 + ((rows*cols) >> 1), maxSparks);
        if (!sparks.resize(numSparks)) {
            return false;
        }
        sparks.shrink_to_fit();

        if (numSparks != aux1) { //reset to flare if sparks were reallocated (it may be good idea to reset segment if bounds change)
            dying_gravity = 0.0f;
            aux0 = 0;
            aux1 = numSparks;
        }

        buffer.fade(SEGCOLOR(1), 252);

        Spark& flare = sparks.front(); //first spark is flare data
        float gravity = -0.0004f - (parameters.speed/800000.0f); // m/s/s
        gravity *= rows;

        if (aux0 < 2) { //FLARE
            if (aux0 == 0) { //init flare
                flare.pos = 0;
                flare.posX = hw_random16(2,cols-3);
                unsigned peakHeight = 75 + hw_random8(180); //0-255
                peakHeight = (peakHeight * (rows -1)) >> 8;
                flare.vel = sqrtf(-2.0f * gravity * peakHeight);
                flare.velX = (hw_random8(9)-4)/64.0f;
                flare.col = 255; //brightness
                aux0 = 1;
            }

            // launch
            if (flare.vel > 12 * gravity) {
                // flare
                buffer.setPixelColor(unsigned(flare.posX), rows - uint16_t(flare.pos) - 1, RGBW32(flare.col, flare.col, flare.col, 0));
                flare.pos  += flare.vel;
                flare.pos  = constrain(flare.pos, 0, rows-1);
                flare.posX += flare.velX;
                flare.posX = constrain(flare.posX, 0, cols-1);
                flare.vel  += gravity;
                flare.col  -= 2;
            } else {
                aux0 = 2;  // ready to explode
            }
        } else if (aux0 < 4) {
            /*
             * Explode!
             *
             * Explosion happens where the flare ended.
             * Size is proportional to the height.
             */
            unsigned nSparks = flare.pos + hw_random8(4);
            nSparks = std::max(nSparks, 4U);  // This is not a standard constrain; numSparks is not guaranteed to be at least 4
            nSparks = std::min(nSparks, numSparks);

            // initialize sparks
            if (aux0 == 2) {
                for (unsigned i = 1; i < nSparks; i++) {
                    sparks[i].pos  = flare.pos;
                    sparks[i].posX = flare.posX;
                    sparks[i].vel  = (float(hw_random16(20001)) / 10000.0f) - 0.9f; // from -0.9 to 1.1
                    sparks[i].vel *= rows<32 ? 0.5f : 1; // reduce velocity for smaller strips
                    sparks[i].velX = (float(hw_random16(20001)) / 10000.0f) - 1.0f; // from -1 to 1
                    sparks[i].col  = 345;//abs(sparks[i].vel * 750.0); // set colors before scaling velocity to keep them bright
                    //sparks[i].col = constrain(sparks[i].col, 0, 345);
                    sparks[i].colIndex = hw_random8();
                    sparks[i].vel  *= flare.pos/rows; // proportional to height
                    sparks[i].velX *= flare.posX/cols; // proportional to width
                    sparks[i].vel  *= -gravity *50;
                }
                //sparks[1].col = 345; // this will be our known spark
                dying_gravity = gravity/2;
                aux0 = 3;
            }

            if (sparks[1].col > 4) {//&& sparks[1].pos > 0) { // as long as our known spark is lit, work with all the sparks
                for (unsigned i = 1; i < nSparks; i++) {
                    sparks[i].pos  += sparks[i].vel;
                    sparks[i].posX += sparks[i].velX;
                    sparks[i].vel  += dying_gravity;
                    sparks[i].velX += dying_gravity;
                    if (sparks[i].col > 3)
                        sparks[i].col -= 4;

                    if (sparks[i].pos > 0 && sparks[i].pos < rows) {
                        if (!(sparks[i].posX >= 0 && sparks[i].posX < cols))
                            continue;
                        unsigned prog = sparks[i].col;
                        uint32_t spColor = (SEGMENT.palette) ? parameters.color_wheel(sparks[i].colIndex) : SEGCOLOR(0);
                        CRGBW c = BLACK; //HeatColor(sparks[i].col);
                        if (prog > 300) { //fade from white to spark color
                            c = color_blend(spColor, WHITE, uint8_t((prog - 300)*5));
                        } else if (prog > 45) { //fade from spark color to black
                            c = color_blend(BLACK, spColor, uint8_t(prog - 45));
                            unsigned cooling = (300 - prog) >> 5;
                            c.g = qsub8(c.g, cooling);
                            c.b = qsub8(c.b, cooling * 2);
                        }
                        buffer.setPixelColor(int(sparks[i].posX), rows - int(sparks[i].pos) - 1, c);
                    }
                }
                if (parameters.check3)
                    buffer.blur(16);
                dying_gravity *= .8f; // as sparks burn out they fall slower
            } else {
                aux0 = 6 + hw_random8(10); //wait for this many frames
            }
        } else {
            aux0--;
            if (aux0 < 4) {
                aux0 = 0; //back to flare
            }
        }

        return false;
    }

private:
    SegmentAllocator<Spark>::vector sparks{};
    float dying_gravity{};
    uint16_t aux0{};
    uint16_t aux1{};
};

#endif //WLED_DISABLE_2D
