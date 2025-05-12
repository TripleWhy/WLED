#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
*  POPCORN
*  modified from https://github.com/kitesurfer1404/WS2812FX/blob/master/src/custom/Popcorn.h
*/
class PopcornEffect : public BaseEffect<PopcornEffect, BufferedEffect<EffectDimensionality::d2VStrips>> {
public:
    static constexpr unsigned maxNumPopcorn = 21; // max 21 on 16 segment ESP8266

    //Spark type is used for popcorn, 1D fireworks, and drip
    struct Spark {
        float pos{};
        float posX{};
        float vel{};
        float velX{};
        uint16_t col{};
        uint8_t colIndex{};
    };

private:
    using Self = PopcornEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2VStrips>>;

public:
    static constexpr const char metaData[] PROGMEM = "Popcorn@!,!,,,,,Overlay;!,!,!;!;;m12=1";
    static constexpr const uint8_t effectId = FX_MODE_POPCORN;

    explicit PopcornEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        //allocate segment data
        unsigned strips = coordinate.height;
        unsigned usablePopcorns = maxNumPopcorn;
        if (usablePopcorns * strips * sizeof(Spark) > FAIR_DATA_PER_SEG)
            usablePopcorns = FAIR_DATA_PER_SEG / (strips * sizeof(Spark)) + 1; // at least 1 popcorn per vstrip

        if (!popcorn.resize(usablePopcorns)) { // on a matrix 64x64 this could consume a little less than 27kB when Bar expansion is used
            return false;
        }

        bool hasCol2 = SEGCOLOR(2);
        if (!parameters.check2)
            buffer.fill(hasCol2 ? BLACK : SEGCOLOR(1));

        for (unsigned stripNr=0; stripNr<strips; stripNr++)
            runStrip(parameters, coordinate, stripNr, &popcorn[stripNr * usablePopcorns], usablePopcorns);
        return true;
    }

private:
    void runStrip(TransitionableParameters& parameters, const EffectCoordinate& coordinate, uint16_t stripNr, Spark* popcorn, unsigned usablePopcorns) {
        float gravity = -0.0001f - (parameters.speed/200000.0f); // m/s/s
        gravity *= coordinate.width;

        unsigned numPopcorn = parameters.intensity * usablePopcorns / 255;
        if (numPopcorn == 0)
            numPopcorn = 1;

        for (unsigned i = 0; i < numPopcorn; i++) {
            if (popcorn[i].pos >= 0.0f) { // if kernel is active, update its position
                popcorn[i].pos += popcorn[i].vel;
                popcorn[i].vel += gravity;
            } else { // if kernel is inactive, randomly pop it
                if (hw_random8() < 2) { // POP!!!
                    popcorn[i].pos = 0.01f;

                    unsigned peakHeight = 128 + hw_random8(128); //0-255
                    peakHeight = (peakHeight * (coordinate.width -1)) >> 8;
                    popcorn[i].vel = sqrtf(-2.0f * gravity * peakHeight);

                    if (SEGMENT.palette) {
                        popcorn[i].colIndex = hw_random8();
                    } else {
                        byte col = hw_random8(0, NUM_COLORS);
                        if (!SEGCOLOR(2) || !SEGCOLOR(col)) col = 0;
                        popcorn[i].colIndex = col;
                    }
                }
            }
            if (popcorn[i].pos >= 0.0f) { // draw now active popcorn (either active before or just popped)
                uint32_t col = parameters.color_wheel(popcorn[i].colIndex);
                if (!SEGMENT.palette && popcorn[i].colIndex < NUM_COLORS)
                    col = SEGCOLOR(popcorn[i].colIndex);
                unsigned ledIndex = popcorn[i].pos;
                if (ledIndex < coordinate.width)
                    buffer.setPixelColor(ledIndex, stripNr, col);
            }
        }
    }

private:
    SegmentAllocator<Spark>::vector popcorn{};
};


