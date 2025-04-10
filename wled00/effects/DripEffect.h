#pragma once

#include <array>
#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"
#include "PopcornEffect.h"

/*
 * Drip Effect
 * ported of: https://www.youtube.com/watch?v=sru2fXh4r7k
 */
class DripEffect : public BaseEffect<DripEffect, BufferedEffect<EffectDimensionality::d2VStrips>> {
private:
    using Self = DripEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2VStrips>>;
    using Spark = PopcornEffect::Spark;
    static constexpr int maxNumDrops = 4;

public:
    static constexpr const char metaData[] PROGMEM = "Drip@Gravity,# of drips,,,,,Overlay;!,!;!;;m12=1";
    static constexpr const uint8_t effectId = FX_MODE_DRIP;

    explicit DripEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        if (!SEGMENT.check2)
            buffer.fill(SEGCOLOR(1));

        for (unsigned stripNr=0; stripNr<coordinate.height; stripNr++)
            runStrip(coordinate, stripNr, &drops[stripNr*maxNumDrops]);
    }

private:
    void runStrip(const EffectCoordinate& coordinate, uint16_t stripNr, Spark* drops) {
        unsigned numDrops = 1 + (SEGMENT.intensity >> 6); // 255>>6 = 3

        float gravity = -0.0005f - (SEGMENT.speed/50000.0f);
        gravity *= max(1, (int)coordinate.width-1);
        int sourcedrop = 12;

        for (unsigned j=0;j<numDrops;j++) {
            if (drops[j].colIndex == 0) { //init
                drops[j].pos = coordinate.width-1;    // start at end
                drops[j].vel = 0;           // speed
                drops[j].col = sourcedrop;  // brightness
                drops[j].colIndex = 1;      // drop state (0 init, 1 forming, 2 falling, 5 bouncing)
            }

            buffer.setPixelColor(coordinate.width-1, stripNr, color_blend(BLACK,SEGCOLOR(0), uint8_t(sourcedrop)));// water source
            if (drops[j].colIndex==1) {
                if (drops[j].col>255)
                    drops[j].col=255;
                buffer.setPixelColor(uint16_t(drops[j].pos), stripNr, color_blend(BLACK,SEGCOLOR(0),uint8_t(drops[j].col)));

                drops[j].col += map(SEGMENT.speed, 0, 255, 1, 6); // swelling

                if (hw_random8() < drops[j].col/10) {               // random drop
                    drops[j].colIndex=2;               //fall
                    drops[j].col=255;
                }
            }
            if (drops[j].colIndex > 1) {           // falling
                if (drops[j].pos > 0) {              // fall until end of segment
                    drops[j].pos += drops[j].vel;
                    if (drops[j].pos < 0) drops[j].pos = 0;
                    drops[j].vel += gravity;           // gravity is negative

                    for (int i=1;i<7-drops[j].colIndex;i++) { // some minor math so we don't expand bouncing droplets
                        unsigned pos = constrain(unsigned(drops[j].pos) +i, 0, coordinate.width-1); //this is BAD, returns a pos >= coordinate.width occasionally
                        buffer.setPixelColor(pos, stripNr, color_blend(BLACK,SEGCOLOR(0),uint8_t(drops[j].col/i))); //spread pixel with fade while falling
                    }

                    if (drops[j].colIndex > 2) {       // during bounce, some water is on the floor
                        buffer.setPixelColor(0, stripNr, color_blend(SEGCOLOR(0),BLACK,uint8_t(drops[j].col)));
                    }
                } else {                             // we hit bottom
                    if (drops[j].colIndex > 2) {       // already hit once, so back to forming
                        drops[j].colIndex = 0;
                        drops[j].col = sourcedrop;
                    } else {

                        if (drops[j].colIndex==2) {      // init bounce
                            drops[j].vel = -drops[j].vel/4;// reverse velocity with damping
                            drops[j].pos += drops[j].vel;
                        }
                        drops[j].col = sourcedrop*2;
                        drops[j].colIndex = 5;           // bouncing
                    }
                }
            }
        }
    }


private:
    std::array<Spark, maxNumDrops> drops{};
};


