#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

///////////////////////////////////////////
//   2D Cellular Automata Game of life   //
///////////////////////////////////////////
// Written by Ewoud Wijma, inspired by https://natureofcode.com/book/chapter-7-cellular-automata/ and https://github.com/DougHaber/nlife-color
class Gameoflife2dEffect : public BaseEffect<Gameoflife2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    struct ColorCount {
        CRGB color{};
        int8_t count{};
    };

private:
    using Self = Gameoflife2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;
    static constexpr int crcBufferLen = 2; //(coordinate.width + coordinate.height)*71/100; // roughly sqrt(2)/2 for better repetition detection (Ewowi)

public:
    static constexpr const char metaData[] PROGMEM = "Game Of Life@!;!,!;!;2";
    static constexpr const uint8_t effectId = FX_MODE_2DGAMEOFLIFE;

    explicit Gameoflife2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        const int cols = coordinate.width;
        const int rows = coordinate.height;
        const auto XY = [&](int x, int y) { return (x%cols) + (y%rows) * cols; };

        if (!prevLeds.resize(coordinate.width)) {
            return false;
        }

        CRGB backgroundColor = SEGCOLOR(1);

        if (parameters.call == 0 || strip.now - step > 3000) {
            step = strip.now;
            aux0 = 0;

            //give the leds random state and colors (based on intensity, colors from palette or all posible colors are chosen)
            for (int x = 0; x < cols; x++) for (int y = 0; y < rows; y++) {
                unsigned state = hw_random8()%2;
                if (state == 0)
                    buffer.setPixelColor(x,y, RGBW32(backgroundColor.r, backgroundColor.g, backgroundColor.b, 0));
                else
                    buffer.setPixelColor(x,y, parameters.color_from_palette(hw_random8(), false, PALETTE_SOLID_WRAP, 255));
            }

            std::fill(prevLeds.begin(), prevLeds.end(), CRGB::Black);
            crcBuffer.fill(0u);
        } else if (strip.now - step < FRAMETIME_FIXED * (uint32_t)map(parameters.speed,0,255,64,4)) {
            // update only when appropriate time passes (in 42 FPS slots)
        }

        //copy previous leds (save previous generation)
        //NOTE: using lossy getPixelColor() is a benefit as endlessly repeating patterns will eventually fade out causing a reset
        for (int x = 0; x < cols; x++)
            for (int y = 0; y < rows; y++)
                prevLeds[XY(x,y)] = buffer.getPixelColor(x,y);

        //calculate new leds
        for (int x = 0; x < cols; x++) {
            for (int y = 0; y < rows; y++) {
                ColorCount colorsCount[9]; // count the different colors in the 3*3 matrix
                for (int i=0; i<9; i++)
                    colorsCount[i] = {backgroundColor, 0}; // init colorsCount

                // iterate through neighbors and count them and their different colors
                int neighbors = 0;
                for (int i = -1; i <= 1; i++) {
                    for (int j = -1; j <= 1; j++) { // iterate through 3*3 matrix
                        if (i==0 && j==0)
                            continue; // ignore itself
                        // wrap around segment
                        int xx = x+i, yy = y+j;
                        if (x+i < 0)
                            xx = cols-1;
                        else if (x+i >= cols)
                            xx = 0;
                        if (y+j < 0)
                            yy = rows-1;
                        else if (y+j >= rows)
                            yy = 0;

                        unsigned xy = XY(xx, yy); // previous cell xy to check
                        // count different neighbours and colors
                        if (prevLeds[xy] != backgroundColor) {
                            neighbors++;
                            bool colorFound = false;
                            int k;
                            for (k=0; k<9 && colorsCount[k].count != 0; k++)
                                if (colorsCount[k].color == prevLeds[xy]) {
                                    colorsCount[k].count++;
                                    colorFound = true;
                                }
                            if (!colorFound)
                                colorsCount[k] = {prevLeds[xy], 1}; //add new color found in the array
                        }
                    } // j
                } // i

                // Rules of Life
                uint32_t col = uint32_t(prevLeds[XY(x,y)]) & 0x00FFFFFF;  // uint32_t operator returns RGBA, we want RGBW -> cut off "alpha" byte
                uint32_t bgc = RGBW32(backgroundColor.r, backgroundColor.g, backgroundColor.b, 0);
                if      ((col != bgc) && (neighbors <  2))
                    buffer.setPixelColor(x,y, bgc); // Loneliness
                else if ((col != bgc) && (neighbors >  3))
                    buffer.setPixelColor(x,y, bgc); // Overpopulation
                else if ((col == bgc) && (neighbors == 3)) {                                  // Reproduction
                    // find dominant color and assign it to a cell
                    ColorCount dominantColorCount{backgroundColor, 0};
                    for (int i=0; i<9 && colorsCount[i].count != 0; i++)
                        if (colorsCount[i].count > dominantColorCount.count)
                            dominantColorCount = colorsCount[i];
                    // assign the dominant color w/ a bit of randomness to avoid "gliders"
                    if (dominantColorCount.count > 0 && hw_random8(128))
                        buffer.setPixelColor(x,y, RGBW32(dominantColorCount.color.r, dominantColorCount.color.g, dominantColorCount.color.b, 0));
                } else if ((col == bgc) && (neighbors == 2) && !hw_random8(128)) {               // Mutation
                    buffer.setPixelColor(x,y, parameters.color_from_palette(hw_random8(), false, PALETTE_SOLID_WRAP, 255));
                }
                // else do nothing!
            } // y
        } // x

        // calculate CRC16 of leds
        uint16_t crc = crc16(reinterpret_cast<const unsigned char*>(&prevLeds.front()), prevLeds.size() * sizeof(CRGB));
        // check if we had same CRC and reset if needed
        bool repetition = false;
        for (int i=0; i<crcBufferLen && !repetition; i++)
            repetition = (crc == crcBuffer[i]); // (Ewowi)
        // same CRC would mean image did not change or was repeating itself
        if (!repetition)
            step = strip.now; //if no repetition avoid reset
        // remember CRCs across frames
        crcBuffer[aux0] = crc;
        ++aux0 %= crcBufferLen;
        return true;
    }

private:
    SegmentAllocator<CRGB>::vector prevLeds{};
    std::array<uint16_t, crcBufferLen> crcBuffer{};
    uint32_t step{};
    uint16_t aux0{};
};


#endif //WLED_DISABLE_2D
