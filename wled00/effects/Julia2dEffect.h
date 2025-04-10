#pragma once
#ifndef WLED_DISABLE_2D

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/////////////////////////
//     2D Julia        //
/////////////////////////
// Sliders are:
// intensity = Maximum number of iterations per pixel.
// Custom1 = Location of X centerpoint
// Custom2 = Location of Y centerpoint
// Custom3 = Size of the area (small value = smaller area)
// An animated Julia set by Andrew Tuline.
class Julia2dEffect : public BaseEffect<Julia2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = Julia2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "Julia@,Max iterations per pixel,X center,Y center,Area size, Blur;!;!;2;ix=24,c1=128,c2=128,c3=16";
    static constexpr const uint8_t effectId = FX_MODE_2DJULIA;

    explicit Julia2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
        Base::nextFrameImpl(coordinate);

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        float reAl;
        float imAg;

        if (SEGENV.call == 0) {           // Reset the center if we've just re-started this animation.
            xcen = 0.;
            ycen = 0.;
            xymag = 1.0;

            SEGMENT.custom1 = 128;              // Make sure the location widgets are centered to start.
            SEGMENT.custom2 = 128;
            SEGMENT.custom3 = 16;
            SEGMENT.intensity = 24;
        }

        xcen  = xcen  + (float)(SEGMENT.custom1 - 128)/100000.f;
        ycen  = ycen  + (float)(SEGMENT.custom2 - 128)/100000.f;
        xymag = xymag + (float)((SEGMENT.custom3 - 16)<<3)/100000.f; // reduced resolution slider
        if (xymag < 0.01f) xymag = 0.01f;
        if (xymag > 1.0f) xymag = 1.0f;

        float xmin = xcen - xymag;
        float xmax = xcen + xymag;
        float ymin = ycen - xymag;
        float ymax = ycen + xymag;

        // Whole set should be within -1.2,1.2 to -.8 to 1.
        xmin = constrain(xmin, -1.2f, 1.2f);
        xmax = constrain(xmax, -1.2f, 1.2f);
        ymin = constrain(ymin, -0.8f, 1.0f);
        ymax = constrain(ymax, -0.8f, 1.0f);

        float dx;                       // Delta x is mapped to the matrix size.
        float dy;                       // Delta y is mapped to the matrix size.

        int maxIterations = 15;         // How many iterations per pixel before we give up. Make it 8 bits to match our range of colours.
        float maxCalc = 16.0;           // How big is each calculation allowed to be before we give up.

        maxIterations = SEGMENT.intensity/2;


        // Resize section on the fly for some animaton.
        reAl = -0.94299f;               // PixelBlaze example
        imAg = 0.3162f;

        reAl += (float)sin16_t(strip.now * 34) / 655340.f;
        imAg += (float)sin16_t(strip.now * 26) / 655340.f;

        dx = (xmax - xmin) / (cols);     // Scale the delta x and y values to our matrix size.
        dy = (ymax - ymin) / (rows);

        // Start y
        float y = ymin;
        for (int j = 0; j < rows; j++) {

            // Start x
            float x = xmin;
            for (int i = 0; i < cols; i++) {

                // Now we test, as we iterate z = z^2 + c does z tend towards infinity?
                float a = x;
                float b = y;
                int iter = 0;

                while (iter < maxIterations) {    // Here we determine whether or not we're out of bounds.
                    float aa = a * a;
                    float bb = b * b;
                    float len = aa + bb;
                    if (len > maxCalc) {            // |z| = sqrt(a^2+b^2) OR z^2 = a^2+b^2 to save on having to perform a square root.
                        break;  // Bail
                    }

                 // This operation corresponds to z -> z^2+c where z=a+ib c=(x,y). Remember to use 'foil'.
                    b = 2*a*b + imAg;
                    a = aa - bb + reAl;
                    iter++;
                } // while

                // We color each pixel based on how long it takes to get to infinity, or black if it never gets there.
                if (iter == maxIterations) {
                    buffer.setPixelColor(i, j, 0);
                } else {
                    buffer.setPixelColor(i, j, SEGMENT.color_from_palette(iter*255/maxIterations, false, PALETTE_SOLID_WRAP, 0));
                }
                x += dx;
            }
            y += dy;
        }
        if(SEGMENT.check1)
            buffer.blur(100, true);
    }

private:
    float xcen{};
    float ycen{};
    float xymag{};
};


#endif //WLED_DISABLE_2D
