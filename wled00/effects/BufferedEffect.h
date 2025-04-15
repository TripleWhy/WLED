#pragma once

#include "../FX.h"
#include "../wled.h"
#include "../src/font/console_font_4x6.h"
#include "../src/font/console_font_5x12.h"
#include "../src/font/console_font_5x8.h"
#include "../src/font/console_font_6x8.h"
#include "../src/font/console_font_7x9.h"
#include "Effect.h"

// The buffered Effect is split into two parts:
//  1. BufferedEffectBase contains everything that doesn't depend on a template parameter
//  2. BufferedEffect contains everything that depends on a template parameter.
// Don't use BufferedEffectBase directly, use BufferedEffect.

class BufferedEffectBase : public Effect {
public:
    static constexpr bool verifyBufferBounds = true;
    static constexpr bool verifyXyBounds = true;

    class PixelBufferBase {
        template<EffectDimensionality>
        friend class BufferedEffect;
    public:
        explicit PixelBufferBase() = default;
        explicit PixelBufferBase(size_t size) {
            resizeVector(pixels, size);
        }

        inline bool isEmpty() const {
            return pixels.empty();
        }

        void fill(uint32_t color) {
            std::fill(pixels.begin(), pixels.end(), color);
        }

        inline void fadeOut(uint8_t rate) {
            fade(SEGCOLOR(1), rate);
        }

        /*
         * fade out function, higher rate = quicker fade
         * fading is highly dependant on frame rate (higher frame rates, faster fading)
         * each frame will fade at max 9% or as little as 0.8%
         */
        //TODO there is room for optimization here
        void fade(uint32_t targetColor, uint8_t rate) {
            rate = (256-rate) >> 1;
            const int mappedRate = 256 / (rate + 1);

            for (uint32_t& color : pixels) {
                if (color == targetColor) {
                    continue; // already at target color
                }
                for (int i = 0; i < 32; i += 8) {
                    uint8_t c2 = (targetColor>>i);  // get background channel
                    uint8_t c1 = (color>>i);      // get foreground channel
                    // we can't use bitshift since we are using int
                    int delta = (c2 - c1) * mappedRate / 256;
                    // if fade isn't complete, make sure delta is at least 1 (fixes rounding issues)
                    if (delta == 0) {
                        delta += (c2 == c1) ? 0 : (c2 > c1) ? 1 : -1;
                    }
                    // stuff new value back into color
                    color &= ~(0xFF<<i);
                    color |= ((c1 + delta) & 0xFF) << i;
                }
            }
        }

        void fade(uint8_t scale) {
            if (scale == 0u)
                return;   // optimization - no scaling to apply

            for (uint32_t& color : pixels) {
                color = color_fade(color, scale);
            }
        }

        inline void fadeToBlackBy(uint8_t fadeBy) {
            fade(255 - fadeBy);
        }

        void add(uint32_t color, bool preserveCR = true) {
            for (uint32_t& pixel : pixels) {
                pixel = color_add(pixel, color, preserveCR);
            }
        }

    protected:
        inline uint32_t getPixelColorLinear(unsigned i) const {
            if constexpr (verifyBufferBounds) {
                if (static_cast<size_t>(i) >= pixels.size()) [[unlikely]] {
                    Serial.printf("BufferedEffect::getPixelColorLinear: %u >= %u\n", i, pixels.size());
                    std::terminate();
                }
            }
            return pixels[static_cast<size_t>(i)];
        }
        inline void setPixelColorLinear(unsigned i, uint32_t c) {
            if constexpr (verifyBufferBounds) {
                if (static_cast<size_t>(i) >= pixels.size()) [[unlikely]] {
                    Serial.printf("BufferedEffect::setPixelColorLinear: %u >= %u\n", i, pixels.size());
                    std::terminate();
                }
            }
            pixels[static_cast<size_t>(i)] = c;
        }
        inline void blendPixelColorLinear(unsigned n, uint32_t color, uint8_t blend) {
            setPixelColorLinear(n, color_blend(getPixelColorLinear(n), color, blend));
        }
        inline void addPixelColorLinear(unsigned n, uint32_t color, bool preserveCR = true) {
            setPixelColorLinear(n, color_add(getPixelColorLinear(n), color, preserveCR));
        }
        inline void fadePixelColorLinear(unsigned n, uint8_t fade) {
            setPixelColorLinear(n, color_fade(getPixelColorLinear(n), fade, true));
        }

    private:
        SegmentAllocator<uint32_t>::vector pixels;
    };

private:
    using Self = BufferedEffectBase;
    using Base = Effect;

    template<EffectDimensionality>
    friend class BufferedEffect;

protected:
    using Base::Base;
};

template<EffectDimensionality _dimensionality>
class BufferedEffect : public BufferedEffectBase {
public:
    class PixelBuffer : public PixelBufferBase {
    public:
        explicit constexpr PixelBuffer() = default;
        explicit constexpr PixelBuffer(size_t length) : PixelBufferBase(length) {
            static_assert(dimensionality == EffectDimensionality::d1, "Use more coordinate arguments.");
        }
        explicit constexpr PixelBuffer(size_t width, size_t height) : PixelBufferBase(width * height) {
            static_assert(dimensionality != EffectDimensionality::d1, "Use fewer coordinate arguments.");
        }

        inline void setPixelColor(unsigned x, uint32_t color) {
            static_assert(dimensionality == EffectDimensionality::d1, "Use more coordinate arguments.");
            setPixelColorLinear(x, color);
        }
        inline void setPixelColor(unsigned x, CRGB c) {
            setPixelColor(x, RGBW32(c.r,c.g,c.b,0));
        }

        inline void setPixelColor(unsigned x, unsigned y, uint32_t color) {
            static_assert(dimensionality != EffectDimensionality::d1, "Use fewer coordinate arguments.");
            setPixelColorLinear(convertToLinear(x, y), color);
        }
        inline void setPixelColor(unsigned x, unsigned y, CRGB c) {
            setPixelColor(x, y, RGBW32(c.r,c.g,c.b,0));
        }

        inline uint32_t getPixelColor(unsigned x) const {
            static_assert(dimensionality == EffectDimensionality::d1, "Use more coordinate arguments.");
            return getPixelColorLinear(x);
        }

        inline uint32_t getPixelColor(unsigned x, unsigned y) const {
            static_assert(dimensionality != EffectDimensionality::d1, "Use fewer coordinate arguments.");
            return getPixelColorLinear(convertToLinear(x, y));
        }

        inline void blendPixelColor(unsigned x, uint32_t color, uint8_t blend) {
            static_assert(dimensionality == EffectDimensionality::d1, "Use more coordinate arguments.");
            blendPixelColorLinear(x, color, blend);
        }

        inline void blendPixelColor(unsigned x, unsigned y, uint32_t color, uint8_t blend) {
            static_assert(dimensionality != EffectDimensionality::d1, "Use fewer coordinate arguments.");
            blendPixelColorLinear(convertToLinear(x, y), color, blend);
        }

        inline void addPixelColor(unsigned x, uint32_t color, bool preserveCR = true) {
            static_assert(dimensionality == EffectDimensionality::d1, "Use more coordinate arguments.");
            addPixelColorLinear(x, color, preserveCR);
        }

        inline void addPixelColor(unsigned x, unsigned y, uint32_t color, bool preserveCR = true) {
            static_assert(dimensionality != EffectDimensionality::d1, "Use fewer coordinate arguments.");
            addPixelColorLinear(convertToLinear(x, y), color, preserveCR);
        }

        inline void fadePixelColor(unsigned x, uint8_t fade) {
            static_assert(dimensionality == EffectDimensionality::d1, "Use more coordinate arguments.");
            fadePixelColorLinear(x, fade);
        }

        inline void fadePixelColor(unsigned x, unsigned y, uint8_t fade) {
            static_assert(dimensionality != EffectDimensionality::d1, "Use fewer coordinate arguments.");
            fadePixelColorLinear(convertToLinear(x, y), fade);
        }

        inline void blur(uint8_t blur_amount, bool smear = false) {
            if constexpr (dimensionality == EffectDimensionality::d1) {
                blur1d(blur_amount, smear);
            } else {
                blur2d(blur_amount, blur_amount, smear); // symmetrical 2D blur
            }
        }

        /*
         * blurs segment content, source: FastLED colorutils.cpp
         * Note: for blur_amount > 215 this function does not work properly (creates alternating pattern)
         */
        void blur1d(uint8_t blur_amount, bool smear = false, unsigned start = 0, unsigned end = std::numeric_limits<unsigned>::max()) {
            static_assert(dimensionality == EffectDimensionality::d1, "This function is for 1D effects only.");

            if (blur_amount == 0) {
                return; // optimization: 0 means "don't blur"
            }
            uint8_t keep = smear ? 255 : 255 - blur_amount;
            uint8_t seep = blur_amount >> 1;
            uint32_t carryover = BLACK;
            uint32_t lastnew;       // not necessary to initialize lastnew and last, as both will be initialized by the first loop iteration
            uint32_t last;
            uint32_t curnew = BLACK;
            end = std::min(end, pixels.size());
            for (unsigned i = 0; i < end; i++) {
                uint32_t cur = getPixelColor(i);
                uint32_t part = color_fade(cur, seep);
                curnew = color_fade(cur, keep);
                if (i > 0) {
                    if (carryover)
                        curnew = color_add(curnew, carryover);
                    uint32_t prev = color_add(lastnew, part);
                    // optimization: only set pixel if color has changed
                    if (last != prev)
                        setPixelColor(i - 1, prev);
                } else {
                    setPixelColor(i, curnew); // first pixel
                }
                lastnew = curnew;
                last = cur; // save original value for comparison on next iteration
                carryover = part;
            }
            setPixelColor(end - 1, curnew);
        }

        // 2D blurring, can be asymmetrical
        void blur2d(uint8_t blur_x, uint8_t blur_y, bool smear = false) {
            static_assert(dimensionality != EffectDimensionality::d1, "This function is for 2D effects only.");

            const unsigned cols = Segment::getEffectWidth<dimensionality>();
            const unsigned rows = Segment::getEffectHeight<dimensionality>();
            uint32_t lastnew;   // not necessary to initialize lastnew and last, as both will be initialized by the first loop iteration
            uint32_t last;
            if (blur_x) {
                const uint8_t keepx = smear ? 255 : 255 - blur_x;
                const uint8_t seepx = blur_x >> 1;
                for (unsigned row = 0; row < rows; row++) { // blur rows (x direction)
                    uint32_t carryover = BLACK;
                    uint32_t curnew = BLACK;
                    for (unsigned x = 0; x < cols; x++) {
                        uint32_t cur = getPixelColor(x, row);
                        uint32_t part = color_fade(cur, seepx);
                        curnew = color_fade(cur, keepx);
                        if (x > 0) {
                            if (carryover) curnew = color_add(curnew, carryover);
                            uint32_t prev = color_add(lastnew, part);
                            // optimization: only set pixel if color has changed
                            if (last != prev) setPixelColor(x - 1, row, prev);
                        } else setPixelColor(x, row, curnew); // first pixel
                        lastnew = curnew;
                        last = cur; // save original value for comparison on next iteration
                        carryover = part;
                    }
                    setPixelColor(cols-1, row, curnew); // set last pixel
                }
            }
            if (blur_y) {
                const uint8_t keepy = smear ? 255 : 255 - blur_y;
                const uint8_t seepy = blur_y >> 1;
                for (unsigned col = 0; col < cols; col++) {
                    uint32_t carryover = BLACK;
                    uint32_t curnew = BLACK;
                    for (unsigned y = 0; y < rows; y++) {
                        uint32_t cur = getPixelColor(col, y);
                        uint32_t part = color_fade(cur, seepy);
                        curnew = color_fade(cur, keepy);
                        if (y > 0) {
                            if (carryover) curnew = color_add(curnew, carryover);
                            uint32_t prev = color_add(lastnew, part);
                            // optimization: only set pixel if color has changed
                            if (last != prev) setPixelColor(col, y - 1, prev);
                        } else setPixelColor(col, y, curnew); // first pixel
                        lastnew = curnew;
                        last = cur; //save original value for comparison on next iteration
                        carryover = part;
                    }
                    setPixelColor(col, rows - 1, curnew);
                }
            }
        }

        // move() - move all pixels in desired direction delta number of pixels
        // @param dir direction: 0=left, 1=left-up, 2=up, 3=right-up, 4=right, 5=right-down, 6=down, 7=left-down
        // @param delta number of pixels to move
        // @param wrap around
        void movePixels(unsigned dir, unsigned delta, bool wrap = false) {
            static_assert(dimensionality != EffectDimensionality::d1, "This function is for 2D effects only.");

            switch (dir) {
            case 0: movePixelsX( delta, wrap);                            break;
            case 1: movePixelsX( delta, wrap); movePixelsY( delta, wrap); break;
            case 2:                            movePixelsY( delta, wrap); break;
            case 3: movePixelsX(-delta, wrap); movePixelsY( delta, wrap); break;
            case 4: movePixelsX(-delta, wrap);                            break;
            case 5: movePixelsX(-delta, wrap); movePixelsY(-delta, wrap); break;
            case 6:                            movePixelsY(-delta, wrap); break;
            case 7: movePixelsX( delta, wrap); movePixelsY(-delta, wrap); break;
            }
        }

        //TODO this can probably be optimized
        void movePixelsX(int delta, bool wrap) {
            static_assert(dimensionality != EffectDimensionality::d1, "This function is for 2D effects only.");

            if (delta == 0)
                return; // not active

            const int vW = Segment::getEffectWidth<dimensionality>();   // segment width in logical pixels (can be 0 if segment is inactive)
            const int vH = Segment::getEffectHeight<dimensionality>();  // segment height in logical pixels (is always >= 1)
            int absDelta = abs(delta);
            if (absDelta >= vW)
                return;
            uint32_t newPxCol[vW];
            int newDelta;
            int stop = vW;
            int start = 0;
            if (wrap)
                newDelta = (delta + vW) % vW; // +cols in case delta < 0
            else {
                if (delta < 0)
                    start = absDelta;
                stop = vW - absDelta;
                newDelta = delta > 0 ? delta : 0;
            }
            for (int y = 0; y < vH; y++) {
                for (int x = 0; x < stop; x++) {
                    int srcX = x + newDelta;
                    if (wrap)
                        srcX %= vW; // Wrap using modulo when `wrap` is true
                    newPxCol[x] = getPixelColor(srcX, y);
                }
                for (int x = 0; x < stop; x++)
                    setPixelColor(x + start, y, newPxCol[x]);
            }
        }

        //TODO this can probably be optimized
        void movePixelsY(int delta, bool wrap) {
            static_assert(dimensionality != EffectDimensionality::d1, "This function is for 2D effects only.");

            if (delta == 0)
                return; // not active

            const int vW = Segment::getEffectWidth<dimensionality>();   // segment width in logical pixels (can be 0 if segment is inactive)
            const int vH = Segment::getEffectHeight<dimensionality>();  // segment height in logical pixels (is always >= 1)
            int absDelta = abs(delta);
            if (absDelta >= vH)
                return;
            uint32_t newPxCol[vH];
            int newDelta;
            int stop = vH;
            int start = 0;
            if (wrap)
                newDelta = (delta + vH) % vH; // +rows in case delta < 0
            else {
                if (delta < 0) start = absDelta;
                stop = vH - absDelta;
                newDelta = delta > 0 ? delta : 0;
            }
            for (int x = 0; x < vW; x++) {
                for (int y = 0; y < stop; y++) {
                    int srcY = y + newDelta;
                    if (wrap)
                        srcY %= vH; // Wrap using modulo when `wrap` is true
                    newPxCol[y] = getPixelColor(x, srcY);
                }
                for (int y = 0; y < stop; y++)
                    setPixelColor(x, y + start, newPxCol[y]);
            }
        }

        void drawCircle(uint16_t cx, uint16_t cy, uint8_t radius, uint32_t col, bool soft = false) {
            static_assert(dimensionality != EffectDimensionality::d1, "This function is for 2D effects only.");

            if (radius == 0)
                return; // not active
            if (soft) {
                // Xiaolin Wu’s algorithm
                const int rsq = radius*radius;
                int x = 0;
                int y = radius;
                unsigned oldFade = 0;
                while (x < y) {
                    float yf = sqrtf(float(rsq - x*x)); // needs to be floating point
                    uint8_t fade = float(0xFF) * (ceilf(yf) - yf); // how much color to keep
                    if (oldFade > fade)
                        y--;
                    oldFade = fade;
                    int px, py;
                    for (uint8_t i = 0; i < 16; i++) {
                            int swaps = (i & 0x4 ? 1 : 0); // 0,  0,  0,  0,  1,  1,  1,  1,  0,  0,  0,  0,  1,  1,  1,  1
                            int adj =  (i < 8) ? 0 : 1;    // 0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1
                            int dx = (i & 1) ? -1 : 1;     // 1, -1,  1, -1,  1, -1,  1, -1,  1, -1,  1, -1,  1, -1,  1, -1
                            int dy = (i & 2) ? -1 : 1;     // 1,  1, -1, -1,  1,  1, -1, -1,  1,  1, -1, -1,  1,  1, -1, -1
                            if (swaps) {
                                px = cx + (y - adj) * dx;
                                py = cy + x * dy;
                            } else {
                                px = cx + x * dx;
                                py = cy + (y - adj) * dy;
                            }
                            uint32_t pixCol = getPixelColor(px, py);
                            setPixelColor(px, py, adj ? color_blend(pixCol, col, fade) : color_blend(col, pixCol, fade));
                    }
                    x++;
                }
            } else {
                // Bresenham’s Algorithm
                int d = 3 - (2*radius);
                int y = radius, x = 0;
                while (y >= x) {
                    for (int i = 0; i < 4; i++) {
                        int dx = (i & 1) ? -x : x;
                        int dy = (i & 2) ? -y : y;
                        setPixelColor(cx + dx, cy + dy, col);
                        setPixelColor(cx + dy, cy + dx, col);
                    }
                    x++;
                    if (d > 0) {
                        y--;
                        d += 4 * (x - y) + 10;
                    } else {
                        d += 4 * x + 6;
                    }
                }
            }
        }

        // by stepko, taken from https://editor.soulmatelights.com/gallery/573-blobs
        void fillCircle(uint16_t cx, uint16_t cy, uint8_t radius, uint32_t col, bool soft = false) {
            static_assert(dimensionality != EffectDimensionality::d1, "This function is for 2D effects only.");

            if (radius == 0)
                return; // not active

            const int vW = Segment::getEffectWidth<dimensionality>();   // segment width in logical pixels (can be 0 if segment is inactive)
            const int vH = Segment::getEffectHeight<dimensionality>();  // segment height in logical pixels (is always >= 1)

            // draw soft bounding circle
            if (soft)
                drawCircle(cx, cy, radius, col, soft);
            // fill it
            for (int y = -radius; y <= radius; y++) {
                for (int x = -radius; x <= radius; x++) {
                    if (x * x + y * y <= radius * radius &&
                        int(cx)+x >= 0 && int(cy)+y >= 0 &&
                        int(cx)+x < vW && int(cy)+y < vH)
                    setPixelColor(cx + x, cy + y, col);
                }
            }
        }

        // inline void drawCharacter(unsigned char chr, int16_t x, int16_t y, uint8_t w, uint8_t h, CRGB c) { drawCharacter(chr, x, y, w, h, RGBW32(c.r,c.g,c.b,0)); } // automatic inline
        //     void drawCharacter(unsigned char chr, int16_t x, int16_t y, uint8_t w, uint8_t h, uint32_t color, uint32_t col2 = 0, int8_t rotate = 0, bool usePalGrad = false);
        // inline void drawCharacter(unsigned char chr, int16_t x, int16_t y, uint8_t w, uint8_t h, CRGB c, CRGB c2, int8_t rotate = 0, bool usePalGrad = false) { drawCharacter(chr, x, y, w, h, RGBW32(c.r,c.g,c.b,0), RGBW32(c2.r,c2.g,c2.b,0), rotate, usePalGrad); } // automatic inline
        // draws a raster font character on canvas
        // only supports: 4x6=24, 5x8=40, 5x12=60, 6x8=48 and 7x9=63 fonts ATM
        void drawCharacter(unsigned char chr, int16_t x, int16_t y, uint8_t w, uint8_t h, uint32_t color, uint32_t col2, int8_t rotate = 0, bool usePalGrad = false) {
            if (chr < 32 || chr > 126)
                return; // only ASCII 32-126 supported
            chr -= 32; // align with font table entries
            const int font = w*h;

            CRGB col = CRGB(color);
            CRGBPalette16 grad = CRGBPalette16(col, col2 ? CRGB(col2) : col);
            if (usePalGrad)
                grad = SEGPALETTE; // selected palette as gradient

            const int width = Segment::getEffectWidth<dimensionality>();
            const int height = Segment::getEffectHeight<dimensionality>();

            //if (w<5 || w>6 || h!=8) return;
            for (int i = 0; i<h; i++) { // character height
                uint8_t bits = 0;
                switch (font) {
                    case 24: bits = pgm_read_byte_near(&console_font_4x6[(chr * h) + i]); break;  // 5x8 font
                    case 40: bits = pgm_read_byte_near(&console_font_5x8[(chr * h) + i]); break;  // 5x8 font
                    case 48: bits = pgm_read_byte_near(&console_font_6x8[(chr * h) + i]); break;  // 6x8 font
                    case 63: bits = pgm_read_byte_near(&console_font_7x9[(chr * h) + i]); break;  // 7x9 font
                    case 60: bits = pgm_read_byte_near(&console_font_5x12[(chr * h) + i]); break; // 5x12 font
                    default: return;
                }
                CRGBW c = ColorFromPalette(grad, (i+1)*255/h, 255u, LINEARBLEND_NOWRAP);
                for (int j = 0; j<w; j++) { // character width
                    int x0, y0;
                    switch (rotate) {
                        case -1: x0 = x + (h-1) - i; y0 = y + (w-1) - j; break; // -90 deg
                        case -2:
                        case  2: x0 = x + j;         y0 = y + (h-1) - i; break; // 180 deg
                        case  1: x0 = x + i;         y0 = y + j;         break; // +90 deg
                        default: x0 = x + (w-1) - j; y0 = y + i;         break; // no rotation
                    }
                    if (x0 < 0 || x0 >= width || y0 < 0 || y0 >= height)
                        continue; // drawing off-screen
                    if (((bits>>(j+(8-w))) & 0x01)) { // bit set
                        setPixelColor(x0, y0, c.color32);
                    }
                }
            }
        }

        void wuPixel(const EffectCoordinate& coordinate, uint32_t x, uint32_t y, CRGB c) {      //awesome wu_pixel procedure by reddit u/sutaburosu
            constexpr auto WU_WEIGHT = [](unsigned a, unsigned b) -> uint8_t { return ((uint8_t) (((a)*(b)+(a)+(b))>>8)); };
            // extract the fractional parts and derive their inverses
            unsigned xx = x & 0xff, yy = y & 0xff, ix = 255 - xx, iy = 255 - yy;
            // calculate the intensities for each affected pixel
            uint8_t wu[4] = {WU_WEIGHT(ix, iy), WU_WEIGHT(xx, iy),
                             WU_WEIGHT(ix, yy), WU_WEIGHT(xx, yy)};
            // multiply the intensities by the colour, and saturating-add them to the pixels
            for (int i = 0; i < 4; i++) {
                unsigned wu_x = (x >> 8) + (i & 1);        // precalculate x
                unsigned wu_y = (y >> 8) + ((i >> 1) & 1); // precalculate y
                if ((wu_x >= coordinate.width) || (wu_y >= coordinate.height)) {
                    continue;
                }
                CRGB led = getPixelColor(wu_x, wu_y);
                CRGB oldLed = led;
                led.r = qadd8(led.r, c.r * wu[i] >> 8);
                led.g = qadd8(led.g, c.g * wu[i] >> 8);
                led.b = qadd8(led.b, c.b * wu[i] >> 8);
                if (led != oldLed) setPixelColor(wu_x, wu_y, RGBW32(led.r, led.g, led.b, 0)); // don't repaint if same color
            }
        }
    };

private:
    using Self = BufferedEffect;
    using Base = BufferedEffectBase;

public:
    static constexpr const EffectDimensionality dimensionality = _dimensionality;
    static_assert(_dimensionality != EffectDimensionality::d0, "0D effect buffers don't make sense... or do they? In any case they are not currently implemented.");

protected:
    explicit BufferedEffect(const EffectInformation& ei, bool initBufferWithCurrentState)
        : Base{ei}
    {
        if (!initBufferWithCurrentState) {
            return;
        }
        const unsigned width = Segment::getEffectWidth<dimensionality>();
        const unsigned height = Segment::getEffectHeight<dimensionality>();
        const size_t length = width * height;

        if (!resizeVector(buffer.pixels, length)) {
            return;
        }

        for (unsigned y = 0u; y < height; ++y) {
            for (unsigned x = 0u; x < width; ++x) {
                buffer.pixels[convertToLinear(x, y)] = SEGMENT.getPixelColor(convertSegmentPixelIndex(x, y));
            }
        }
    }

public:
    bool nextFrameImpl(const EffectCoordinate& coordinate) {
        return resizeVector(buffer.pixels, coordinate.width * coordinate.height);
    }

    uint32_t getPixelColorImpl(const EffectCoordinate& coordinate, const LazyColor& currentColor) {
        return buffer.getPixelColorLinear(convertToLinear(coordinate.getXAbsolute(), coordinate.getYAbsolute()));
    }

private:
    static inline unsigned convertToLinear(unsigned x, unsigned y) {
        if constexpr (dimensionality == EffectDimensionality::d0) {
            return 0;
        } else if constexpr (dimensionality == EffectDimensionality::d1) {
            return x;
        } else {
            if constexpr (verifyXyBounds) {
                if (x >= Segment::getEffectWidth<dimensionality>()) [[unlikely]] {
                    Serial.printf("BufferedEffect::convertToLinear: x %u >= %u\n", x, Segment::getEffectWidth<dimensionality>());
                    std::terminate();
                }
                if (y >= Segment::getEffectHeight<dimensionality>()) [[unlikely]] {
                    Serial.printf("BufferedEffect::convertToLinear: y %u >= %u\n", y, Segment::getEffectHeight<dimensionality>());
                    std::terminate();
                }
            }
            return y * Segment::getEffectWidth<dimensionality>() + x;
        }
    }
    static inline unsigned convertSegmentPixelIndex(unsigned x, unsigned y) {
        if constexpr (dimensionality == EffectDimensionality::d2VStrips) {
            return ((x) | (int((y) + 1) << 16)); // original indexToVStrip
        } else {
            return convertToLinear(x, y);
        }
    }

protected:
    PixelBuffer buffer{};
};
