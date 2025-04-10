#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

/*
 * Tetris or Stacking (falling bricks) Effect
 * by Blaz Kristan (AKA blazoncek) (https://github.com/blazoncek, https://blaz.at/home)
 */
class TetrixEffect : public BaseEffect<TetrixEffect, BufferedEffect<EffectDimensionality::d2VStrips>> {
private:
    struct Tetris {
        float    pos{};
        float    speed{};
        uint8_t  col{};   // color index
        uint16_t brick{}; // brick size in pixels
        uint16_t stack{0u}; // stack size in pixels
        uint32_t step{}; // state
    };

    using Self = TetrixEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2VStrips>>;

public:
    static constexpr const char metaData[] PROGMEM = "Tetrix@!,Width,,,,One color;!,!;!;;sx=0,ix=0,pal=11,m12=1";
    static constexpr const uint8_t effectId = FX_MODE_TETRIX;

    explicit TetrixEffect(const EffectInformation& ei) : Base{ei, true} {}

    void nextFrameImpl(const EffectCoordinate& coordinate) {
      Base::nextFrameImpl(coordinate);

      drops.resize(coordinate.height);
      if (drops.size() != coordinate.height) {
        drops.clear();
        return;
      }
      drops.shrink_to_fit();

      for (unsigned y=0; y < coordinate.height; ++y)
        runStrip(y, coordinate.width, &drops[y]);
    }

private:
    // virtualStrip idea by @ewowi (Ewoud Wijma)
    // requires virtual strip # to be embedded into upper 16 bits of index in setPixelcolor()
    // the following functions will not work on virtual strips: fill(), fade_out(), fadeToBlack(), blur()
    void runStrip(unsigned y, unsigned width, Tetris *drop) {
      const bool oneColor = SEGMENT.check1;
      // initialize dropping on first call or segment full
      if (SEGENV.call == 0) {
        drop->stack = 0;                  // reset brick stack size
        drop->step = strip.now + 2000;    // start by fading out strip
        if (oneColor) drop->col = 0;      // use only one color from palette
      }

      if (drop->step == 0) {              // init brick
        // speed calculation: a single brick should reach bottom of strip in X seconds
        // if the speed is set to 1 this should take 5s and at 255 it should take 0.25s
        // as this is dependant on width it should be taken into account and the fact that effect runs every FRAMETIME s
        int speed = SEGMENT.speed ? SEGMENT.speed : hw_random8(1,255);
        speed = map(speed, 1, 255, 5000, 250); // time taken for full (width) drop
        drop->speed = float(width * FRAMETIME) / float(speed); // set speed
        drop->pos   = width;             // start at end of segment (no need to subtract 1)
        if (!oneColor) drop->col = hw_random8(0,15)<<4;   // limit color choices so there is enough HUE gap
        drop->step  = 1;                  // drop state (0 init, 1 forming, 2 falling)
        drop->brick = (SEGMENT.intensity ? (SEGMENT.intensity>>5)+1 : hw_random8(1,5)) * (1+(width>>6));  // size of brick
      }

      if (drop->step == 1) {              // forming
        if (hw_random8()>>6) {               // random drop
          drop->step = 2;                 // fall
        }
      }

      if (drop->step == 2) {              // falling
        if (drop->pos > drop->stack) {    // fall until top of stack
          drop->pos -= drop->speed;       // may add gravity as: speed += gravity
          if (int(drop->pos) < int(drop->stack)) drop->pos = drop->stack;
          for (unsigned i = unsigned(drop->pos); i < width; i++) {
            uint32_t col = i < unsigned(drop->pos)+drop->brick ? SEGMENT.color_from_palette(drop->col, false, false, 0) : SEGCOLOR(1);
            buffer.setPixelColor(i, y, col);
          }
        } else {                          // we hit bottom
          drop->step = 0;                 // proceed with next brick, go back to init
          drop->stack += drop->brick;     // increase the stack size
          if (drop->stack >= width) drop->step = strip.now + 2000; // fade out stack
        }
      }

      if (drop->step > 2) {               // fade strip
        drop->brick = 0;                  // reset brick size (no more growing)
        if (drop->step > strip.now) {
          // allow fading of virtual strip
          for (unsigned x = 0; x < width; ++x) buffer.blendPixelColor(x, y, SEGCOLOR(1), 25); // 10% blend
        } else {
          drop->stack = 0;                // reset brick stack size
          drop->step = 0;                 // proceed with next brick
          if (oneColor) drop->col += 8;   // gradually increase palette index
        }
      }
    }

private:
    SegmentAllocator<Tetris>::vector drops;
};
