#include <cstdint>

//wled_math.cpp
//float cos_t(float phi); // use float math
//float sin_t(float phi);
//float tan_t(float x);
int16_t sin16_t(uint16_t theta);
int16_t cos16_t(uint16_t theta);
uint8_t sin8_t(uint8_t theta);
uint8_t cos8_t(uint8_t theta);
float sin_approx(float theta); // uses integer math (converted to float), accuracy +/-0.0015 (compared to sinf())
float cos_approx(float theta);
float tan_approx(float x);
float atan2_t(float y, float x);
float acos_t(float x);
float asin_t(float x);
template <typename T> T atan_t(T x);
float floor_t(float x);
float fmod_t(float num, float denom);
uint32_t sqrt32_bw(uint32_t x);
#define sin_t sin_approx
#define cos_t cos_approx
#define tan_t tan_approx

uint8_t ease8InOutCubic(uint8_t i);
uint16_t ease16InOutCubic(uint16_t i);
uint8_t ease8InOutQuad(uint8_t i);

// inline math functions
__attribute__ ((always_inline)) inline uint8_t  scale8(uint8_t i, uint8_t scale ) { return ((int)i * (1 + (int)scale)) >> 8; }
__attribute__ ((always_inline)) inline uint8_t  scale8_video(uint8_t i, uint8_t scale ) { return (((int)i * (int)scale) >> 8) + ((i&&scale)?1:0); }

__attribute__ ((always_inline)) inline uint16_t scale16(uint16_t i, uint16_t scale ) { return ((uint32_t)i * (1 + (uint32_t)scale)) >> 16; }
__attribute__ ((always_inline)) inline uint8_t  qadd8(uint8_t i, uint8_t j) { unsigned t = i + j; return t > 255 ? 255 : t; }
__attribute__ ((always_inline)) inline uint8_t  qsub8(uint8_t i, uint8_t j) { int t = i - j; return t < 0 ? 0 : t; }
__attribute__ ((always_inline)) inline uint8_t  qmul8(uint8_t i, uint8_t j) { unsigned p = (unsigned)i * (unsigned)j; return p > 255 ? 255 : p; }
__attribute__ ((always_inline)) inline int8_t   abs8(int8_t i) { return i < 0 ? -i : i; }
__attribute__ ((always_inline)) inline int8_t   lerp8by8(uint8_t a, uint8_t b, uint8_t frac) { return a + ((((int32_t)b - (int32_t)a) * ((int32_t)frac+1)) >> 8); }


/*
#include <math.h>  // standard math functions. use a lot of flash
#define sin_t sinf
#define cos_t cosf
#define tan_t tanf
#define asin_t asinf
#define acos_t acosf
#define atan_t atanf
#define fmod_t fmodf
#define floor_t floorf
*/

// PRNG for 16bit and 8bit random numbers used by some effects (fastled replacement)
class PRNG {
private:
  uint16_t seed;
public:
  PRNG(uint16_t initialSeed = 0x1234) : seed(initialSeed) {}
  void setSeed(uint16_t s) { seed = s; }
  uint16_t getSeed() const { return seed; }
  uint16_t random16() {
      uint32_t s = seed;
      s *= 0x9E37;
      s ^= s >> 11;
      seed = (s & 0xFFFF) ^ (s >> 16);
      return seed;
  }
  uint16_t random16(uint16_t lim) { return ((uint32_t)random16() * lim) >> 16; }
  uint16_t random16(uint16_t min, uint16_t lim) { uint16_t delta = lim - min; return random16(delta) + min; }
  uint8_t random8() { return random16() >> 8; }
  uint8_t random8(uint8_t lim) { return (uint8_t)(((uint16_t)random8() * lim) >> 8); }
  uint8_t random8(uint8_t min, uint8_t lim) { uint8_t delta = lim - min; return random8(delta) + min; }
};
extern PRNG prng;
