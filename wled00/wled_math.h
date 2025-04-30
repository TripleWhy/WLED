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
