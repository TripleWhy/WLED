#pragma once

#include <cstdint>

uint8_t sin_gap(uint16_t in) {
  if (in & 0x100) return 0;
  return sin8_t(in + 192); // correct phase shift of sine so that it starts and stops at 0
}

uint16_t triwave16(uint16_t in) {
    if (in < 0x8000) return in *2;
    return 0xFFFF - (in - 0x8000)*2;
}
