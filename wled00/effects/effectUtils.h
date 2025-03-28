#pragma once

#include <cstdint>

uint16_t triwave16(uint16_t in) {
    if (in < 0x8000) return in *2;
    return 0xFFFF - (in - 0x8000)*2;
}
