#pragma once
#include <cstdint>

#define NUM_COLORS       3 /* number of colors per segment */

struct TransitionableParameters {
    uint32_t colors[NUM_COLORS];
    uint8_t  speed;
    uint8_t  intensity;
    uint8_t  custom1, custom2;    // custom FX parameters/sliders
    struct {
        uint8_t custom3 : 5;        // reduced range slider (0-31)
        bool    check1  : 1;        // checkmark 1
        bool    check2  : 1;        // checkmark 2
        bool    check3  : 1;        // checkmark 3
    };
    uint32_t call;  // call counter

    bool on;
};
