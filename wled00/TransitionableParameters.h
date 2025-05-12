#pragma once
#include <cstdint>
#include <array>

constexpr size_t NUM_COLORS = 3u; /* number of colors per segment */

class TransitionableParameters {
public:
    uint8_t  speed{};
    uint8_t  intensity{};

    // custom FX parameters/sliders
    uint8_t  custom1{};
    uint8_t  custom2{};
    struct {
        uint8_t custom3 : 5;        // reduced range slider (0-31)
        bool    check1  : 1;        // checkmark 1
        bool    check2  : 1;        // checkmark 2
        bool    check3  : 1;        // checkmark 3
    };

    uint32_t call{};  // call counter
    bool on{};

private:
    std::array<uint32_t, NUM_COLORS> colors{};
    std::array<uint32_t, NUM_COLORS> gammaCorrectedColors{};

public:
    constexpr TransitionableParameters()
        : custom3{0},
          check1{false},
          check2{false},
          check3{false}
    {
    }

    TransitionableParameters(
        std::array<uint32_t, NUM_COLORS> colors,
        uint8_t speed,
        uint8_t intensity,
        uint8_t custom1,
        uint8_t custom2,
        uint8_t custom3,
        bool check1,
        bool check2,
        bool check3,
        uint32_t call,
        bool on
    ) : speed{speed},
        intensity{intensity},
        custom1{custom1},
        custom2{custom2},
        custom3{custom3},
        check1{check1},
        check2{check2},
        check3{check3},
        call{call},
        on{on},
        colors{colors},
        gammaCorrectedColors{gamma32(colors[0]), gamma32(colors[1]), gamma32(colors[2])}
    {
    }

    constexpr TransitionableParameters(const TransitionableParameters& other) = default;
    constexpr TransitionableParameters(TransitionableParameters&& other) = default;
    constexpr TransitionableParameters& operator=(const TransitionableParameters& other) = default;
    constexpr TransitionableParameters& operator=(TransitionableParameters&& other) = default;
    ~TransitionableParameters() = default;

    void setRawColor(uint_fast8_t index, uint32_t color) {
        colors[index] = color;
        gammaCorrectedColors[index] = gamma32(color);
    }
    constexpr uint32_t getRawColor(uint_fast8_t index) const {
        return colors[index];
    }
    constexpr uint32_t getGammaCorrectedColor(uint_fast8_t index) const {
        return gammaCorrectedColors[index];
    }
};
