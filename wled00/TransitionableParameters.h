#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

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

    bool on{};
    uint32_t call{};  // call counter
    CRGBPalette16 palette{};

private:
    std::array<uint32_t, NUM_COLORS> colors{};
    std::array<uint32_t, NUM_COLORS> gammaCorrectedColors{};
    // This forces constructor and copy functions to not be constexpr. Should CRGBPalette16 be adjusted?

public:
    // This isn't defaulted because bitfield members can't be defaulted in their definition until c++20.
    TransitionableParameters()
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
    );

    TransitionableParameters(const TransitionableParameters& other) = default;
    TransitionableParameters(TransitionableParameters&& other) = default;
    TransitionableParameters& operator=(const TransitionableParameters& other) = default;
    TransitionableParameters& operator=(TransitionableParameters&& other) = default;
    ~TransitionableParameters() = default;

    void setRawColor(uint_fast8_t index, uint32_t color);
    constexpr uint32_t getRawColor(uint_fast8_t index) const {
        return colors[index];
    }
    constexpr uint32_t getGammaCorrectedColor(uint_fast8_t index) const {
        return gammaCorrectedColors[index];
    }

    /*
     * Gets a single color from the currently selected palette.
     * @param i Palette Index (if mapping is true, the full palette will be _virtualSegmentLength long, if false, 255). Will wrap around automatically.
     * @param mapping if true, LED position in segment is considered for color
     * @param moving FastLED palettes will usually wrap back to the start smoothly. Set to true if effect has moving palette and you want wrap.
     * @param mcol If the default palette 0 is selected, return the standard color 0, 1 or 2 instead. If >2, Party palette is used instead
     * @param pbri Value to scale the brightness of the returned color by. Default is 255. (no scaling)
     * @returns Single color from palette
     */
    [[gnu::hot]] uint32_t color_from_palette(uint16_t i, bool mapping, bool moving, uint8_t mcol, uint8_t pbri = 255) const;
    [[gnu::hot]] uint32_t color_wheel(uint8_t pos) const;

};
