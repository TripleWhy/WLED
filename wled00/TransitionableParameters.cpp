#include "colors.h"
#include "effects/effectUtils.h"
#include "TransitionableParameters.h"
#include "wled.h"

TransitionableParameters::TransitionableParameters(
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
    on{on},
    call{call},
    colors{colors},
    gammaCorrectedColors{gamma32(colors[0]), gamma32(colors[1]), gamma32(colors[2])}
{
}

void TransitionableParameters::setRawColor(uint_fast8_t index, uint32_t color) {
    colors[index] = color;
    gammaCorrectedColors[index] = gamma32(color);
}

uint32_t TransitionableParameters::color_from_palette(uint16_t i, bool mapping, bool moving, uint8_t mcol, uint8_t pbri) const {
    uint32_t color = getGammaCorrectedColor(mcol < NUM_COLORS ? mcol : 0);
    // default palette or no RGB support on segment
    if ((SEGMENT.palette == 0 && mcol < NUM_COLORS) || !SEGMENT.hasRGB()) {
        return color_fade(color, pbri, true);
    }

    const int vL = Segment::vLength();
    unsigned paletteIndex = i;
    if (mapping && vL > 1) paletteIndex = (i*255)/(vL -1);
    // paletteBlend: 0 - wrap when moving, 1 - always wrap, 2 - never wrap, 3 - none (undefined/no interpolation of palette entries)
    // ColorFromPalette interpolations are: NOBLEND, LINEARBLEND, LINEARBLEND_NOWRAP
    TBlendType blend = NOBLEND;
    switch (strip.paletteBlend) { // NOTE: paletteBlend should be global
        case 0: blend = moving ? LINEARBLEND : LINEARBLEND_NOWRAP; break;
        case 1: blend = LINEARBLEND; break;
        case 2: blend = LINEARBLEND_NOWRAP; break;
    }
    CRGBW palcol = palette.ColorFromPalette(paletteIndex, pbri, blend);
    palcol.w = W(color);

    return palcol.color32;
}

/*
 * Put a value 0 to 255 in to get a color value.
 * The colours are a transition r -> g -> b -> back to r
 * Inspired by the Adafruit examples.
 */
uint32_t TransitionableParameters::color_wheel(uint8_t pos) const {
    if (SEGMENT.palette) // perhaps "strip.paletteBlend < 2" should be better instead of "true"
        return color_from_palette(pos, false, true, 0);
    uint8_t w = W(getGammaCorrectedColor(0));
    pos = 255 - pos;
    if (pos < 85) {
        return RGBW32((255 - pos * 3), 0, (pos * 3), w);
    } else if (pos < 170) {
        pos -= 85;
        return RGBW32(0, (pos * 3), (255 - pos * 3), w);
    } else {
        pos -= 170;
        return RGBW32((pos * 3), (255 - pos * 3), 0, w);
    }
}