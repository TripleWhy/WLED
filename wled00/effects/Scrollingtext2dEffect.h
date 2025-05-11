#pragma once

#include "../FX.h"
#include "BufferedEffect.h"
#include "Effect.h"

////////////////////////////
//     2D Scrolling text  //
////////////////////////////
class Scrollingtext2dEffect : public BaseEffect<Scrollingtext2dEffect, BufferedEffect<EffectDimensionality::d2>> {
private:
    using Self = Scrollingtext2dEffect;
    using Base = BaseEffect<Self, BufferedEffect<EffectDimensionality::d2>>;

public:
    static constexpr const char metaData[] PROGMEM = "Scrolling Text@!,Y Offset,Trail,Font size,Rotate,Gradient,Overlay,Reverse;!,!,Gradient;!;2;ix=128,c1=0,rev=0,mi=0,rY=0,mY=0";
    static constexpr const uint8_t effectId = FX_MODE_2DSCROLLTEXT;

    explicit Scrollingtext2dEffect(const EffectInformation& ei) : Base{ei, false} {}

    bool nextFrameImpl(TransitionableParameters& parameters, const EffectCoordinate& coordinate) {
        if (!Base::nextFrameImpl(parameters, coordinate)) {
            return false;
        }

        const int cols = coordinate.width;
        const int rows = coordinate.height;

        unsigned letterWidth, rotLW;
        unsigned letterHeight, rotLH;
        switch (map(parameters.custom2, 0, 255, 1, 5)) {
            default:
            case 1: letterWidth = 4; letterHeight =  6; break;
            case 2: letterWidth = 5; letterHeight =  8; break;
            case 3: letterWidth = 6; letterHeight =  8; break;
            case 4: letterWidth = 7; letterHeight =  9; break;
            case 5: letterWidth = 5; letterHeight = 12; break;
        }
        // letters are rotated
        if (((parameters.custom3+1)>>3) % 2) {
            rotLH = letterWidth;
            rotLW = letterHeight;
        } else {
            rotLW = letterWidth;
            rotLH = letterHeight;
        }

        char text[WLED_MAX_SEGNAME_LEN+1] = {'\0'};
        if (SEGMENT.name) for (size_t i=0,j=0; i<strlen(SEGMENT.name); i++) if (SEGMENT.name[i]>31 && SEGMENT.name[i]<128) text[j++] = SEGMENT.name[i];
        const bool zero = strchr(text, '0') != nullptr;

        char sec[5];
        int  AmPmHour = hour(localTime);
        bool isitAM = true;
        if (useAMPM) {
            if (AmPmHour > 11) { AmPmHour -= 12; isitAM = false; }
            if (AmPmHour == 0) { AmPmHour  = 12; }
            sprintf_P(sec, PSTR(" %2s"), (isitAM ? "AM" : "PM"));
        } else {
            sprintf_P(sec, PSTR(":%02d"), second(localTime));
        }

        if (!strlen(text)) { // fallback if empty segment name: display date and time
            sprintf_P(text, PSTR("%s %d, %d %d:%02d%s"), monthShortStr(month(localTime)), day(localTime), year(localTime), AmPmHour, minute(localTime), sec);
        } else {
            if (text[0] == '#') for (auto &c : text) c = std::toupper(c);
            if      (!strncmp_P(text,PSTR("#DATE"),5)) sprintf_P(text, zero?PSTR("%02d.%02d.%04d"):PSTR("%d.%d.%d"),   day(localTime),   month(localTime),  year(localTime));
            else if (!strncmp_P(text,PSTR("#DDMM"),5)) sprintf_P(text, zero?PSTR("%02d.%02d")     :PSTR("%d.%d"),      day(localTime),   month(localTime));
            else if (!strncmp_P(text,PSTR("#MMDD"),5)) sprintf_P(text, zero?PSTR("%02d/%02d")     :PSTR("%d/%d"),      month(localTime), day(localTime));
            else if (!strncmp_P(text,PSTR("#TIME"),5)) sprintf_P(text, zero?PSTR("%02d:%02d%s")   :PSTR("%2d:%02d%s"), AmPmHour,         minute(localTime), sec);
            else if (!strncmp_P(text,PSTR("#HHMM"),5)) sprintf_P(text, zero?PSTR("%02d:%02d")     :PSTR("%d:%02d"),    AmPmHour,         minute(localTime));
            else if (!strncmp_P(text,PSTR("#HH"),3))   sprintf  (text, zero?    ("%02d")          :    ("%d"),         AmPmHour);
            else if (!strncmp_P(text,PSTR("#MM"),3))   sprintf  (text, zero?    ("%02d")          :    ("%d"),         minute(localTime));
            else if (!strncmp_P(text,PSTR("#SS"),3))   sprintf  (text,          ("%02d")                     ,         second(localTime));
            else if (!strncmp_P(text,PSTR("#DD"),3))   sprintf  (text, zero?    ("%02d")          :    ("%d"),         day(localTime));
            else if (!strncmp_P(text,PSTR("#DAY"),4))  sprintf  (text,          ("%s")                       ,         dayShortStr(day(localTime)));
            else if (!strncmp_P(text,PSTR("#DDDD"),5)) sprintf  (text,          ("%s")                       ,         dayStr(day(localTime)));
            else if (!strncmp_P(text,PSTR("#MO"),3))   sprintf  (text, zero?    ("%02d")          :    ("%d"),         month(localTime));
            else if (!strncmp_P(text,PSTR("#MON"),4))  sprintf  (text,          ("%s")                       ,         monthShortStr(month(localTime)));
            else if (!strncmp_P(text,PSTR("#MMMM"),5)) sprintf  (text,          ("%s")                       ,         monthStr(month(localTime)));
            else if (!strncmp_P(text,PSTR("#YY"),3))   sprintf  (text,          ("%02d")                     ,         year(localTime)%100);
            else if (!strncmp_P(text,PSTR("#YYYY"),5)) sprintf_P(text, zero?PSTR("%04d")          :    ("%d"),         year(localTime));
        }

        const int  numberOfLetters = strlen(text);
        int width = (numberOfLetters * rotLW);
        int yoffset = map(parameters.intensity, 0, 255, -rows/2, rows/2) + (rows-rotLH)/2;
        if (width <= cols) {
            // scroll vertically (e.g. ^^ Way out ^^) if it fits
            int speed = map(parameters.speed, 0, 255, 5000, 1000);
            int frac = strip.now % speed + 1;
            if (parameters.intensity == 255) {
                yoffset = (2 * frac * rows)/speed - rows;
            } else if (parameters.intensity == 0) {
                yoffset = rows - (2 * frac * rows)/speed;
            }
        }

        if (step < strip.now) {
            // calculate start offset
            if (width > cols) {
                if (parameters.check3) {
                    if (aux0 == 0) aux0  = width + cols - 1;
                    else                --aux0;
                } else                ++aux0 %= width + cols;
            } else                    aux0  = (cols + width)/2;
            ++aux1 &= 0xFF; // color shift
            step = strip.now + map(parameters.speed, 0, 255, 250, 50); // shift letters every ~250ms to ~50ms
        }

        if (!parameters.check2) buffer.fadeOut(255 - (parameters.custom1>>4));  // trail
        bool usePaletteGradient = false;
        uint32_t col1 = SEGMENT.color_from_palette(aux1, false, PALETTE_SOLID_WRAP, 0);
        uint32_t col2 = BLACK;
        if (parameters.check1) { // use gradient
            if(SEGMENT.palette == 0) { // use colors for gradient
            col1 = SEGCOLOR(0);
            col2 = SEGCOLOR(2);
            }
            else usePaletteGradient = true;
        }

        for (int i = 0; i < numberOfLetters; i++) {
            int xoffset = int(cols) - int(aux0) + rotLW*i;
            if (xoffset + rotLW < 0) continue; // don't draw characters off-screen
            buffer.drawCharacter(text[i], xoffset, yoffset, letterWidth, letterHeight, col1, col2, map(parameters.custom3, 0, 31, -2, 2), usePaletteGradient);
        }
        return true;
    }

private:
    uint32_t step{};
    uint16_t aux0{};
    uint16_t aux1{};
};


