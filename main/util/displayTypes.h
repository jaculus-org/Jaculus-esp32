#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

struct DisplayColor {
    uint8_t r, g, b, a;

    bool operator==(const DisplayColor &o) const {
        return r == o.r && g == o.g && b == o.b && a == o.a;
    }
    bool operator!=(const DisplayColor &o) const { return !(*this == o); }
};

namespace DisplayColors {
static const DisplayColor BLACK(0, 0, 0);
static const DisplayColor WHITE(255, 255, 255);
static const DisplayColor RED(255, 0, 0);
static const DisplayColor GREEN(0, 255, 0);
static const DisplayColor BLUE(0, 0, 255);
static const DisplayColor YELLOW(255, 255, 0);
static const DisplayColor MAGENTA(255, 0, 255);
static const DisplayColor CYAN(0, 255, 255);
} // namespace DisplayColors

struct DisplayPixel {
    int x;
    int y;
    DisplayColor color;

    DisplayPixel() : x(0), y(0), color() {}
    DisplayPixel(int x, int y, DisplayColor color) : x(x), y(y), color(color) {}
};

bool operator==(const DisplayPixel &lhs, const DisplayPixel &rhs);
bool operator!=(const DisplayPixel &lhs, const DisplayPixel &rhs);

using DisplayPixels = std::vector<DisplayPixel>;

namespace DisplayUtils {
inline size_t unpackColor(const uint8_t *src, int format, DisplayColor &out) {
    switch (format) {
    case 3: { // 1-bit monochrome
        uint8_t val = (src[0] & 0x01) ? 255 : 0;
        out.r = val;
        out.g = val;
        out.b = val;
        out.a = 255;
        return 1;
    }
    case 4: { // 4-bit grayscale
        uint8_t val = src[0] & 0x0F;
        val |= (val << 4); // Map 0-15 to 0-255 by replicating the nibble
        out.r = val;
        out.g = val;
        out.b = val;
        out.a = 255;
        return 1;
    }
    case 5: { // 8-bit grayscale
        out.r = src[0];
        out.g = src[0];
        out.b = src[0];
        out.a = 255;
        return 1;
    }
    case 6: { // 8-bit RGB 3:3:2
        out.r = src[0] & 0xE0;
        out.g = (src[0] << 3) & 0xE0;
        out.b = (src[0] << 6) & 0xC0;
        out.a = 255;
        return 1;
    }
    case 7: { // 16-bit RGB 5:6:5 Little-Endian
        uint16_t val = src[0] | (src[1] << 8);
        out.r = (val >> 8) & 0xF8;
        out.g = (val >> 3) & 0xFC;
        out.b = (val << 3) & 0xF8;
        out.a = 255;
        return 2;
    }
    case 8: { // 16-bit RGB 5:6:5 Big-Endian
        uint16_t val = (src[0] << 8) | src[1];
        out.r = (val >> 8) & 0xF8;
        out.g = (val >> 3) & 0xFC;
        out.b = (val << 3) & 0xF8;
        out.a = 255;
        return 2;
    }
    case 9: { // 24-bit RGB 8:8:8
        out.r = src[0];
        out.g = src[1];
        out.b = src[2];
        out.a = 255;
        return 3;
    }
    case 10: { // 32-bit RGBA 8:8:8:8
        out.r = src[0];
        out.g = src[1];
        out.b = src[2];
        out.a = src[3];
        return 4;
    }
    case 12: { // 12-bit xRGB 4:4:4:4
        out.g = (src[0] & 0xF0) | (src[0] >> 4);
        out.b = ((src[0] & 0x0F) << 4) | (src[0] & 0x0F);
        out.a = (src[1] & 0xF0) | (src[1] >> 4);
        out.r = ((src[1] & 0x0F) << 4) | (src[1] & 0x0F);
        return 2;
    }
    default:
        return 0;
    }
}
} // namespace DisplayUtils
