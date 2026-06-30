#pragma once
#include "displayTypes.h"
#include <cstddef>
#include <cstdint>

class IDisplayHolder {
  public:
    virtual ~IDisplayHolder() = default;

    virtual void start() = 0;
    virtual void clear() = 0;
    virtual void setBrightness(uint8_t brightness) = 0;
    virtual bool isInitialized() const = 0;

    virtual void setBuffer(const uint8_t *rawData, size_t size, int format,
                           bool clearPrevious) = 0;

    virtual void setBufferFromRaw(const std::vector<DisplayPixel> &pixels,
                                  bool clearPrevious) = 0;
};
