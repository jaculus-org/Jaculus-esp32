#pragma once

#include "../../util/displayTypes.h"
#include "../../util/iDisplayHolder.h"
#include "../displayBindings.h"
#include "ESP32-HUB75-MatrixPanel-I2S-DMA.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "jac/device/logger.h"
#include <cstdint>
#include <cstring>
#include <format>
#include <jac/machine/class.h>
#include <jac/machine/functionFactory.h>
#include <jac/machine/machine.h>
#include <jac/machine/values.h>
#include <vector>

static MatrixPanel_I2S_DMA *s_display = nullptr;

template <class Next> class Hub75Feature;

class Hub75Holder : public IDisplayHolder {
  public:
    inline static Hub75Holder *active_instance = nullptr;

  private:
    MatrixPanel_I2S_DMA *display;
    DisplayColor *m_previousBuffer;
    DisplayColor *m_current_frame_buffer;
    uint8_t *m_pixel_is_set;

    int m_width;
    int m_height;
    int m_chain_length;
    size_t m_buffer_count;
    bool m_initialized;
    static const char *TAG;

    HUB75_I2S_CFG::i2s_pins getDefaultPins() {
        return {.r1 = 4,
                .g1 = 5,
                .b1 = 6,
                .r2 = 7,
                .g2 = 15,
                .b2 = 16,
                .a = 18,
                .b = 8,
                .c = 10,
                .d = 42,
                .e = 17,
                .lat = 47,
                .oe = 2,
                .clk = 48};
    }

    bool isValidCoordinate(int x, int y) const {
        return x >= 0 && x < m_width && y >= 0 && y < m_height;
    }

  public:
    Hub75Holder(int panelWidth, int panelHeight, int chainLength)
        : display(nullptr), m_previousBuffer(nullptr),
          m_current_frame_buffer(nullptr), m_pixel_is_set(nullptr),
          m_initialized(false) {
        m_width = panelWidth * chainLength;
        m_height = panelHeight;
        m_chain_length = chainLength;
        m_buffer_count = m_width * m_height;
    }

    ~Hub75Holder() {
        if (m_current_frame_buffer)
            heap_caps_free(m_current_frame_buffer);
        if (m_previousBuffer)
            heap_caps_free(m_previousBuffer);
        if (m_pixel_is_set)
            heap_caps_free(m_pixel_is_set);

        if (active_instance == this) {
            active_instance = nullptr;
        }
    }

    void start() override {
        if (s_display != nullptr) {
            jac::Logger::debug("Hub75Holder: Soft-Reload detected. Hardware "
                               "requires clean state.");
            jac::Logger::debug("Hub75Holder: Rebooting device...");

            vTaskDelay(pdMS_TO_TICKS(100));
            esp_restart();

            while (true)
                vTaskDelay(100);
        }

        jac::Logger::debug("Hub75Holder: Starting Fresh Initialization...");

        if (m_previousBuffer) {
            heap_caps_free(m_previousBuffer);
            m_previousBuffer = nullptr;
        }
        if (m_current_frame_buffer) {
            heap_caps_free(m_current_frame_buffer);
            m_current_frame_buffer = nullptr;
        }
        if (m_pixel_is_set) {
            heap_caps_free(m_pixel_is_set);
            m_pixel_is_set = nullptr;
        }

        m_initialized = false;

        HUB75_I2S_CFG mxconfig(m_width / m_chain_length, m_height,
                               m_chain_length, getDefaultPins());
        mxconfig.double_buff = false;

        s_display = new MatrixPanel_I2S_DMA(mxconfig);

        if (!s_display || !s_display->begin()) {
            jac::Logger::error(
                "Hub75Holder: Hardware Init Failed! Rebooting...");
            vTaskDelay(pdMS_TO_TICKS(200));
            esp_restart();
            return;
        }

        this->display = s_display;

        jac::Logger::debug("Hub75Holder: Allocating PSRAM...");
        uint32_t caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
        m_previousBuffer = (DisplayColor *)heap_caps_malloc(
            m_buffer_count * sizeof(DisplayColor), caps);
        m_current_frame_buffer = (DisplayColor *)heap_caps_malloc(
            m_buffer_count * sizeof(DisplayColor), caps);
        m_pixel_is_set =
            (uint8_t *)heap_caps_calloc(m_buffer_count, sizeof(uint8_t), caps);

        if (!m_previousBuffer || !m_current_frame_buffer || !m_pixel_is_set) {
            jac::Logger::error("Hub75Holder: Out of Memory (PSRAM)");
            return;
        }

        for (size_t i = 0; i < m_buffer_count; i++) {
            m_previousBuffer[i] = DisplayColors::BLACK;
            m_current_frame_buffer[i] = DisplayColors::BLACK;
        }

        display->setBrightness8(90);
        display->clearScreen();

        m_initialized = true;
        jac::Logger::debug("Hub75Holder: Initialization COMPLETE.");
    }

    void setPixel(int x, int y, const DisplayColor &color) {
        if (!m_initialized || !isValidCoordinate(x, y))
            return;
        display->drawPixelRGB888(x, y, (uint8_t)((color.r * color.a) / 255),
                                 (uint8_t)((color.g * color.a) / 255),
                                 (uint8_t)((color.b * color.a) / 255));
    }

    void setBufferFromRaw(const std::vector<DisplayPixel> &pixels,
                          bool clearPrevious) override {
        if (!m_initialized || !m_current_frame_buffer)
            return;

        memset(m_pixel_is_set, 0, m_buffer_count * sizeof(uint8_t));

        for (const auto &pixel : pixels) {
            if (isValidCoordinate(pixel.x, pixel.y)) {
                int index = pixel.y * m_width + pixel.x;
                m_current_frame_buffer[index] = pixel.color;
                m_pixel_is_set[index] = 1;
            }
        }

        for (int i = 0; i < m_buffer_count; ++i) {
            DisplayColor new_color;
            if (m_pixel_is_set[i]) {
                new_color = m_current_frame_buffer[i];
            } else {
                new_color =
                    clearPrevious ? DisplayColors::BLACK : m_previousBuffer[i];
            }

            if (m_previousBuffer[i] != new_color) {
                int y = i / m_width;
                int x = i % m_width;

                setPixel(x, y, new_color);
                m_previousBuffer[i] = new_color;
            }
        }
    }

    void setBuffer(const uint8_t *rawData, size_t size, int format,
                   bool clearPrevious) override {
        if (!m_initialized || !m_current_frame_buffer)
            return;

        DisplayColor black{0, 0, 0, 0};
        size_t colorBytes = DisplayUtils::unpackColor(rawData, format, black);
        if (colorBytes == 0)
            return;

        size_t expectedSize = m_buffer_count * colorBytes;
        if (size < expectedSize)
            return;

        for (int i = 0; i < m_buffer_count; ++i) {
            DisplayColor new_color = clearPrevious ? black : m_previousBuffer[i];
            DisplayUtils::unpackColor(&rawData[i * colorBytes], format,
                                      new_color);

            if (m_previousBuffer[i] != new_color) {
                int y = i / m_width;
                int x = i % m_width;
                uint8_t r = (new_color.r * new_color.a) / 255;
                uint8_t g = (new_color.g * new_color.a) / 255;
                uint8_t b = (new_color.b * new_color.a) / 255;

                display->drawPixelRGB888(x, y, r, g, b);
                m_previousBuffer[i] = new_color;
            }
        }
    }

    void clear() override {
        if (m_initialized) {
            display->clearScreen();
            for (size_t i = 0; i < m_buffer_count; i++) {
                m_previousBuffer[i] = DisplayColors::BLACK;
            }
        }
    }

    void setBrightness(uint8_t brightness) override {
        if (m_initialized)
            display->setBrightness8(brightness);
    }

    bool isInitialized() const override { return m_initialized; }
};

template <class Next> class Hub75Feature : public Next {
  public:
    class Hub75ProtoBuilder : public jac::ProtoBuilder::Opaque<Hub75Holder>,
                              public jac::ProtoBuilder::Properties {
      public:
        static Hub75Holder *constructOpaque(jac::ContextRef ctx,
                                            std::vector<jac::ValueWeak> args) {
            int pW = args.size() > 0 ? args[0].to<int>() : 64;
            int pH = args.size() > 1 ? args[1].to<int>() : 32;
            int cL = args.size() > 2 ? args[2].to<int>() : 1;
            auto *instance = new Hub75Holder(pW, pH, cL);
            instance->start();
            return instance;
        }

        static void addProperties(jac::ContextRef ctx, jac::Object proto) {
            DisplayProtoBindings<Hub75Holder>::addCommonProperties(ctx, proto);
        }
    };

    using Hub75Class = jac::Class<Hub75ProtoBuilder>;
    Hub75Feature() { Hub75Class::init("Hub75"); }

    void initialize() {
        Next::initialize();
        auto &mod = this->newModule("hub75");
        mod.addExport("Hub75", Hub75Class::getConstructor(this->context()));
    }
};
