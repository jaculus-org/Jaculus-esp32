#pragma once

#include <set>
#include <string>
#include <unordered_map>

#include "espCommon.h"


struct PlatformInfo {
    static inline const std::string NAME = "ESP32";

    struct PinConfig {
        static inline const std::set<int> DIGITAL_PINS = {
            0, 2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22,
            23, 25, 26, 27, 32, 33, 34, 35, 36, 37, 38, 39
        };
        static inline const std::unordered_map<int, std::pair<int, int>> ANALOG_PINS = {
            { 32, { 1, ADC1_GPIO32_CHANNEL } },
            { 33, { 1, ADC1_GPIO33_CHANNEL } },
            { 34, { 1, ADC1_GPIO34_CHANNEL } },
            { 35, { 1, ADC1_GPIO35_CHANNEL } },
            { 36, { 1, ADC1_GPIO36_CHANNEL } },
            { 37, { 1, ADC1_GPIO37_CHANNEL } },
            { 38, { 1, ADC1_GPIO38_CHANNEL } },
            { 39, { 1, ADC1_GPIO39_CHANNEL } }
        };
        static inline const std::set<int> INTERRUPT_PINS = DIGITAL_PINS;
        static inline constexpr int DEFAULT_I2C_SDA_PIN = 21;
        static inline constexpr int DEFAULT_I2C_SCL_PIN = 22;
    };
};


template<class Next>
using PlatformInfoFeature = EspCommon<PlatformInfo, Next>;
