#pragma once

#include <jac/machine/functionFactory.h>
#include <jac/machine/machine.h>

#include <stdexcept>
#include <unordered_map>

#include "driver/ledc.h"


template<class Next>
class LedcFeature : public Next {
    class Ledc {
        std::unordered_map<int, int> _usedTimers;
        std::unordered_map<int, int> _usedChannels;
    public:
        void configureTimer(int timerNum, int frequency, int resolution) {
            if (resolution < 1 || resolution >= LEDC_TIMER_BIT_MAX) {
                throw std::runtime_error("Resolution must be between 1 and " + std::to_string(LEDC_TIMER_BIT_MAX - 1));
            }
            if (frequency < 1) {
                throw std::runtime_error("Frequency must be greater than 0");
            }

            ledc_timer_config_t ledc_timer = {
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .duty_resolution = static_cast<ledc_timer_bit_t>(resolution),
                .timer_num = static_cast<ledc_timer_t>(timerNum),
                .freq_hz = static_cast<uint32_t>(frequency),
                .clk_cfg = LEDC_AUTO_CLK,
                .deconfigure = false
            };
            esp_err_t err = ledc_timer_config(&ledc_timer);
            if (err != ESP_OK) {
                throw std::runtime_error(esp_err_to_name(err));
            }

            _usedTimers.insert({ timerNum, resolution });
        }

        void configureChannel(int channelNum, int gpioNum, int timerNum, int duty) {
            auto timer = _usedTimers.find(timerNum);
            if (timer == _usedTimers.end()) {
                throw std::runtime_error("Timer not configured");
            }
            if (duty < 0 || duty > 1023) {
                throw std::runtime_error("Duty must be between 0 and 1023");
            }

            duty = (1 << timer->second) * duty / 1023;

            ledc_channel_config_t ledc_channel = {
                .gpio_num = Next::getDigitalPin(gpioNum),
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .channel = static_cast<ledc_channel_t>(channelNum),
                .intr_type = LEDC_INTR_DISABLE,
                .timer_sel = static_cast<ledc_timer_t>(timerNum),
                .duty = static_cast<uint32_t>(duty),
                .hpoint = 0,
                .flags = { 0 }
            };
            esp_err_t err = ledc_channel_config(&ledc_channel);
            if (err != ESP_OK) {
                throw std::runtime_error(esp_err_to_name(err));
            }

            _usedChannels.insert({ channelNum, timerNum });
        }

        void setFrequency(int timerNum, int frequency) {
            if (_usedTimers.find(timerNum) == _usedTimers.end()) {
                throw std::runtime_error("Timer not in use");
            }
            if (frequency < 1) {
                throw std::runtime_error("Frequency must be greater than 0");
            }

            esp_err_t err = ledc_set_freq(LEDC_LOW_SPEED_MODE, static_cast<ledc_timer_t>(timerNum), frequency);
            if (err != ESP_OK) {
                throw std::runtime_error(esp_err_to_name(err));
            }
        }

        void setDuty(int channelNum, int duty) {
            auto channel = _usedChannels.find(channelNum);
            if (channel == _usedChannels.end()) {
                throw std::runtime_error("Channel not in use");
            }
            auto timer = _usedTimers.find(channel->second);
            if (duty < 0 || duty > 1023) {
                throw std::runtime_error("Duty must be between 0 and 1023");
            }

            duty = (1 << timer->second) * duty / 1023;

            esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(channelNum), duty);
            if (err != ESP_OK) {
                throw std::runtime_error(esp_err_to_name(err));
            }

            err = ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(channelNum));
            if (err != ESP_OK) {
                throw std::runtime_error(esp_err_to_name(err));
            }
        }

        void stopTimer(int timerNum) {
            bool channelInUse = false;
            for (auto channel : _usedChannels) {
                if (channel.second == timerNum) {
                    channelInUse = true;
                    break;
                }
            }
            if (channelInUse) {
                throw std::runtime_error("Timer still in use by channel");
            }

            esp_err_t err = ledc_timer_rst(LEDC_LOW_SPEED_MODE, static_cast<ledc_timer_t>(timerNum));
            if (err != ESP_OK) {
                throw std::runtime_error(esp_err_to_name(err));
            }

            _usedTimers.erase(timerNum);
        }

        void stopChannel(int channelNum) {
            esp_err_t err = ledc_stop(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(channelNum), 0);
            if (err != ESP_OK) {
                throw std::runtime_error(esp_err_to_name(err));
            }

            _usedChannels.erase(channelNum);
        }

        ~Ledc() {
            for (auto channel : _usedChannels) {
                ledc_stop(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(channel.first), 0);
            }
            for (auto timer : _usedTimers) {
                ledc_timer_rst(LEDC_LOW_SPEED_MODE, static_cast<ledc_timer_t>(timer.first));
            }
        }
    };
public:
    Ledc ledc;

    void initialize() {
        Next::initialize();

        jac::FunctionFactory ff(this->context());

        jac::Module& ledcModule = this->newModule("ledc");
        ledcModule.addExport("configureTimer", ff.newFunctionVariadic([this](std::vector<jac::ValueWeak> args) {
            if (args.size() < 2) {
                throw std::runtime_error("Expected at least 2 arguments");
            }
            int timerNum = args[0].to<int>();
            int frequency = args[1].to<int>();
            int resolution = 10;
            if (args.size() == 3) {
                resolution = args[2].to<int>();
            }

            this->ledc.configureTimer(timerNum, frequency, resolution);
        }));
        ledcModule.addExport("configureChannel", ff.newFunction(noal::function(&Ledc::configureChannel, &ledc)));
        ledcModule.addExport("setFrequency", ff.newFunction(noal::function(&Ledc::setFrequency, &ledc)));
        ledcModule.addExport("setDuty", ff.newFunction(noal::function(&Ledc::setDuty, &ledc)));
        ledcModule.addExport("stopTimer", ff.newFunction(noal::function(&Ledc::stopTimer, &ledc)));
        ledcModule.addExport("stopChannel", ff.newFunction(noal::function(&Ledc::stopChannel, &ledc)));
    }
};
