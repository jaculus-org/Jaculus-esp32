#pragma once

#include <jac/machine/functionFactory.h>
#include <jac/machine/machine.h>

#include <set>
#include <stdexcept>

#include "driver/adc.h"


template<class Next>
class AdcFeature : public Next {
    using PinConfig = Next::PlatformInfo::PinConfig;

    static std::pair<int, int> getAnalogPin(int pin) {
        auto it = PinConfig::ANALOG_PINS.find(pin);
        if (it == PinConfig::ANALOG_PINS.end()) {
            throw std::runtime_error("Invalid analog pin");
        }

        return it->second;
    }

    class Adc {
    public:
        void configure(int pin, int atten = static_cast<int>(ADC_ATTEN_DB_12)) {
            int adcNum;
            int channel;
            std::tie(adcNum, channel) = getAnalogPin(pin);

            esp_err_t err = adc1_config_width(static_cast<adc_bits_width_t>(ADC_WIDTH_BIT_DEFAULT));
            if (err != ESP_OK) {
                throw std::runtime_error(esp_err_to_name(err));
            }

            err = adc1_config_channel_atten(static_cast<adc1_channel_t>(channel), static_cast<adc_atten_t>(atten));
            if (err != ESP_OK) {
                throw std::runtime_error(esp_err_to_name(err));
            }
        }

        int read(int pin) {
            int adcNum;
            int channel;
            std::tie(adcNum, channel) = getAnalogPin(pin);

            return adc1_get_raw(static_cast<adc1_channel_t>(channel)) >> (ADC_WIDTH_BIT_DEFAULT - 10);
        }
    };

public:
    Adc adc;

    void initialize() {
        Next::initialize();

        jac::FunctionFactory ff(this->context());
        jac::Module& adcModule = this->newModule("adc");

        jac::Object attten = jac::Object::create(this->context());
        attten.defineProperty("Db0", jac::Value::from(this->context(), static_cast<int>(ADC_ATTEN_DB_0)));
        attten.defineProperty("Db2_5", jac::Value::from(this->context(), static_cast<int>(ADC_ATTEN_DB_2_5)));
        attten.defineProperty("Db6", jac::Value::from(this->context(), static_cast<int>(ADC_ATTEN_DB_6)));
        attten.defineProperty("Db12", jac::Value::from(this->context(), static_cast<int>(ADC_ATTEN_DB_12)));

        adcModule.addExport("configure", ff.newFunctionVariadic([this](std::vector<jac::ValueWeak> args) {
            if (args.size() < 1 || args.size() > 2) {
                throw std::runtime_error("Invalid number of arguments");
            }

            int pin = args[0].to<int>();
            int atten = static_cast<int>(ADC_ATTEN_DB_12);
            if (args.size() == 2) {
                atten = args[1].to<int>();
            }

            adc.configure(pin, atten);
        }));
        adcModule.addExport("read", ff.newFunction(noal::function(&Adc::read, &adc)));
        adcModule.addExport("Attenuation", attten);
    }
};
