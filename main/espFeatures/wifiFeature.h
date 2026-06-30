#pragma once

#include <jac/machine/class.h>
#include <jac/machine/functionFactory.h>
#include <jac/machine/machine.h>

#include "../platform/espWifi.h"

#include "esp_mac.h"
#include <array>
#include <iomanip>
#include <sstream>

template<class Next>
class WifiFeature : public Next {
public:
    void initialize() {
        Next::initialize();

        jac::FunctionFactory ff(this->context());
        auto& module = this->newModule("wifi");

        module.addExport("currentIp", ff.newFunction(noal::function([this]() {
            auto& wifi = EspWifiController::get();
            if(wifi.currentIp().addr == 0) {
                return jac::Value::null(this->context());
            }
            else {
                return jac::Value::from(this->context(), wifi.currentIpStr());
            }
        })));

        module.addExport("listNetworks", ff.newFunction(noal::function([this]() {
            auto nsHandle = EspNvsKeyValue::open("wifi_net");
            if(!nsHandle) {
                throw jac::Exception::create(jac::Exception::Type::InternalError, "failed to open namespace");
            }
            return nsHandle->keys();
        })));

        module.addExport("address", ff.newFunction(noal::function([]() {
            std::array<uint8_t, 6> mac{};
            if (esp_efuse_mac_get_default(mac.data()) != ESP_OK) {
                throw std::runtime_error("Failed to read device MAC address");
            }

            std::stringstream ss;
            ss << std::hex << std::setfill('0');
            for (size_t i = 0; i < mac.size(); ++i) {
                ss << std::setw(2) << static_cast<int>(mac[i]);
                if (i + 1 != mac.size()) {
                    ss << ":";
                }
            }
            return ss.str();
        })));
    }
};
