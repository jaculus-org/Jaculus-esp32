#pragma once

#include <jac/machine/class.h>
#include <jac/machine/functionFactory.h>
#include <jac/machine/machine.h>

#include "../platform/espWifi.h"

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
    }
};
