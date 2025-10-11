#pragma once

#include <jac/machine/functionFactory.h>
#include <jac/machine/machine.h>

#include <simple_radio.h>
#include <sstream>


template<>
struct jac::ConvTraits<PacketDataType> {
    static Value to(ContextRef ctx, PacketDataType val) {
        switch (val) {
        case PacketDataType::Number:
            return Value::from(ctx, "number");
        case PacketDataType::String:
            return Value::from(ctx, "string");
        case PacketDataType::KeyValue:
            return Value::from(ctx, "keyvalue");
        }
    }

    static PacketDataType from(ContextRef ctx, ValueWeak val) {
        auto str = val.to<std::string>();
        if (str == "number") {
            return PacketDataType::Number;
        }
        else if (str == "string") {
            return PacketDataType::String;
        }
        else if (str == "keyvalue") {
            return PacketDataType::KeyValue;
        }
        else {
            throw std::runtime_error("Invalid PacketDataType");
        }
    }
};

using EspBdAddress = std::array<uint8_t, ESP_BD_ADDR_LEN>;

template<>
struct jac::ConvTraits<EspBdAddress> {
    static Value to(ContextRef ctx, EspBdAddress val) {
        std::stringstream ss;
        ss << std::hex;
        for (int i = 0; i < 6; i++) {
            ss << static_cast<int>(val[i]);
            if (i != 5) {
                ss << ":";
            }
        }
        return Value::from(ctx, ss.str());
    }

    static EspBdAddress from(ContextRef ctx, ValueWeak val) {
        auto str = val.to<std::string>();
        EspBdAddress addr;
        std::stringstream ss(str);
        ss >> std::hex;
        for (int i = 0; i < 6; i++) {
            int byte;
            ss >> byte;
            addr[i] = static_cast<uint8_t>(byte);
            if (i != 5) {
                char c;
                ss >> c;
                if (c != ':') {
                    throw std::runtime_error("Invalid address format");
                }
            }
        }
        return addr;
    }
};

template<>
struct jac::ConvTraits<PacketInfo> {
    static Value to(ContextRef ctx, PacketInfo val) {
        EspBdAddress addr;
        std::copy(val.addr, val.addr + ESP_BD_ADDR_LEN, addr.begin());
        auto obj = Object::create(ctx);
        obj.set("group", static_cast<int>(val.group));
        obj.set("address", addr);
        obj.set("rssi", static_cast<int>(val.rssi));
        return obj;
    }

    static PacketInfo from(ContextRef ctx, ValueWeak val) {
        auto obj = val.to<Object>();
        auto addr = obj.get<EspBdAddress>("address");

        PacketInfo info;
        info.group = obj.get<int>("group");
        std::copy(addr.begin(), addr.end(), info.addr);
        info.rssi = obj.get<int>("rssi");
        return info;
    }
};


template<class Next>
class SimpleRadioFeature : public Next {
public:
    void initialize() {
        Next::initialize();

        jac::FunctionFactory ff(this->context());

        jac::Module& simpleradioModule = this->newModule("simpleradio");

        simpleradioModule.addExport("begin", ff.newFunction(noal::function([](int group) {
            auto config = SimpleRadio.DEFAULT_CONFIG;
            config.init_nvs = false;  // Jaculus-Esp32 initializes it
            esp_err_t err = SimpleRadio.begin(group, config);
            if (err != ESP_OK) {
                throw std::runtime_error("Failed to initialize SimpleRadio: " + std::to_string(err));
            }
        })));
        simpleradioModule.addExport("setGroup", ff.newFunction(noal::function([](int group) { SimpleRadio.setGroup(group); })));
        simpleradioModule.addExport("group", ff.newFunction(noal::function([]() -> int { return SimpleRadio.group(); })));
        simpleradioModule.addExport("address", ff.newFunction(noal::function([]() -> EspBdAddress {
            EspBdAddress res;
            SimpleRadio.address(res.data());
            return res;
        })));
        simpleradioModule.addExport("sendString", ff.newFunction(noal::function([](std::string str) { SimpleRadio.sendString(str); })));
        simpleradioModule.addExport("sendNumber", ff.newFunction(noal::function([](int num) { SimpleRadio.sendNumber(num); })));
        simpleradioModule.addExport("sendKeyValue", ff.newFunction(noal::function([](std::string key, double value) {
            SimpleRadio.sendKeyValue(key, value);
        })));
        simpleradioModule.addExport("on", ff.newFunction(noal::function([this](PacketDataType type, jac::Function callback) {
            switch (type) {
            case PacketDataType::Number:
                SimpleRadio.setOnNumberCallback([this, callback](double num, PacketInfo info) mutable {
                    this->scheduleEvent([callback, num, info]() mutable {
                        callback.call<void>(num, info);
                    });
                });
                break;
            case PacketDataType::String:
                SimpleRadio.setOnStringCallback([this, callback](std::string str, PacketInfo info) mutable {
                    this->scheduleEvent([callback, str, info]() mutable {
                        callback.call<void>(str, info);
                    });
                });
                break;
            case PacketDataType::KeyValue:
                SimpleRadio.setOnKeyValueCallback([this, callback](std::string key, double value, PacketInfo info) mutable {
                    this->scheduleEvent([callback, key, value, info]() mutable {
                        callback.call<void>(key, value, info);
                    });
                });
                break;
            }
        })));
        simpleradioModule.addExport("off", ff.newFunction(noal::function([](PacketDataType type) {
            switch (type) {
            case PacketDataType::Number:
                SimpleRadio.setOnNumberCallback(nullptr);
                break;
            case PacketDataType::String:
                SimpleRadio.setOnStringCallback(nullptr);
                break;
            case PacketDataType::KeyValue:
                SimpleRadio.setOnKeyValueCallback(nullptr);
                break;
            }
        })));
        simpleradioModule.addExport("end", ff.newFunction(noal::function([]() { SimpleRadio.end(); })));
    }

    ~SimpleRadioFeature() {
        SimpleRadio.setOnNumberCallback(nullptr);
        SimpleRadio.setOnStringCallback(nullptr);
        SimpleRadio.setOnKeyValueCallback(nullptr);
        SimpleRadio.end();
    }
};
