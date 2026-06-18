#pragma once

#include <jac/machine/class.h>
#include <jac/machine/functionFactory.h>
#include <jac/machine/machine.h>

#include <memory>
#include <noal_func.h>
#include <optional>
#include <array>
#include <span>
#include <vector>

#include "onewire_bus.h"
#include "onewire_device.h"
#include "onewire_crc.h"
#include "freertos/FreeRTOS.h"


template<class PlatformInfo>
class OneWire {
    onewire_bus_handle_t bus = nullptr;
    int pin;

    void checkOpen() const {
        if (bus == nullptr) {
            throw std::runtime_error("OneWire instance is closed");
        }
    }

public:
    OneWire(int pin) : pin(pin) {
        onewire_bus_config_t bus_config = {
            .bus_gpio_num = pin,
            .flags = {
                .en_pull_up = false
            }
        };

        onewire_bus_rmt_config_t rmt_config = {
            .max_rx_bytes = 10
        };

        esp_err_t err = onewire_new_bus_rmt(&bus_config, &rmt_config, &bus);
        if (err != ESP_OK) {
            throw std::runtime_error("Failed to create OneWire bus");
        }
    }

    static std::optional<OneWire> find(int pin) {
        return std::nullopt;
    }

    bool reset() {
        checkOpen();

        esp_err_t err = onewire_bus_reset(bus);
        if (err == ESP_OK) {
            return true;
        }
        else if (err == ESP_ERR_NOT_FOUND) {
            return false;
        }
        else {
            throw std::runtime_error("OneWire bus reset failed");
        }
    }

    std::vector<uint8_t> read(int count) {
        checkOpen();

        if (count <= 0) {
            throw std::runtime_error("Count must be greater than 0");
        }

        std::vector<uint8_t> data(count);
        esp_err_t err = onewire_bus_read_bytes(bus, data.data(), count);

        if (err != ESP_OK) {
            throw std::runtime_error("Failed to read from OneWire bus");
        }

        return data;
    }

    void write(std::span<const uint8_t> data) {
        checkOpen();

        esp_err_t err = onewire_bus_write_bytes(bus, data.data(), data.size());

        if (err != ESP_OK) {
            throw std::runtime_error("Failed to write to OneWire bus");
        }
    }

    void select(std::span<const uint8_t> rom) {
        checkOpen();

        if (rom.size() != 8) {
            throw std::runtime_error("ROM address must be 8 bytes");
        }

        reset();

        uint8_t cmd = 0x55;
        std::vector<uint8_t> romCmd = {cmd};
        romCmd.insert(romCmd.end(), rom.begin(), rom.end());
        write(std::span<const uint8_t>(romCmd));
    }

    void skip() {
        checkOpen();

        reset();

        const std::array<uint8_t, 1> skipCommand = {0xCC};
        write(std::span<const uint8_t>(skipCommand));
    }

    std::vector<std::vector<uint8_t>> search() {
        checkOpen();

        std::vector<std::vector<uint8_t>> devices;
        onewire_device_iter_handle_t iter = nullptr;

        esp_err_t err = onewire_new_device_iter(bus, &iter);
        if (err != ESP_OK) {
            throw std::runtime_error("Failed to create OneWire device iterator");
        }

        onewire_device_t dev;
        while (onewire_device_iter_get_next(iter, &dev) == ESP_OK) {
            std::vector<uint8_t> rom(8);
            uint64_t addr = dev.address;
            for (int i = 0; i < 8; i++) {
                rom[i] = (addr >> (i * 8)) & 0xFF;
            }
            devices.push_back(rom);
        }

        err = onewire_del_device_iter(iter);
        if (err != ESP_OK) {
            throw std::runtime_error("Failed to delete OneWire device iterator");
        }

        return devices;
    }

    uint8_t crc8(std::span<const uint8_t> data) {
        return onewire_crc8(0, const_cast<uint8_t*>(data.data()), data.size());
    }

    void close() {
        if (bus != nullptr) {
            onewire_bus_del(bus);
            bus = nullptr;
        }
    }

    ~OneWire() {
        close();
    }
};


template<class OneWireFeature>
struct OneWireProtoBuilder : public jac::ProtoBuilder::Opaque<OneWire<typename OneWireFeature::PlatformInfo>>, public jac::ProtoBuilder::Properties {
    using OneWire_ = OneWire<typename OneWireFeature::PlatformInfo>;

    static OneWire_* constructOpaque(jac::ContextRef /*ctx*/, std::vector<jac::ValueWeak> args) {
        if (args.size() != 1) {
            throw std::runtime_error("Expected one argument: pin");
        }

        return new OneWire_(args[0].to<int>());
    }

    static void addProperties(JSContext* ctx, jac::Object proto) {
        jac::FunctionFactory ff(ctx);

        OneWireProtoBuilder::template addMethodMember<decltype(&OneWire_::reset), &OneWire_::reset>(ctx, proto, "reset");
        OneWireProtoBuilder::template addMethodMember<decltype(&OneWire_::skip), &OneWire_::skip>(ctx, proto, "skip");
        OneWireProtoBuilder::template addMethodMember<decltype(&OneWire_::close), &OneWire_::close>(ctx, proto, "close");

        proto.defineProperty("read", ff.newFunctionThisVariadic([](jac::ContextRef ctx, jac::ValueWeak thisVal, std::vector<jac::ValueWeak> args) {
            auto& feature = *reinterpret_cast<OneWireFeature*>(JS_GetContextOpaque(ctx));  // NOLINT
            auto& ow = *OneWireProtoBuilder::getOpaque(ctx, thisVal);

            int count = 1;
            if (!args.empty()) {
                count = args[0].to<int>();
            }

            auto data = ow.read(count);
            return feature.toUint8Array(data);
        }));

        proto.defineProperty("write", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, jac::Value dataVal) {
            auto& feature = *reinterpret_cast<OneWireFeature*>(JS_GetContextOpaque(ctx));  // NOLINT
            auto& ow = *OneWireProtoBuilder::getOpaque(ctx, thisVal);

            auto dataVec = feature.toStdVector(dataVal);
            ow.write(std::span<const uint8_t>(dataVec));
        }));

        proto.defineProperty("select", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, jac::Value romVal) {
            auto& feature = *reinterpret_cast<OneWireFeature*>(JS_GetContextOpaque(ctx));  // NOLINT
            auto& ow = *OneWireProtoBuilder::getOpaque(ctx, thisVal);

            auto rom = feature.toStdVector(romVal);
            if (rom.size() != 8) {
                throw std::runtime_error("ROM address must be 8 bytes");
            }

            ow.select(std::span<const uint8_t>(rom));
        }));

        proto.defineProperty("search", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal) {
            auto& feature = *reinterpret_cast<OneWireFeature*>(JS_GetContextOpaque(ctx));  // NOLINT
            auto& ow = *OneWireProtoBuilder::getOpaque(ctx, thisVal);

            auto devices = ow.search();

            auto arr = jac::Array::create(ctx);
            for (size_t i = 0; i < devices.size(); i++) {
                jac::Value deviceVal = feature.toUint8Array(devices[i]);
                arr.set(i, deviceVal);
            }

            return arr;
        }));

        proto.defineProperty("crc8", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, jac::Value dataVal) {
            auto& feature = *reinterpret_cast<OneWireFeature*>(JS_GetContextOpaque(ctx));  // NOLINT
            auto& ow = *OneWireProtoBuilder::getOpaque(ctx, thisVal);

            auto dataVec = feature.toStdVector(dataVal);
            uint8_t crc = ow.crc8(std::span<const uint8_t>(dataVec));
            return jac::Value::from(ctx, static_cast<int>(crc));
        }));
    }
};


template<class Next>
class OneWireFeature : public Next {
public:
    using OneWireClass = jac::Class<OneWireProtoBuilder<OneWireFeature>>;

    OneWireFeature() {
        OneWireClass::init("OneWire");
    }

    void initialize() {
        Next::initialize();

        jac::Module& mod = this->newModule("onewire");
        mod.addExport("OneWire", OneWireClass::getConstructor(this->context()));
    }
};
