#pragma once

#include <jac/machine/class.h>
#include <jac/machine/functionFactory.h>
#include <jac/machine/machine.h>

#include <memory>
#include <noal_func.h>
#include <unordered_map>

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"


template<class PlatformInfo>
class I2C {
    i2c_port_t port;
    bool open = false;
public:
    I2C(int port) : port(static_cast<i2c_port_t>(port)) {}

    static std::optional<I2C> find(int pin) {
        return std::nullopt;
    }

    std::vector<uint8_t> readFrom(uint8_t address, size_t quantity) {
        std::vector<uint8_t> data(quantity);
        esp_err_t err = i2c_master_read_from_device(port, address, data.data(), quantity, 100 / portTICK_PERIOD_MS);
        if (err != ESP_OK) {
            throw std::runtime_error(esp_err_to_name(err));
        }

        return data;
    }

    // not compatible with Espruino - takes variadic data
    void writeTo(uint8_t address, std::vector<uint8_t> data) {
        esp_err_t err = i2c_master_write_to_device(port, address, data.data(), data.size(), 100 / portTICK_PERIOD_MS);

        if (err != ESP_OK) {
            throw std::runtime_error(esp_err_to_name(err));
        }
    }

    std::vector<uint8_t> writeRead(uint8_t address, std::vector<uint8_t> data, size_t quantity) {
        std::vector<uint8_t> res(quantity);
        esp_err_t err = i2c_master_write_read_device(port, address, data.data(), data.size(), res.data(), quantity, 100 / portTICK_PERIOD_MS);

        if (err != ESP_OK) {
            throw std::runtime_error(esp_err_to_name(err));
        }

        return res;
    }

    void setup(std::optional<int> scl, std::optional<int> sda, std::optional<int> bitrate) {
        if (open) {
            throw std::runtime_error("I2C already open");
        }

        int scl_ = scl.value_or(PlatformInfo::PinConfig::DEFAULT_I2C_SCL_PIN);
        int sda_ = sda.value_or(PlatformInfo::PinConfig::DEFAULT_I2C_SDA_PIN);
        int bitrate_ = bitrate.value_or(400000);

        i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = sda_,
            .scl_io_num = scl_,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master = {
                .clk_speed = static_cast<uint32_t>(bitrate_),
            },
            .clk_flags = 0,
        };

        i2c_param_config(port, &conf);
        i2c_driver_install(port, conf.mode, 0, 0, 0);
    }

    void close() {
        if (open) {
            i2c_driver_delete(port);
            open = false;
        }
    }

    ~I2C() {
        close();
    }
};

template<class I2CFeature>
struct I2CProtoBuilder : public jac::ProtoBuilder::Opaque<I2C<typename I2CFeature::PlatformInfo>>, public jac::ProtoBuilder::Properties {
    using I2C_ = I2C<typename I2CFeature::PlatformInfo>;

    static void addProperties(JSContext* ctx, jac::Object proto) {
        jac::FunctionFactory ff(ctx);

        // XXX: ugly hack
        static auto toUint8Array = [](jac::ContextRef ctx, std::vector<uint8_t> data) -> jac::Value {
            auto res = jac::ArrayBuffer::create(ctx, std::span(data.data(), data.size()));

            auto& machine = *static_cast<I2CFeature*>(JS_GetContextOpaque(ctx));
            jac::Value convertor = machine.eval("(buf) => new Uint8Array(buf)", "<I2CFeature::readFrom>");

            return convertor.to<jac::Function>().call<jac::Value>(res);
        };

        static auto toStdVector = [](jac::ContextRef ctx, jac::Value data) -> std::vector<uint8_t> {
            std::vector<uint8_t> dataVec;

            if (JS_IsString(data.getVal())) {
                auto str = data.toString();
                dataVec.resize(str.size());
                std::copy(str.begin(), str.end(), dataVec.begin());
            }
            else {
                auto& machine = *static_cast<I2CFeature*>(JS_GetContextOpaque(ctx));
                jac::Value toArrayBuffer = machine.eval(
R"--(
(data) => {
    if (data instanceof ArrayBuffer) return data;
    if (ArrayBuffer.isView(data)) return data.buffer;
    if (typeof data === 'number') return new Int8Array([data]).buffer;
    if (Array.isArray(data)) return new Int8Array(data).buffer;
    throw new Error('Invalid data type');
}
)--", "<I2CFeature::writeTo>");

                auto res = toArrayBuffer.to<jac::Function>().call<jac::ArrayBuffer>(data);
                auto dataView = res.typedView<uint8_t>();
                dataVec.resize(dataView.size());
                std::copy(dataView.begin(), dataView.end(), dataVec.begin());
            }

            return dataVec;
        };

        proto.defineProperty("readFrom", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, int address, int quantity) {
            auto& i2c = *I2CProtoBuilder::getOpaque(ctx, thisVal);
            auto data = i2c.readFrom(address, quantity);

            return toUint8Array(ctx, data);
        }));

        proto.defineProperty("writeTo", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, int address, jac::Value data) {
            auto& i2c = *I2CProtoBuilder::getOpaque(ctx, thisVal);
            auto dataVec = toStdVector(ctx, data);

            i2c.writeTo(address, std::move(dataVec));
        }));

        proto.defineProperty("writeRead", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, int address, jac::Value data, int quantity) {
            auto& i2c = *I2CProtoBuilder::getOpaque(ctx, thisVal);
            auto dataVec = toStdVector(ctx, data);
            auto res = i2c.writeRead(address, std::move(dataVec), quantity);

            return toUint8Array(ctx, res);
        }));

        proto.defineProperty("setup", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, jac::Object options) {
            auto& i2c = *I2CProtoBuilder::getOpaque(ctx, thisVal);

            std::optional<int> scl;
            std::optional<int> sda;
            std::optional<int> bitrate;

            if (options.hasProperty("scl")) {
                scl = options.get<int>("scl");
            }
            if (options.hasProperty("sda")) {
                sda = options.get<int>("sda");
            }
            if (options.hasProperty("bitrate")) {
                bitrate = options.get<int>("bitrate");
            }

            i2c.setup(scl, sda, bitrate);
        }));
    }
};


template<class Next>
class I2CFeature : public Next {
public:
    using I2CClass = jac::Class<I2CProtoBuilder<I2CFeature>>;

    I2CFeature() {
        I2CClass::init("I2C");
    }

    void initialize() {
        Next::initialize();

        jac::Module& mod = this->newModule("i2c");
        for (int i = 0; i < SOC_I2C_NUM; ++i) {
            mod.addExport("I2C" + std::to_string(i), I2CClass::createInstance(this->context(), new I2C<typename Next::PlatformInfo>(i)));
        }
    }
};
