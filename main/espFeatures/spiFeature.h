#pragma once

#include <jac/machine/class.h>
#include <jac/machine/functionFactory.h>
#include <jac/machine/machine.h>

#include <memory>
#include <noal_func.h>
#include <optional>
#include <array>

#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include <chrono>


class SPI {
    spi_host_device_t host;
    spi_device_handle_t deviceHandle;
    bool open = false;

public:
    SPI(int hostId) : host(static_cast<spi_host_device_t>(hostId)) {}

    std::vector<uint8_t> transfer(std::vector<uint8_t> data, int cs, int rxLength = 0, bool qio = false) {
        if (!open) {
            throw std::runtime_error("SPI not configured");
        }
        gpio_set_direction(static_cast<gpio_num_t>(cs), GPIO_MODE_OUTPUT);
        gpio_set_level(static_cast<gpio_num_t>(cs), 0);

        std::vector<uint8_t> rx(rxLength > 0 ? rxLength : data.size());

        spi_transaction_t transaction = {
            .flags = static_cast<uint32_t>(qio ? SPI_TRANS_MODE_QIO : 0),
            .cmd = 0,
            .addr = 0,
            .length = data.size() * 8,
            .rxlength = rx.size() * 8,
            .user = nullptr,
            .tx_buffer = data.data(),
            .rx_buffer = rx.data()
        };

        esp_err_t err = spi_device_transmit(deviceHandle, &transaction);
        if (err != ESP_OK) {
            throw std::runtime_error(esp_err_to_name(err));
        }

        gpio_set_level(static_cast<gpio_num_t>(cs), 1);

        return rx;
    }

    void setup(std::optional<int> sck, std::array<int, 4> dataPins, std::optional<int> baud, std::optional<int> mode, std::optional<bool> lsb) {
        int sck_ = sck.value_or(-1);
        int baud_ = baud.value_or(10'000'000);
        int mode_ = mode.value_or(0);
        bool lsb_ = lsb.value_or(false);

        spi_bus_config_t busConfig = {
            .data0_io_num = dataPins[0],
            .data1_io_num = dataPins[1],
            .sclk_io_num = sck_,
            .data2_io_num = dataPins[2],
            .data3_io_num = dataPins[3],
            .data4_io_num = -1,
            .data5_io_num = -1,
            .data6_io_num = -1,
            .data7_io_num = -1,
            .max_transfer_sz = 0,
            .flags = 0,
            .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
            .intr_flags = 0,
        };

        esp_err_t err = spi_bus_initialize(host, &busConfig, SPI_DMA_DISABLED);
        if (err != ESP_OK) {
            throw std::runtime_error(esp_err_to_name(err));
        }

        spi_device_interface_config_t deviceConfigTemplate{
            .command_bits = 0,
            .address_bits = 0,
            .dummy_bits = 0,
            .mode = static_cast<uint8_t>(mode_),
            .clock_source = SPI_CLK_SRC_DEFAULT,
            .duty_cycle_pos = 0,
            .cs_ena_pretrans = 0,
            .cs_ena_posttrans = 0,
            .clock_speed_hz = baud_,
            .input_delay_ns = 0,
            .spics_io_num = -1,
            .flags = static_cast<uint32_t>(lsb_ ? (SPI_DEVICE_TXBIT_LSBFIRST | SPI_DEVICE_RXBIT_LSBFIRST) : 0),
            .queue_size = 1,
            .pre_cb = nullptr,
            .post_cb = nullptr,
        };

        err = spi_bus_add_device(host, &deviceConfigTemplate, &deviceHandle);
        if (err != ESP_OK) {
            spi_bus_free(host);
            throw std::runtime_error(esp_err_to_name(err));
        }

        open = true;
    }

    void close() {
        if (open) {
            spi_bus_remove_device(deviceHandle);
            spi_bus_free(host);
            open = false;
        }
    }

    ~SPI() {
        close();
    }
};


template<class SPIFeature>
struct SPIProtoBuilder : public jac::ProtoBuilder::Opaque<SPI>, public jac::ProtoBuilder::Properties {
    static void addProperties(JSContext* ctx, jac::Object proto) {
        jac::FunctionFactory ff(ctx);

        proto.defineProperty("send", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, jac::Value data, int cs) {
            auto& feature = *reinterpret_cast<SPIFeature*>(JS_GetContextOpaque(ctx));  // NOLINT
            auto& spi = *SPIProtoBuilder::getOpaque(ctx, thisVal);
            auto dataVec = feature.toStdVector(data);
            auto rx = spi.transfer(std::move(dataVec), cs);
            return feature.toUint8Array(rx);
        }));
        proto.defineProperty("write", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, jac::Value data, int cs) {
        std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
            auto& feature = *reinterpret_cast<SPIFeature*>(JS_GetContextOpaque(ctx));  // NOLINT
            auto& spi = *SPIProtoBuilder::getOpaque(ctx, thisVal);
            auto dataVec = feature.toStdVector(data);
            spi.transfer(std::move(dataVec), cs);
            std::chrono::time_point<std::chrono::steady_clock> end = std::chrono::steady_clock::now();
            jac::Logger::debug("transfer took " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()) + "ms");
        }));
        proto.defineProperty("transfer", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, jac::Value data, int cs, int rxLength, bool qio) {
            auto& feature = *reinterpret_cast<SPIFeature*>(JS_GetContextOpaque(ctx));  // NOLINT
            auto& spi = *SPIProtoBuilder::getOpaque(ctx, thisVal);
            auto dataVec = feature.toStdVector(data);
            auto rx = spi.transfer(std::move(dataVec), cs, rxLength, qio);
            return feature.toUint8Array(rx);
        }));
        proto.defineProperty("setup", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, jac::Object options) {
            auto& spi = *SPIProtoBuilder::getOpaque(ctx, thisVal);

            std::optional<int> sck;
            std::array<int, 4> dataPins = { -1, -1, -1, -1 };
            std::optional<int> baud;
            std::optional<int> mode;
            std::optional<bool> lsb = false;

            bool hasMosi = options.hasProperty("mosi");
            bool hasData0 = options.hasProperty("data0");
            if (hasMosi && hasData0) {
                throw jac::Exception::create(jac::Exception::Type::TypeError, "Cannot specify both mosi and data0");
            }
            bool hasMiso = options.hasProperty("miso");
            bool hasData1 = options.hasProperty("data1");
            if (hasMiso && hasData1) {
                throw jac::Exception::create(jac::Exception::Type::TypeError, "Cannot specify both miso and data1");
            }

            if (options.hasProperty("sck")) { sck = options.get<int>("sck"); }
            if (hasMosi) { dataPins[0] = options.get<int>("mosi"); }
            if (hasData0) { dataPins[0] = options.get<int>("data0"); }
            if (hasMiso) { dataPins[1] = options.get<int>("miso"); }
            if (hasData1) { dataPins[1] = options.get<int>("data1"); }
            if (options.hasProperty("data2")) { dataPins[2] = options.get<int>("data2"); }
            if (options.hasProperty("data3")) { dataPins[3] = options.get<int>("data3"); }
            if (options.hasProperty("baud")) { baud = options.get<int>("baud"); }
            if (options.hasProperty("mode")) { mode = options.get<int>("mode"); }
            if (options.hasProperty("order")) {
                auto order = options.get<std::string>("order");
                if (order == "lsb") {
                    lsb = true;
                }
                else if (order == "msb") {
                    lsb = false;
                }
                else {
                    throw jac::Exception::create(jac::Exception::Type::TypeError, "Invalid bit order");
                }
            }

            spi.setup(sck, dataPins, baud, mode, lsb);
        }));
    }
};

template<class Next>
class SPIFeature : public Next {
public:
    using SPIClass = jac::Class<SPIProtoBuilder<SPIFeature>>;

    SPIFeature() { SPIClass::init("SPI"); }

    void initialize() {
        Next::initialize();

        jac::Module& mod = this->newModule("spi");
        for (int i = 0; i < SPI_HOST_MAX; ++i) {
            mod.addExport("SPI" + std::to_string(i + 1), SPIClass::createInstance(this->context(), new SPI(i)));
        }
    }
};
