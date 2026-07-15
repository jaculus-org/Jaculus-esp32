#pragma once

#include <jac/machine/class.h>
#include <jac/machine/functionFactory.h>
#include <jac/machine/machine.h>
#include <jac/machine/values.h>

#include <atomic>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "driver/uart.h"


struct SerialOptions {
    int tx;
    int rx;
    int baudRate;
    uart_parity_t parity;
    uart_stop_bits_t stopBits;
    uart_word_length_t dataBits;
    int rxBufferSize;
    int txBufferSize;
    bool invertTx;
    bool invertRx;
};


template<class Feature>
struct SerialProtoBuilder;


template<class Feature>
class Serial {
    friend struct SerialProtoBuilder<Feature>;

    enum class PendingType {
        Get,
        Read,
    };

    struct PendingRequest {
        PendingType type;
        jac::Function resolve;
        jac::Function reject;
    };

    Feature* _feature;
    uart_port_t _port;
    QueueHandle_t _eventQueue = nullptr;
    std::thread _eventThread;
    std::atomic<bool> _open = false;
    std::mutex _rxMutex;
    std::deque<PendingRequest> _pending;

    void ensureOpen() const {
        if (!_open) {
            throw jac::Exception::create(jac::Exception::Type::InternalError, "Serial port is not open");
        }
    }

    bool tryReadByte(std::vector<uint8_t>& data) {
        uint8_t ch = 0;
        int read = uart_read_bytes(_port, &ch, 1, 0);
        if (read <= 0) {
            return false;
        }

        data = {ch};
        return true;
    }

    bool tryReadAll(std::vector<uint8_t>& data) {
        size_t bufferedSize = 0;
        esp_err_t err = uart_get_buffered_data_len(_port, &bufferedSize);
        if (err != ESP_OK) {
            throw jac::Exception::create(jac::Exception::Type::InternalError, std::string("Failed to query UART buffer length (") + esp_err_to_name(err) + ")");
        }
        if (bufferedSize == 0) {
            return false;
        }

        data.resize(bufferedSize);
        int read = uart_read_bytes(_port, data.data(), data.size(), 0);
        if (read <= 0) {
            data.clear();
            return false;
        }

        data.resize(read);
        return true;
    }

    void processPending() {
        std::lock_guard<std::mutex> lock(_rxMutex);

        while (!_pending.empty()) {
            std::vector<uint8_t> data;
            bool hasData = _pending.front().type == PendingType::Get
                ? tryReadByte(data)
                : tryReadAll(data);

            if (!hasData) {
                break;
            }

            PendingRequest request = std::move(_pending.front());
            _pending.pop_front();
            request.resolve.template call<void>(_feature->toUint8Array(std::move(data)));
        }
    }

    void rejectPending() {
        std::lock_guard<std::mutex> lock(_rxMutex);
        while (!_pending.empty()) {
            PendingRequest request = std::move(_pending.front());
            _pending.pop_front();
            request.reject.template call<void>(jac::Exception::create(jac::Exception::Type::Error, "Serial port is closed"));
        }
    }

    void processRx() {
        if (!_open) {
            return;
        }

        processPending();
    }

public:
    Serial(Feature* feature, uart_port_t port):
        _feature(feature),
        _port(port)
    {}

    void setup(const SerialOptions& options) {
        close();

        if (options.baudRate <= 0) {
            throw jac::Exception::create(jac::Exception::Type::RangeError, "baudRate must be greater than 0");
        }
        if (options.rxBufferSize <= 0) {
            throw jac::Exception::create(jac::Exception::Type::RangeError, "rxBufferSize must be greater than 0");
        }
        if (options.txBufferSize < 0) {
            throw jac::Exception::create(jac::Exception::Type::RangeError, "txBufferSize must be greater than or equal to 0");
        }

        uart_config_t config = {
            .baud_rate = options.baudRate,
            .data_bits = options.dataBits,
            .parity = options.parity,
            .stop_bits = options.stopBits,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0,
            .source_clk = UART_SCLK_DEFAULT,
        };

        int txPin = Feature::getDigitalPin(options.tx);
        int rxPin = Feature::getDigitalPin(options.rx);

        esp_err_t err = uart_driver_install(_port, options.rxBufferSize, options.txBufferSize, 10, &_eventQueue, 0);
        if (err != ESP_OK) {
            throw jac::Exception::create(jac::Exception::Type::InternalError, std::string("Failed to install UART driver (") + esp_err_to_name(err) + ")");
        }

        err = uart_param_config(_port, &config);
        if (err != ESP_OK) {
            uart_driver_delete(_port);
            _eventQueue = nullptr;
            throw jac::Exception::create(jac::Exception::Type::InternalError, std::string("Failed to configure UART (") + esp_err_to_name(err) + ")");
        }

        err = uart_set_pin(_port, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (err != ESP_OK) {
            uart_driver_delete(_port);
            _eventQueue = nullptr;
            throw jac::Exception::create(jac::Exception::Type::InternalError, std::string("Failed to configure UART pins (") + esp_err_to_name(err) + ")");
        }

        uint32_t inverseMask = 0;
        if (options.invertTx) {
            inverseMask |= UART_SIGNAL_TXD_INV;
        }
        if (options.invertRx) {
            inverseMask |= UART_SIGNAL_RXD_INV;
        }
        err = uart_set_line_inverse(_port, inverseMask);
        if (err != ESP_OK) {
            uart_driver_delete(_port);
            _eventQueue = nullptr;
            throw jac::Exception::create(jac::Exception::Type::InternalError, std::string("Failed to set UART signal inversion (") + esp_err_to_name(err) + ")");
        }

        _open = true;

        _eventThread = std::thread([this]() noexcept {
            uart_event_t event{};
            while (_open) {
                if (xQueueReceive(_eventQueue, &event, pdMS_TO_TICKS(100)) != pdTRUE) {
                    continue;
                }

                if (!_open) {
                    break;
                }

                switch (event.type) {
                    case UART_DATA:
                        _feature->scheduleEvent([this]() {
                            this->processRx();
                        });
                        break;
                    case UART_FIFO_OVF:
                    case UART_BUFFER_FULL:
                        uart_flush_input(_port);
                        xQueueReset(_eventQueue);
                        break;
                    default:
                        break;
                }
            }
        });
    }

    jac::Promise get(jac::ContextRef ctx) {
        ensureOpen();

        auto [promise, resolve, reject] = jac::Promise::create(ctx);

        {
            std::lock_guard<std::mutex> lock(_rxMutex);
            std::vector<uint8_t> data;
            if (_pending.empty() && tryReadByte(data)) {
                resolve.call<void>(_feature->toUint8Array(std::move(data)));
                return promise;
            }

            _pending.push_back(PendingRequest{
                .type = PendingType::Get,
                .resolve = std::move(resolve),
                .reject = std::move(reject),
            });
        }

        return promise;
    }

    jac::Promise read(jac::ContextRef ctx) {
        ensureOpen();

        auto [promise, resolve, reject] = jac::Promise::create(ctx);

        {
            std::lock_guard<std::mutex> lock(_rxMutex);
            std::vector<uint8_t> data;
            if (_pending.empty() && tryReadAll(data)) {
                resolve.call<void>(_feature->toUint8Array(std::move(data)));
                return promise;
            }

            _pending.push_back(PendingRequest{
                .type = PendingType::Read,
                .resolve = std::move(resolve),
                .reject = std::move(reject),
            });
        }

        return promise;
    }

    void write(std::vector<uint8_t> data) {
        ensureOpen();

        int written = uart_write_bytes(_port, reinterpret_cast<const char*>(data.data()), data.size());
        if (written < 0 || written != static_cast<int>(data.size())) {
            throw jac::Exception::create(jac::Exception::Type::InternalError, "Failed to write to UART");
        }
    }

    void flush() {
        ensureOpen();

        esp_err_t err = uart_wait_tx_done(_port, portMAX_DELAY);
        if (err != ESP_OK) {
            throw jac::Exception::create(jac::Exception::Type::InternalError, std::string("Failed to flush UART (") + esp_err_to_name(err) + ")");
        }
    }

    void closeNative(bool rejectPromises) {
        if (!_open.exchange(false)) {
            return;
        }

        if (rejectPromises) {
            rejectPending();
        }
        else {
            std::lock_guard<std::mutex> lock(_rxMutex);
            _pending.clear();
        }

        if (_eventThread.joinable()) {
            _eventThread.join();
        }

        uart_driver_delete(_port);
        _eventQueue = nullptr;
    }

    ~Serial() {
        closeNative(false);
    }

    void close() {
        closeNative(true);
    }
};


template<class Feature>
struct SerialProtoBuilder : public jac::ProtoBuilder::Opaque<Serial<Feature>>, public jac::ProtoBuilder::Properties {
    using Serial_ = Serial<Feature>;

    static SerialOptions optionsFromObject(jac::Object options) {
        if (!options.hasProperty("tx")) {
            throw jac::Exception::create(jac::Exception::Type::TypeError, "Missing required property 'tx'");
        }
        if (!options.hasProperty("rx")) {
            throw jac::Exception::create(jac::Exception::Type::TypeError, "Missing required property 'rx'");
        }
        if (!options.hasProperty("baudRate")) {
            throw jac::Exception::create(jac::Exception::Type::TypeError, "Missing required property 'baudRate'");
        }

        SerialOptions config{
            .tx = options.get<int>("tx"),
            .rx = options.get<int>("rx"),
            .baudRate = options.get<int>("baudRate"),
            .parity = UART_PARITY_DISABLE,
            .stopBits = UART_STOP_BITS_1,
            .dataBits = UART_DATA_8_BITS,
            .rxBufferSize = 1024,
            .txBufferSize = 1024,
            .invertTx = false,
            .invertRx = false,
        };

        if (options.hasProperty("parity")) {
            config.parity = static_cast<uart_parity_t>(options.get<int>("parity"));
        }
        if (options.hasProperty("stopBits")) {
            config.stopBits = static_cast<uart_stop_bits_t>(options.get<int>("stopBits"));
        }
        if (options.hasProperty("dataBits")) {
            config.dataBits = static_cast<uart_word_length_t>(options.get<int>("dataBits"));
        }
        if (options.hasProperty("rxBufferSize")) {
            config.rxBufferSize = options.get<int>("rxBufferSize");
        }
        if (options.hasProperty("txBufferSize")) {
            config.txBufferSize = options.get<int>("txBufferSize");
        }
        if (options.hasProperty("invertTx")) {
            config.invertTx = options.get<bool>("invertTx");
        }
        if (options.hasProperty("invertRx")) {
            config.invertRx = options.get<bool>("invertRx");
        }

        if (config.parity != UART_PARITY_DISABLE && config.parity != UART_PARITY_EVEN && config.parity != UART_PARITY_ODD) {
            throw jac::Exception::create(jac::Exception::Type::RangeError, "Invalid parity");
        }
        if (config.stopBits != UART_STOP_BITS_1 && config.stopBits != UART_STOP_BITS_2) {
            throw jac::Exception::create(jac::Exception::Type::RangeError, "Invalid stopBits");
        }
        if (config.dataBits != UART_DATA_5_BITS && config.dataBits != UART_DATA_6_BITS &&
            config.dataBits != UART_DATA_7_BITS && config.dataBits != UART_DATA_8_BITS) {
            throw jac::Exception::create(jac::Exception::Type::RangeError, "Invalid dataBits");
        }

        return config;
    }

    static void addProperties(jac::ContextRef ctx, jac::Object proto) {
        jac::FunctionFactory ff(ctx);

        proto.defineProperty("setup", ff.newFunctionThis([](jac::ContextRef ctx_, jac::ValueWeak thisVal, jac::Object options) {
            auto& self = *SerialProtoBuilder::getOpaque(ctx_, thisVal);
            self.setup(optionsFromObject(options));
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("write", ff.newFunctionThis([](jac::ContextRef ctx_, jac::ValueWeak thisVal, jac::Value data) {
            auto& feature = *reinterpret_cast<Feature*>(JS_GetContextOpaque(ctx_));  // NOLINT
            auto& self = *SerialProtoBuilder::getOpaque(ctx_, thisVal);
            self.write(feature.toStdVector(data));
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("get", ff.newFunctionThis([](jac::ContextRef ctx_, jac::ValueWeak thisVal) {
            auto& self = *SerialProtoBuilder::getOpaque(ctx_, thisVal);
            return self.get(ctx_);
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("read", ff.newFunctionThis([](jac::ContextRef ctx_, jac::ValueWeak thisVal) {
            auto& self = *SerialProtoBuilder::getOpaque(ctx_, thisVal);
            return self.read(ctx_);
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("flush", ff.newFunctionThis([](jac::ContextRef ctx_, jac::ValueWeak thisVal) {
            auto& self = *SerialProtoBuilder::getOpaque(ctx_, thisVal);
            self.flush();
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("close", ff.newFunctionThis([](jac::ContextRef ctx_, jac::ValueWeak thisVal) {
            auto& self = *SerialProtoBuilder::getOpaque(ctx_, thisVal);
            self.close();
        }), jac::PropFlags::Enumerable);
    }
};


template<class Next>
class SerialFeature : public Next {
    std::vector<jac::Object> _serials;
public:
    using SerialClass = jac::Class<SerialProtoBuilder<SerialFeature<Next>>>;

    SerialFeature() {
        SerialClass::init("Serial");
    }

    ~SerialFeature() {
        for (auto& serial : _serials) {
            auto* ptr = SerialProtoBuilder<SerialFeature<Next>>::getOpaque(this->context(), serial);
            ptr->closeNative(false);
        }
    }

    void initialize() {
        Next::initialize();

        auto& mod = this->newModule("serial");

        jac::Object parity = jac::Object::create(this->context());
        parity.defineProperty("None", jac::Value::from(this->context(), static_cast<int>(UART_PARITY_DISABLE)), jac::PropFlags::Enumerable);
        parity.defineProperty("Even", jac::Value::from(this->context(), static_cast<int>(UART_PARITY_EVEN)), jac::PropFlags::Enumerable);
        parity.defineProperty("Odd", jac::Value::from(this->context(), static_cast<int>(UART_PARITY_ODD)), jac::PropFlags::Enumerable);

        jac::Object stopBits = jac::Object::create(this->context());
        stopBits.defineProperty("One", jac::Value::from(this->context(), static_cast<int>(UART_STOP_BITS_1)), jac::PropFlags::Enumerable);
        stopBits.defineProperty("Two", jac::Value::from(this->context(), static_cast<int>(UART_STOP_BITS_2)), jac::PropFlags::Enumerable);

        jac::Object dataBits = jac::Object::create(this->context());
        dataBits.defineProperty("Five", jac::Value::from(this->context(), static_cast<int>(UART_DATA_5_BITS)), jac::PropFlags::Enumerable);
        dataBits.defineProperty("Six", jac::Value::from(this->context(), static_cast<int>(UART_DATA_6_BITS)), jac::PropFlags::Enumerable);
        dataBits.defineProperty("Seven", jac::Value::from(this->context(), static_cast<int>(UART_DATA_7_BITS)), jac::PropFlags::Enumerable);
        dataBits.defineProperty("Eight", jac::Value::from(this->context(), static_cast<int>(UART_DATA_8_BITS)), jac::PropFlags::Enumerable);

        mod.addExport("Parity", parity);
        mod.addExport("StopBits", stopBits);
        mod.addExport("DataBits", dataBits);

        for (int i = 1; i < SOC_UART_HP_NUM; ++i) {
            jac::Object serial = SerialClass::createInstance(this->context(), new Serial<SerialFeature<Next>>(this, static_cast<uart_port_t>(i))).template to<jac::Object>();
            _serials.emplace_back(serial);
            mod.addExport("Serial" + std::to_string(i), std::move(serial));
        }
    }
};
