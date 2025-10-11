#pragma once


#include <jac/machine/functionFactory.h>
#include <jac/machine/machine.h>
#include <jac/machine/values.h>

#include <array>
#include <atomic>
#include <map>
#include <memory>

#include "../util/capsAllocator.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"


enum class PinMode {
    DISABLE,
    OUTPUT,
    INPUT,
    INPUT_PULLUP,
    INPUT_PULLDOWN
};

enum class InterruptMode {
    DISABLE = static_cast<int>(GPIO_INTR_DISABLE),
    RISING = static_cast<int>(GPIO_INTR_POSEDGE),
    FALLING = static_cast<int>(GPIO_INTR_NEGEDGE),
    CHANGE = static_cast<int>(GPIO_INTR_ANYEDGE),
};

template<>
struct jac::ConvTraits<PinMode> {
    static Value to(ContextRef ctx, PinMode val) {
        int mode = static_cast<int>(val);
        return Value::from(ctx, mode);
    }

    static PinMode from(ContextRef ctx, ValueWeak val) {
        int mode = val.to<int>();
        return static_cast<PinMode>(mode);
    }
};


namespace detail {

    using Callback_t = std::function<void(std::chrono::time_point<std::chrono::steady_clock>)>;

    template<size_t N>
    class InterruptQueue {
        using ArrayType = std::array<std::shared_ptr<Callback_t>, N>;
        ArrayType queue;
        ArrayType::iterator head = queue.begin();
        ArrayType::iterator tail = queue.begin();

        auto next(ArrayType::iterator it) {
            it++;
            return it == queue.end() ? queue.begin() : it;
        }

    public:
        bool push(std::shared_ptr<Callback_t> callback) {
            if (next(tail) == head) {
                return false;
            }
            *tail = callback;
            tail = next(tail);
            return true;
        }

        std::shared_ptr<Callback_t> pop() {
            if (head == tail) {
                return nullptr;
            }
            auto callback = *head;
            *head = nullptr;
            head = next(head);
            return callback;
        }

        bool empty() {
            return head == tail;
        }
    };

    template<typename Holder>
    class Interrupts {
        static constexpr size_t DEBOUNCE_TIME = 2;

        std::shared_ptr<Callback_t> rising;
        std::shared_ptr<Callback_t> falling;
        std::shared_ptr<Callback_t> change;
        gpio_num_t pin;
        TickType_t lastTime = 0;
        bool lastRising = false;
        Holder* holder;
    public:
        Interrupts(gpio_num_t pin, Holder* holder) : pin(pin), holder(holder) {}

        std::shared_ptr<Callback_t>& operator[](InterruptMode mode) {
            switch (mode) {
                case InterruptMode::RISING:
                    return rising;
                case InterruptMode::FALLING:
                    return falling;
                case InterruptMode::CHANGE:
                    return change;
                default:
                    throw std::runtime_error("Invalid interrupt mode");
            }
        }

        operator bool() const {
            return rising || falling || change;
        }

        gpio_num_t getPin() const {
            return pin;
        }

        Holder* getHolder() {
            return holder;
        }

        bool updateLast(bool risingEdge) {
            auto now = xTaskGetTickCountFromISR();
            if (lastRising == risingEdge && now - lastTime < DEBOUNCE_TIME) {
                return false;
            }
            lastTime = now;
            lastRising = risingEdge;
            return true;
        }
    };

} // namespace detail


template<class GpioFeature>
class Gpio {
    struct Holder;

    using InterruptQueue_ = detail::InterruptQueue<32>;
    using Interrupts_ = detail::Interrupts<Holder>;

    struct Holder {
        InterruptQueue_ _interruptQueue;
        std::map<int, Interrupts_> _interruptCallbacks;
        TaskHandle_t shovelTaskHandle = nullptr;
    };
    std::unique_ptr<Holder, EspCapsDeleter<MALLOC_CAP_INTERNAL, Holder>> _holder;

    GpioFeature* _feature;
    std::atomic<bool> stopShovel{false};

    static void shovel(void* pvParameters) {
        auto& gpio = *static_cast<Gpio*>(pvParameters);
        while (true) {
            auto res = xTaskNotifyWait(pdFALSE, ULONG_MAX, nullptr, portMAX_DELAY);
            if (gpio.stopShovel) {
                vTaskDelete(nullptr);
                return;
            }
            if (res == pdFALSE) {
                continue;
            }
            auto& queue = gpio._holder->_interruptQueue;
            if (queue.empty()) {
                continue;
            }
            auto callback = queue.pop();
            gpio._feature->scheduleEvent([callback]() {
                auto now = std::chrono::steady_clock::now();
                (*callback)(now);
            });
        }
    }
public:
    Gpio(GpioFeature* feature) : _feature(feature) {
        EspCapsAllocator<MALLOC_CAP_INTERNAL, Holder> alloc;
        auto mem = alloc.allocate(1);
        alloc.construct(mem);
        _holder.reset(mem);

        auto res = xTaskCreate(
            shovel,
            "gpio_shovel",
            2048,
            this,
            tskIDLE_PRIORITY + 1,
            &_holder->shovelTaskHandle
        );
        if (res != pdPASS) {
            throw std::runtime_error("Failed to create GPIO shovel task");
        }
    }

    ~Gpio() {
        stopShovel = true;
        if (_holder->shovelTaskHandle) {
            xTaskNotify(_holder->shovelTaskHandle, 0, eNoAction);
            while (true) {
                eTaskState state = eTaskGetState(_holder->shovelTaskHandle);
                if (state == eDeleted || state == eInvalid) {
                    break;
                }
            }
        }
    }

    void pinMode(int pinNum, PinMode mode) {
        gpio_num_t pin = GpioFeature::getDigitalPin(pinNum);

        switch (mode) {
        case PinMode::DISABLE:
            gpio_set_direction(pin, GPIO_MODE_DISABLE);
            break;
        case PinMode::OUTPUT:
            gpio_set_direction(pin, GPIO_MODE_OUTPUT);
            break;
            break;
        case PinMode::INPUT:
            gpio_set_direction(pin, GPIO_MODE_INPUT);
            gpio_set_pull_mode(pin, GPIO_FLOATING);
            break;
        case PinMode::INPUT_PULLUP:
            gpio_set_direction(pin, GPIO_MODE_INPUT);
            gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
            break;
        case PinMode::INPUT_PULLDOWN:
            gpio_set_direction(pin, GPIO_MODE_INPUT);
            gpio_set_pull_mode(pin, GPIO_PULLDOWN_ONLY);
            break;
        }
    }

    void write(int pinNum, int value) {
        gpio_num_t pin = GpioFeature::getDigitalPin(pinNum);
        gpio_set_level(pin, value);
    }

    int read(int pinNum) {
        gpio_num_t pin = GpioFeature::getDigitalPin(pinNum);
        return gpio_get_level(pin);
    }

    void attachInterrupt(int pinNum, InterruptMode mode, detail::Callback_t callback) {
        gpio_num_t pin = GpioFeature::getInterruptPin(pinNum);

        auto it = _holder->_interruptCallbacks.find(pinNum);
        if (it == _holder->_interruptCallbacks.end()) {
            it = _holder->_interruptCallbacks.emplace(
                std::piecewise_construct,
                std::forward_as_tuple(pinNum),
                std::forward_as_tuple(pin, _holder.get())
            ).first;
            gpio_set_intr_type(pin, GPIO_INTR_ANYEDGE);
            gpio_isr_handler_add(pin, [](void* arg) {
                auto& callbacks = *static_cast<Interrupts_*>(arg);
                auto& holder = *callbacks.getHolder();
                auto& interruptQueue = holder._interruptQueue;

                bool risingEdge = gpio_get_level(callbacks.getPin()) == 1;
                if (!callbacks.updateLast(risingEdge)) {
                    return;
                }

                bool generated = false;
                if (callbacks[InterruptMode::CHANGE]) {
                    interruptQueue.push(callbacks[InterruptMode::CHANGE]);
                    generated = true;
                }
                if (callbacks[InterruptMode::RISING] && risingEdge) {
                    interruptQueue.push(callbacks[InterruptMode::RISING]);
                    generated = true;
                }
                if (callbacks[InterruptMode::FALLING] && !risingEdge) {
                    interruptQueue.push(callbacks[InterruptMode::FALLING]);
                    generated = true;
                }

                if (generated) {
                    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                    xTaskNotifyFromISR(holder.shovelTaskHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
                    if (xHigherPriorityTaskWoken) {
                        portYIELD_FROM_ISR();
                    }
                }
            }, &(it->second));
            gpio_intr_enable(pin);
        }

        if ((it->second)[mode]) {
            throw std::runtime_error("Interrupt already attached");
        }

        EspCapsAllocator<MALLOC_CAP_INTERNAL, detail::Callback_t> alloc;
        (it->second)[mode] = std::allocate_shared<detail::Callback_t>(alloc, std::move(callback));
    }

    void detachInterrupt(int pinNum, InterruptMode mode) {
        gpio_num_t pin = GpioFeature::getInterruptPin(pinNum);

        auto it = _holder->_interruptCallbacks.find(pinNum);
        if (it == _holder->_interruptCallbacks.end() || !(it->second)[mode]) {
            throw std::runtime_error("Interrupt not attached");
        }

        (it->second)[mode] = nullptr;

        if (!it->second) {
            gpio_intr_disable(pin);
            gpio_isr_handler_remove(pin);
            _holder->_interruptCallbacks.erase(pinNum);
        }
    }

    void on(std::string event, int pinNum, detail::Callback_t callback) {
        if (event == "rising") {
            attachInterrupt(pinNum, InterruptMode::RISING, std::move(callback));
        }
        else if (event == "falling") {
            attachInterrupt(pinNum, InterruptMode::FALLING, std::move(callback));
        }
        else if (event == "change") {
            attachInterrupt(pinNum, InterruptMode::CHANGE, std::move(callback));
        }
        else {
            throw std::runtime_error("Invalid event");
        }
    }

    void off(std::string event, int pinNum) {
        if (event == "rising") {
            detachInterrupt(pinNum, InterruptMode::RISING);
        }
        else if (event == "falling") {
            detachInterrupt(pinNum, InterruptMode::FALLING);
        }
        else if (event == "change") {
            detachInterrupt(pinNum, InterruptMode::CHANGE);
        }
        else {
            throw std::runtime_error("Invalid event");
        }
    }
};


template<class Next>
class GpioFeature : public Next {
    using PinConfig = Next::PlatformInfo::PinConfig;
    using Gpio_ = Gpio<GpioFeature>;
public:
    Gpio_ gpio;

    void initialize() {
        Next::initialize();

        for (auto pin : PinConfig::DIGITAL_PINS) {
            gpio.pinMode(pin, PinMode::DISABLE);
        }
        for (auto pin : PinConfig::INTERRUPT_PINS) {
            gpio_intr_disable(Next::getDigitalPin(pin));
        }

        gpio_install_isr_service(0);

        jac::FunctionFactory ff(this->context());
        auto& module = this->newModule("gpio");
        module.addExport("pinMode", ff.newFunction(noal::function(&Gpio_::pinMode, &gpio)));
        module.addExport("read", ff.newFunction(noal::function(&Gpio_::read, &gpio)));
        module.addExport("write", ff.newFunction(noal::function(&Gpio_::write, &gpio)));

        // TODO: rename pinMode and PinMode to avoid confusion
        jac::Object pinModeEnum = jac::Object::create(this->context());
        pinModeEnum.set("DISABLE", static_cast<int>(PinMode::DISABLE));
        pinModeEnum.set("OUTPUT", static_cast<int>(PinMode::OUTPUT));
        pinModeEnum.set("INPUT", static_cast<int>(PinMode::INPUT));
        pinModeEnum.set("INPUT_PULLUP", static_cast<int>(PinMode::INPUT_PULLUP));
        pinModeEnum.set("INPUT_PULLDOWN", static_cast<int>(PinMode::INPUT_PULLDOWN));
        module.addExport("PinMode", pinModeEnum);

        module.addExport("on", ff.newFunction([this](std::string event, int pin, jac::Function callback) {
            this->gpio.on(event, pin, [this, callback = std::move(callback)](Next::TimePoint timestamp) mutable {
                jac::Object info = jac::Object::create(this->context());
                info.set("timestamp", this->createTimestamp(timestamp));

                callback.call<void>(info);
            });
        }));
        module.addExport("off", ff.newFunction(noal::function(&Gpio_::off, &gpio)));
    }

    GpioFeature() : gpio(this) {}

    ~GpioFeature() {
        for (auto pin : PinConfig::INTERRUPT_PINS) {
            gpio_intr_disable(Next::getDigitalPin(pin));
            gpio_isr_handler_remove(Next::getDigitalPin(pin));
        }

        gpio_uninstall_isr_service();
    }
};
