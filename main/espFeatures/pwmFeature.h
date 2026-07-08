#pragma once

#include <jac/machine/class.h>
#include <jac/machine/functionFactory.h>
#include <jac/machine/machine.h>

#include <functional>
#include <map>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include "driver/ledc.h"


enum class PwmType {
    Fixed,
    Flexible,
};

struct PwmTimerState {
    bool exclusive;
    int frequency;
    int resolution;
    int refCount;
};

struct PwmChannelState {
    ledc_timer_t timer;
    gpio_num_t pin;
};

template<class Feature>
class PwmManager;

template<class Feature, PwmType Type>
class PwmReservation {
    friend class PwmManager<Feature>;

    PwmManager<Feature>* _manager;
    gpio_num_t _pin;
    ledc_timer_t _timer;
    ledc_channel_t _channel;
    int _frequency;
    int _resolution;

    void invalidate() {
        _pin = GPIO_NUM_NC;
    }

public:
    PwmReservation() = default;

    PwmReservation(PwmManager<Feature>* manager, gpio_num_t pin, ledc_timer_t timer, ledc_channel_t channel, int frequency, int resolution):
        _manager(manager), _pin(pin), _timer(timer), _channel(channel), _frequency(frequency), _resolution(resolution) {}

    PwmReservation(const PwmReservation&) = delete;
    PwmReservation& operator=(const PwmReservation&) = delete;

    PwmReservation(PwmReservation&& other) noexcept {
        _manager = other._manager;
        _pin = other._pin;
        _timer = other._timer;
        _channel = other._channel;
        _frequency = other._frequency;
        _resolution = other._resolution;
        other.invalidate();
    }

    PwmReservation& operator=(PwmReservation&& other) noexcept {
        if (this != &other) {
            close();
            _manager = other._manager;
            _pin = other._pin;
            _timer = other._timer;
            _channel = other._channel;
            _frequency = other._frequency;
            _resolution = other._resolution;
            other.invalidate();
        }
        return *this;
    }

    ~PwmReservation() {
        close();
    }

    void ensureOpen() const {
        if (_pin == GPIO_NUM_NC) {
            throw std::runtime_error("PWM is closed");
        }
    }

    void setDuty(int duty) {
        ensureOpen();
        PwmManager<Feature>::validateDuty(duty, _resolution);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, _channel, static_cast<uint32_t>(duty));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, _channel);
    }

    void setFrequency(int frequency) {
        static_assert(Type == PwmType::Flexible);
        ensureOpen();
        PwmManager<Feature>::validateFrequency(frequency);

        auto err = ledc_set_freq(LEDC_LOW_SPEED_MODE, _timer, static_cast<uint32_t>(frequency));
        if (err != ESP_OK) {
            throw std::runtime_error(esp_err_to_name(err));
        }

        _manager->_timers.at(_timer).frequency = frequency;
        _frequency = frequency;
    }

    void setResolution(int resolution) {
        static_assert(Type == PwmType::Flexible);
        ensureOpen();
        PwmManager<Feature>::validateResolution(resolution);

        uint32_t oldDuty = ledc_get_duty(LEDC_LOW_SPEED_MODE, _channel);
        uint32_t oldMaxDuty = (1U << _resolution) - 1U;
        uint32_t newMaxDuty = (1U << resolution) - 1U;
        uint32_t newDuty = oldMaxDuty == 0 ? 0 : (oldDuty * newMaxDuty + oldMaxDuty / 2U) / oldMaxDuty;

        _manager->configureTimer(_timer, _frequency, resolution);
        _manager->_timers.at(_timer).resolution = resolution;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, _channel, newDuty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, _channel);
        _resolution = resolution;
    }

    void close() noexcept {
        if (_pin == GPIO_NUM_NC) {
            return;
        }

        _manager->closeReservation(*this);
        invalidate();
    }

    int pin() const {
        ensureOpen();
        return static_cast<int>(_pin);
    }

    ledc_timer_t timer() const {
        ensureOpen();
        return _timer;
    }

    ledc_channel_t channel() const {
        ensureOpen();
        return _channel;
    }

    int frequency() const {
        ensureOpen();
        return _frequency;
    }

    int resolution() const {
        ensureOpen();
        return _resolution;
    }
};

template<class Feature>
class PwmManager {
    template<class, PwmType>
    friend class PwmReservation;

    std::unordered_map<ledc_timer_t, PwmTimerState> _timers;
    std::unordered_map<ledc_channel_t, PwmChannelState> _channels;
    std::map<std::pair<int, int>, ledc_timer_t> _sharedTimers;

    static void validateResolution(int resolution) {
        if (resolution < 1 || resolution >= LEDC_TIMER_BIT_MAX) {
            throw std::runtime_error("Resolution must be between 1 and " + std::to_string(LEDC_TIMER_BIT_MAX - 1));
        }
    }

    static void validateFrequency(int frequency) {
        if (frequency < 1) {
            throw std::runtime_error("Frequency must be greater than 0");
        }
    }

    static void validateDuty(int duty, int resolution) {
        int maxDuty = (1 << resolution) - 1;
        if (duty < 0 || duty > maxDuty) {
            throw std::runtime_error("Duty must be between 0 and " + std::to_string(maxDuty));
        }
    }

    ledc_timer_t findFreeTimer() const {
        for (int timer = 0; timer < LEDC_TIMER_MAX; ++timer) {
            auto timerId = static_cast<ledc_timer_t>(timer);
            if (_timers.find(timerId) == _timers.end()) {
                return timerId;
            }
        }
        throw std::runtime_error("No free PWM timers available");
    }

    ledc_channel_t findFreeChannel() const {
        for (int channel = 0; channel < LEDC_CHANNEL_MAX; ++channel) {
            auto channelId = static_cast<ledc_channel_t>(channel);
            if (_channels.find(channelId) == _channels.end()) {
                return channelId;
            }
        }
        throw std::runtime_error("No free PWM channels available");
    }

    void configureTimer(ledc_timer_t timerNum, int frequency, int resolution) {
        ledc_timer_config_t ledcTimer = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = static_cast<ledc_timer_bit_t>(resolution),
            .timer_num = timerNum,
            .freq_hz = static_cast<uint32_t>(frequency),
            .clk_cfg = LEDC_AUTO_CLK,
            .deconfigure = false,
        };

        auto err = ledc_timer_config(&ledcTimer);
        if (err != ESP_OK) {
            throw std::runtime_error(esp_err_to_name(err));
        };
    }

    void configureChannel(ledc_channel_t channelNum, gpio_num_t gpioNum, ledc_timer_t timerNum, int duty, int resolution) {
        validateDuty(duty, resolution);
        ledc_channel_config_t ledcChannel = {
            .gpio_num = gpioNum,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = channelNum,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = timerNum,
            .duty = static_cast<uint32_t>(duty),
            .hpoint = 0,
            .flags = { 0 },
        };

        auto err = ledc_channel_config(&ledcChannel);
        if (err != ESP_OK) {
            throw std::runtime_error(esp_err_to_name(err));
        };
    }

    ledc_timer_t reserveFixedTimer(int frequency, int resolution) {
        std::pair<int, int> key{ frequency, resolution };
        auto sharedIt = _sharedTimers.find(key);
        if (sharedIt != _sharedTimers.end()) {
            auto timerIt = _timers.find(sharedIt->second);
            if (timerIt == _timers.end()) {
                _sharedTimers.erase(sharedIt);
            }
            else {
                timerIt->second.refCount += 1;
                return sharedIt->second;
            }
        }

        ledc_timer_t timerNum = findFreeTimer();
        configureTimer(timerNum, frequency, resolution);
        _timers.emplace(timerNum, PwmTimerState{
            .exclusive = false,
            .frequency = frequency,
            .resolution = resolution,
            .refCount = 1,
        });
        _sharedTimers.emplace(key, timerNum);
        return timerNum;
    }

    ledc_timer_t reserveFlexibleTimer(int frequency, int resolution) {
        ledc_timer_t timerNum = findFreeTimer();
        configureTimer(timerNum, frequency, resolution);
        _timers.emplace(timerNum, PwmTimerState{
            .exclusive = true,
            .frequency = frequency,
            .resolution = resolution,
            .refCount = 1,
        });
        return timerNum;
    }

    ledc_channel_t reserveChannel(gpio_num_t pin, ledc_timer_t timerNum, int duty) {
        ledc_channel_t channelNum = findFreeChannel();
        const auto timerIt = _timers.find(timerNum);
        if (timerIt == _timers.end()) {
            throw std::runtime_error("Timer not in use");
        }

        configureChannel(channelNum, pin, timerNum, duty, timerIt->second.resolution);
        _channels.emplace(channelNum, PwmChannelState{
            .timer = timerNum,
            .pin = pin,
        });
        return channelNum;
    }

    void releaseTimer(ledc_timer_t timerNum) {
        auto timerIt = _timers.find(timerNum);
        if (timerIt == _timers.end()) {
            return;
        }

        ledc_timer_rst(LEDC_LOW_SPEED_MODE, timerNum);
        if (!timerIt->second.exclusive) {
            _sharedTimers.erase(std::pair<int, int>{
                timerIt->second.frequency,
                timerIt->second.resolution,
            });
        }
        _timers.erase(timerIt);
    }

    void releaseTimerReference(ledc_timer_t timerNum) {
        auto timerIt = _timers.find(timerNum);
        if (timerIt == _timers.end()) {
            return;
        }

        timerIt->second.refCount -= 1;
        if (timerIt->second.refCount == 0) {
            releaseTimer(timerNum);
        }
    }

    void closeChannel(ledc_channel_t channelNum) {
        auto channelIt = _channels.find(channelNum);
        if (channelIt == _channels.end()) {
            return;
        }

        ledc_stop(LEDC_LOW_SPEED_MODE, channelNum, 0);

        ledc_timer_t timerNum = channelIt->second.timer;
        _channels.erase(channelIt);
        releaseTimerReference(timerNum);
    }

public:
    template<PwmType Type>
    PwmReservation<Feature, Type> reserve(int pin, int frequency, int resolution, int initialDuty) {
        validateFrequency(frequency);
        validateResolution(resolution);
        validateDuty(initialDuty, resolution);
        gpio_num_t gpioPin = Feature::getDigitalPin(pin);

        ledc_timer_t timerNum = LEDC_TIMER_0;
        if constexpr (Type == PwmType::Fixed) {
            timerNum = reserveFixedTimer(frequency, resolution);
        }
        else {
            timerNum = reserveFlexibleTimer(frequency, resolution);
        }

        ledc_channel_t channelNum = LEDC_CHANNEL_0;
        try {
            channelNum = reserveChannel(gpioPin, timerNum, initialDuty);
        }
        catch (...) {
            releaseTimerReference(timerNum);
            throw;
        }
        return PwmReservation<Feature, Type>(this, gpioPin, timerNum, channelNum, frequency, resolution);
    }

    template<PwmType Type>
    void closeReservation(PwmReservation<Feature, Type>& reservation) noexcept {
        closeChannel(reservation._channel);
    }

    ~PwmManager() {
        while (!_channels.empty()) {
            closeChannel(_channels.begin()->first);
        }

        while (!_timers.empty()) {
            releaseTimer(_timers.begin()->first);
        }
    }
};

template<class Feature, PwmType Type>
class PwmProtoBuilder : public jac::ProtoBuilder::Opaque<typename Feature::template PwmReservation<Type>>,
                            public jac::ProtoBuilder::Properties,
                            public jac::ProtoBuilder::LifetimeHandles {
    using Reservation = typename Feature::template PwmReservation<Type>;

public:
    template<class ValueType, auto Getter>
    static void defineGetter(jac::ContextRef ctx, jac::Object proto, std::string name) {
        using GetterRaw = JSValue(*)(JSContext*, JSValueConst);

        GetterRaw get = [](JSContext* ctx_, JSValueConst thisVal) -> JSValue {
            Reservation* ptr = PwmProtoBuilder::getOpaque(ctx_, jac::ValueWeak(ctx_, thisVal));
            return jac::Value::from(ctx_, static_cast<ValueType>((ptr->*Getter)())).loot().second;
        };

        JSValue getter = JS_NewCFunction2(ctx, reinterpret_cast<JSCFunction*>(reinterpret_cast<void*>(get)), ("get " + name).c_str(), 0, JS_CFUNC_getter, 0); // NOLINT
        jac::Atom atom(ctx, JS_NewAtom(ctx, name.c_str()));
        JS_DefinePropertyGetSet(ctx, proto.getVal(), atom.get(), getter, JS_UNDEFINED, static_cast<int>(jac::PropFlags::Enumerable));
    }

public:
    static Reservation* constructOpaque(jac::ContextRef ctx, std::vector<jac::ValueWeak> args) {
        if (args.size() < 1) {
            throw jac::Exception::create(jac::Exception::Type::TypeError, "Invalid number of arguments");
        }

        jac::ObjectWeak options = args[0].to<jac::ObjectWeak>();
        if (!options.hasProperty("pin")) {
            throw jac::Exception::create(jac::Exception::Type::TypeError, "Missing required property 'pin'");
        }
        if (!options.hasProperty("frequency")) {
            throw jac::Exception::create(jac::Exception::Type::TypeError, "Missing required property 'frequency'");
        }

        int pin = options.get<int>("pin");
        int frequency = options.get<int>("frequency");
        int resolution = options.hasProperty("resolution") ? options.get<int>("resolution") : 10;
        int duty = options.hasProperty("duty") ? options.get<int>("duty") : 0;

        auto* feature = static_cast<Feature*>(JS_GetContextOpaque(ctx));
        if constexpr (Type == PwmType::Fixed) {
            return new Reservation(feature->reserveFixedChannel(pin, frequency, resolution, duty));
        }
        else {
            return new Reservation(feature->reserveFlexibleChannel(pin, frequency, resolution, duty));
        }
    }

    static void addProperties(jac::ContextRef ctx, jac::Object proto) {
        jac::FunctionFactory ff(ctx);

        defineGetter<int, &Reservation::pin>(ctx, proto, "pin");
        defineGetter<int, &Reservation::timer>(ctx, proto, "timer");
        defineGetter<int, &Reservation::channel>(ctx, proto, "channel");
        defineGetter<int, &Reservation::frequency>(ctx, proto, "frequency");
        defineGetter<int, &Reservation::resolution>(ctx, proto, "resolution");

        proto.defineProperty("setDuty", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, int duty) {
            PwmProtoBuilder::getOpaque(ctx, thisVal)->setDuty(duty);
        }), jac::PropFlags::Enumerable);

        if constexpr (Type == PwmType::Flexible) {
            proto.defineProperty("setFrequency", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, int frequency) {
                PwmProtoBuilder::getOpaque(ctx, thisVal)->setFrequency(frequency);
            }), jac::PropFlags::Enumerable);

            proto.defineProperty("setResolution", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, int resolution) {
                PwmProtoBuilder::getOpaque(ctx, thisVal)->setResolution(resolution);
            }), jac::PropFlags::Enumerable);
        }

        proto.defineProperty("close", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal) {
            Reservation& reservation = *PwmProtoBuilder::getOpaque(ctx, thisVal);
            reservation.close();
            static_cast<Feature*>(JS_GetContextOpaque(ctx))->releaseLifetime(thisVal);
        }), jac::PropFlags::Enumerable);
    }

    static void postConstruction(jac::ContextRef ctx, jac::Object thisVal, std::vector<jac::ValueWeak> args) {
        static_cast<Feature*>(JS_GetContextOpaque(ctx))->extendLifetime(thisVal);
    }
};


template<class Next>
class PwmFeature : public Next {
public:
    using PwmManagerType = PwmManager<Next>;
    template<PwmType Type>
    using PwmReservation = ::PwmReservation<Next, Type>;

    using FixedPwmClass = jac::Class<PwmProtoBuilder<PwmFeature, PwmType::Fixed>>;
    using FlexiblePwmClass = jac::Class<PwmProtoBuilder<PwmFeature, PwmType::Flexible>>;

    PwmManagerType _pwm;

    PwmFeature() {
        FixedPwmClass::init("PWM");
        FlexiblePwmClass::init("VariablePWM");
    }

    PwmReservation<PwmType::Fixed> reserveFixedChannel(int pin, int frequency, int resolution, int initialDuty) {
        return _pwm.template reserve<PwmType::Fixed>(pin, frequency, resolution, initialDuty);
    }

    PwmReservation<PwmType::Flexible> reserveFlexibleChannel(int pin, int frequency, int resolution, int initialDuty) {
        return _pwm.template reserve<PwmType::Flexible>(pin, frequency, resolution, initialDuty);
    }

    void initialize() {
        Next::initialize();

        auto& mod = this->newModule("pwm");
        mod.addExport("PWM", FixedPwmClass::getConstructor(this->context()));
        mod.addExport("VariablePWM", FlexiblePwmClass::getConstructor(this->context()));
    }
};
