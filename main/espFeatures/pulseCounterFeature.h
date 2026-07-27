#pragma once

#include <jac/machine/class.h>
#include <jac/machine/functionFactory.h>
#include <jac/machine/machine.h>
#include <jac/machine/values.h>

#include "driver/pulse_cnt.h"


struct EdgeMode {
    pcnt_channel_edge_action_t pos;
    pcnt_channel_edge_action_t neg;
};

struct LevelMode {
    pcnt_channel_level_action_t high;
    pcnt_channel_level_action_t low;
};


template<>
struct jac::ConvTraits<EdgeMode> {
    static Value to(ContextRef ctx, EdgeMode val) {
        jac::Object obj = jac::Object::create(ctx);
        obj.set<int>("pos", val.pos);
        obj.set<int>("low", val.neg);

        return obj;
    }

    static EdgeMode from(ContextRef ctx, ValueWeak val) {
        try {
            auto obj = val.to<jac::Object>();
            int pos = obj.get("pos").to<int>();
            int neg = obj.get("neg").to<int>();
            if (neg < 0 || neg > 3 || pos < 0 || pos > 3) {
                throw jac::Exception::create(jac::Exception::Type::TypeError, "Invalid EdgeMode");
            }
            return {
                .pos = static_cast<pcnt_channel_edge_action_t>(pos),
                .neg = static_cast<pcnt_channel_edge_action_t>(neg)
            };
        }
        catch (jac::Exception& e) {
            throw jac::Exception::create(jac::Exception::Type::TypeError, "EdgeMode must be an object");
        }
    }
};

template<>
struct jac::ConvTraits<LevelMode> {
    static Value to(ContextRef ctx, LevelMode val) {
        jac::Object obj = jac::Object::create(ctx);
        obj.set<int>("high", val.high);
        obj.set<int>("low", val.low);

        return obj;
    }

    static LevelMode from(ContextRef ctx, ValueWeak val) {
        try {
            auto obj = val.to<jac::Object>();
            int high = obj.get("high").to<int>();
            int low = obj.get("low").to<int>();
            if (high < 0 || high > 3 || low < 0 || low > 3) {
                throw jac::Exception::create(jac::Exception::Type::TypeError, "Invalid LevelMode");
            }
            return {
                .high = static_cast<pcnt_channel_level_action_t>(high),
                .low = static_cast<pcnt_channel_level_action_t>(low)
            };
        }
        catch (jac::Exception& e) {
            throw jac::Exception::create(jac::Exception::Type::TypeError, "LevelMode must be an object");
        }
    }
};


template<typename Feature>
class PulseCounter {
public:
    Feature* _feature;
private:
    pcnt_unit_handle_t _unit = nullptr;
    pcnt_channel_handle_t _channel = nullptr;
public:
    PulseCounter(Feature* feature, int pinLevel, int pinEdge, EdgeMode edgeMode, LevelMode levelMode):
        _feature(feature)
    {
        pcnt_unit_config_t unitConfig = {
            .low_limit = std::numeric_limits<int16_t>::min(),
            .high_limit = std::numeric_limits<int16_t>::max(),
            .intr_priority = 0,
            .flags = { 0 }
        };
        unitConfig.flags.accum_count = 0;

        auto res = pcnt_new_unit(&unitConfig, &_unit);
        if (res != ESP_OK) {
            throw jac::Exception::create(jac::Exception::Type::InternalError, std::string("Failed to configure pulse counter unit (") + esp_err_to_name(res) + ")");
        }

        pcnt_chan_config_t channelConfig = {
            .edge_gpio_num = Feature::getDigitalPin(pinEdge), // TODO: rename
            .level_gpio_num = Feature::getDigitalPin(pinLevel),
            .flags = { }
        };
        channelConfig.flags.invert_edge_input = 0;
        channelConfig.flags.invert_level_input = 0;
        channelConfig.flags.virt_edge_io_level = 0;
        channelConfig.flags.virt_level_io_level = 0;
        channelConfig.flags.io_loop_back = 0;

        res = pcnt_new_channel(_unit, &channelConfig, &_channel);
        if (res != ESP_OK) {
            throw jac::Exception::create(jac::Exception::Type::InternalError, std::string("Failed to configure pulse counter channel (") + esp_err_to_name(res) + ")");
        }

        res = pcnt_channel_set_edge_action(_channel, edgeMode.pos, edgeMode.neg);
        if (res != ESP_OK) {
            throw jac::Exception::create(jac::Exception::Type::InternalError, std::string("Failed to configure pulse counter channel edge action (") + esp_err_to_name(res) + ")");
        }

        res = pcnt_channel_set_level_action(_channel, levelMode.high, levelMode.low);
        if (res != ESP_OK) {
            throw jac::Exception::create(jac::Exception::Type::InternalError, std::string("Failed to configure pulse counter channel level action (") + esp_err_to_name(res) + ")");
        }

        res = pcnt_unit_enable(_unit);
        if (res != ESP_OK) {
            throw jac::Exception::create(jac::Exception::Type::InternalError, std::string("Failed to enable pulse counter unit (") + esp_err_to_name(res) + ")");
        }

        start();
    }

    int read() {
        if (_unit == nullptr || _channel == nullptr) {
            throw jac::Exception::create(jac::Exception::Type::InternalError, "Pulse counter is closed");
        }
        int count;
        auto res = pcnt_unit_get_count(_unit, &count);
        if (res != ESP_OK) {
            throw jac::Exception::create(jac::Exception::Type::InternalError, "Failed to read pulse counter unit");
        }
        return count;
    }

    void clear() {
        if (_unit == nullptr || _channel == nullptr) {
            throw jac::Exception::create(jac::Exception::Type::InternalError, "Pulse counter is closed");
        }
        auto res = pcnt_unit_clear_count(_unit);
        if (res != ESP_OK) {
            throw jac::Exception::create(jac::Exception::Type::InternalError, "Failed to clear pulse counter unit");
        }
    }

    void start() {
        if (_unit == nullptr || _channel == nullptr) {
            throw jac::Exception::create(jac::Exception::Type::InternalError, "Pulse counter is closed");
        }
        auto res = pcnt_unit_start(_unit);
        if (res != ESP_OK) {
            throw jac::Exception::create(jac::Exception::Type::InternalError, "Failed to start pulse counter unit");
        }
    }

    void stop() {
        if (_unit == nullptr || _channel == nullptr) {
            throw jac::Exception::create(jac::Exception::Type::InternalError, "Pulse counter is closed");
        }
        auto res = pcnt_unit_stop(_unit);
        if (res != ESP_OK) {
            throw jac::Exception::create(jac::Exception::Type::InternalError, "Failed to stop pulse counter unit");
        }
    }

    void close() {
        if (_channel != nullptr) {
            pcnt_del_channel(_channel);
            _channel = nullptr;
        }
        if (_unit != nullptr) {
            pcnt_unit_disable(_unit);
            pcnt_del_unit(_unit);
            _unit = nullptr;
        }
    }

    ~PulseCounter() {
        close();
    }
};


template<class Feature>
class PulseCounterProtoBuilder : public jac::ProtoBuilder::Opaque<PulseCounter<Feature>>, jac::ProtoBuilder::Properties, jac::ProtoBuilder::LifetimeHandles {
    using PulseCounter_ = PulseCounter<Feature>;
public:
    static PulseCounter_* constructOpaque(jac::ContextRef ctx, jac::ValueVectorWeak args) {
        if (args.size() < 1) {
            throw jac::Exception::create(jac::Exception::Type::TypeError, "Invalid number of arguments");
        }

        jac::ObjectWeak options = args[0].to<jac::ObjectWeak>();

        if (!options.hasProperty("pinLevel")) {
            throw jac::Exception::create(jac::Exception::Type::TypeError, "Missing required property 'pinLevel'");
        }
        int pinLevel = options.get<int>("pinLevel");

        if (!options.hasProperty("pinEdge")) {
            throw jac::Exception::create(jac::Exception::Type::TypeError, "Missing required property 'pinEdge'");
        }
        int pinEdge = options.get<int>("pinEdge");

        if (!options.hasProperty("levelMode")) {
            throw jac::Exception::create(jac::Exception::Type::TypeError, "Missing required property 'levelMode'");
        }
        LevelMode levelMode = options.get<LevelMode>("levelMode");

        if (!options.hasProperty("edgeMode")) {
            throw jac::Exception::create(jac::Exception::Type::TypeError, "Missing required property 'edgeMode'");
        }
        EdgeMode edgeMode = options.get<EdgeMode>("edgeMode");

        return new PulseCounter_(static_cast<Feature*>(JS_GetContextOpaque(ctx)), pinLevel, pinEdge, edgeMode, levelMode);
    }

    static void addProperties(jac::ContextRef ctx, jac::Object proto) {
        PulseCounterProtoBuilder::template addMethodMember<int(PulseCounter_::*)(), &PulseCounter_::read>(ctx, proto, "read");
        PulseCounterProtoBuilder::template addMethodMember<void(PulseCounter_::*)(), &PulseCounter_::clear>(ctx, proto, "clear");
        PulseCounterProtoBuilder::template addMethodMember<void(PulseCounter_::*)(), &PulseCounter_::start>(ctx, proto, "start");
        PulseCounterProtoBuilder::template addMethodMember<void(PulseCounter_::*)(), &PulseCounter_::stop>(ctx, proto, "stop");

        jac::FunctionFactory ff(ctx);

        proto.defineProperty("close", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal) {
            PulseCounter_& self = *PulseCounterProtoBuilder::getOpaque(ctx, thisVal);
            self.close();
            self._feature->releaseLifetime(thisVal);
        }), jac::PropFlags::Enumerable);
    }

    static void postConstruction(jac::ContextRef ctx, jac::Object thisVal, jac::ValueVectorWeak args) {
        PulseCounter_& self = *PulseCounterProtoBuilder::getOpaque(ctx, thisVal);
        self._feature->extendLifetime(thisVal);
    }
};


struct NonexistentPulseCounterProtoBuilder : public jac::ProtoBuilder::Properties {
    static void addProperties(jac::ContextRef ctx, jac::Object proto) {
        jac::FunctionFactory ff(ctx);

        proto.defineProperty("read", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal) {
            throw jac::Exception::create(jac::Exception::Type::InternalError, "Pulse counter not available on this device");
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("clear", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal) {
            throw jac::Exception::create(jac::Exception::Type::InternalError, "Pulse counter not available on this device");
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("start", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal) {
            throw jac::Exception::create(jac::Exception::Type::InternalError, "Pulse counter not available on this device");
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("stop", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal) {
            throw jac::Exception::create(jac::Exception::Type::InternalError, "Pulse counter not available on this device");
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("close", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal) {
            throw jac::Exception::create(jac::Exception::Type::InternalError, "Pulse counter not available on this device");
        }), jac::PropFlags::Enumerable);
    }

    static void postConstruction(jac::ContextRef ctx, jac::Object thisVal, jac::ValueVectorWeak args) {
        throw jac::Exception::create(jac::Exception::Type::InternalError, "Pulse counter not available on this device");
    }
};


template<class Next>
class PulseCounterFeature : public Next {
public:
#if defined(CONFIG_IDF_TARGET_ESP32C3)
    using PulseCounterClass = jac::Class<NonexistentPulseCounterProtoBuilder>;
#else
    using PulseCounterClass = jac::Class<PulseCounterProtoBuilder<PulseCounterFeature>>;
#endif

    PulseCounterFeature() {
        PulseCounterClass::init("PulseCounter");
    }

    void initialize() {
        Next::initialize();

        this->newModule("pulseCounter", [this](jac::Module& mod) {
            jac::Object pcntCtor = PulseCounterClass::getConstructor(this->context());

            jac::Object edgeAction = jac::Object::create(this->context());
            edgeAction.defineProperty("Hold", jac::Value::from(this->context(), static_cast<int>(pcnt_channel_edge_action_t::PCNT_CHANNEL_EDGE_ACTION_HOLD)), jac::PropFlags::Enumerable);
            edgeAction.defineProperty("Increase", jac::Value::from(this->context(), static_cast<int>(pcnt_channel_edge_action_t::PCNT_CHANNEL_EDGE_ACTION_INCREASE)), jac::PropFlags::Enumerable);
            edgeAction.defineProperty("Decrease", jac::Value::from(this->context(), static_cast<int>(pcnt_channel_edge_action_t::PCNT_CHANNEL_EDGE_ACTION_DECREASE)), jac::PropFlags::Enumerable);

            jac::Object levelAction = jac::Object::create(this->context());
            levelAction.defineProperty("Keep", jac::Value::from(this->context(), static_cast<int>(pcnt_channel_level_action_t::PCNT_CHANNEL_LEVEL_ACTION_KEEP)), jac::PropFlags::Enumerable);
            levelAction.defineProperty("Inverse", jac::Value::from(this->context(), static_cast<int>(pcnt_channel_level_action_t::PCNT_CHANNEL_LEVEL_ACTION_INVERSE)), jac::PropFlags::Enumerable);
            levelAction.defineProperty("Hold", jac::Value::from(this->context(), static_cast<int>(pcnt_channel_level_action_t::PCNT_CHANNEL_LEVEL_ACTION_HOLD)), jac::PropFlags::Enumerable);

            mod.addExport("EdgeAction", edgeAction);
            mod.addExport("LevelAction", levelAction);

            mod.addExport("PulseCounter", pcntCtor);
        });
    }
};
