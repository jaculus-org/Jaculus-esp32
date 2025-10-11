#pragma once

#include <jac/machine/class.h>
#include <jac/machine/functionFactory.h>
#include <jac/machine/machine.h>

#include <SmartLeds.h>
#include <memory>
#include <noal_func.h>
#include <unordered_map>

#include <set>


template<>
struct jac::ConvTraits<Rgb> {
    static Value to(ContextRef ctx, Rgb val) {
        auto obj = Object::create(ctx);
        obj.set<int>("r", val.r);
        obj.set<int>("g", val.g);
        obj.set<int>("b", val.b);
        return obj;
    }

    static Rgb from(ContextRef ctx, ValueWeak val) {
        auto obj = val.to<Object>();
        return Rgb(obj.get<int>("r"), obj.get<int>("g"), obj.get<int>("b"));
    }
};


template<>
struct jac::ConvTraits<LedType> {
    static Value to(ContextRef ctx, LedType val) {
        auto obj = Object::create(ctx);
        obj.set<int>("T0H", val.T0H);
        obj.set<int>("T1H", val.T1H);
        obj.set<int>("T0L", val.T0L);
        obj.set<int>("T1L", val.T1L);
        obj.set<int>("TRS", val.TRS);
        return obj;
    }

    static LedType from(ContextRef ctx, ValueWeak val) {
        auto obj = val.to<Object>();
        return LedType(obj.get<int>("T0H"), obj.get<int>("T1H"), obj.get<int>("T0L"), obj.get<int>("T1L"), obj.get<int>("TRS"));
    }
};

template<class Next>
class SmartLedFeature : public Next {
    static inline std::set<int> _usedRmtChannels;

    struct SmartLedProtoBuilder : public jac::ProtoBuilder::Opaque<SmartLed>, public jac::ProtoBuilder::Properties {
        static SmartLed* constructOpaque(JSContext* ctx, std::vector<jac::ValueWeak> args) {
            if (args.size() < 2) {
                throw std::runtime_error("Invalid number of arguments");
            }
            int pin = args[0].to<int>();
            int count = args[1].to<int>();

            LedType type = LED_WS2812;
            if (args.size() > 2) {
                type = args[2].to<LedType>();
            }

            if (Next::PlatformInfo::PinConfig::DIGITAL_PINS.find(pin) == Next::PlatformInfo::PinConfig::DIGITAL_PINS.end()) {
                throw std::runtime_error("Invalid pin number");
            }

            int channel = 0;
            while (_usedRmtChannels.find(channel) != _usedRmtChannels.end()) {
                channel++;
            }
            if (channel >= 4) {
                throw std::runtime_error("No available RMT channels");
            }
            _usedRmtChannels.insert(channel);

            return new SmartLed(type, count, pin, channel, SingleBuffer);
        }

        static void destroyOpaque(JSRuntime* rt, SmartLed* ptr) noexcept {
            if (!ptr) {
                return;
            }

            ptr->wait();
            int channel = ptr->channel();
            delete ptr;
            _usedRmtChannels.erase(channel);
        }

        static void addProperties(JSContext* ctx, jac::Object proto) {
            jac::FunctionFactory ff(ctx);

            proto.defineProperty("show", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisValue) {
                SmartLed& led = *getOpaque(ctx, thisValue);
                led.wait();
                led.show();
            }), jac::PropFlags::Enumerable);

            proto.defineProperty("set", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisValue, int idx, Rgb color) {
                SmartLed& strip = *getOpaque(ctx, thisValue);
                if (idx < 0 || idx >= strip.size()) {
                    throw std::runtime_error("Invalid index");
                }
                strip[idx] = color;
            }), jac::PropFlags::Enumerable);

            proto.defineProperty("get", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisValue, int idx) {
                SmartLed& strip = *getOpaque(ctx, thisValue);
                if (idx < 0 || idx >= strip.size()) {
                    throw std::runtime_error("Invalid index");
                }
                return strip[idx];
            }), jac::PropFlags::Enumerable);

            proto.defineProperty("clear", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisValue) {
                SmartLed& strip = *getOpaque(ctx, thisValue);
                for (auto& led : strip) {
                    led = Rgb(0, 0, 0);
                }
            }), jac::PropFlags::Enumerable);
        }
    };
public:
    using SmartLedClass = jac::Class<SmartLedProtoBuilder>;

    SmartLedFeature() {
        SmartLedClass::init("SmartLed");
    }

    void initialize() {
        Next::initialize();

        jac::Module& mod = this->newModule("smartled");
        jac::Function ctor = SmartLedClass::getConstructor(this->context());
        mod.addExport("SmartLed", ctor);
        mod.addExport("LED_WS2812", jac::Value::from(this->context(), LED_WS2812));
        mod.addExport("LED_WS2812B", jac::Value::from(this->context(), LED_WS2812B));
        mod.addExport("LED_WS2812B_2020", jac::Value::from(this->context(), LedType{ 400, 800, 850, 450, 300000 }));
        mod.addExport("LED_WS2813", jac::Value::from(this->context(), LED_WS2813));
        mod.addExport("LED_SK6812", jac::Value::from(this->context(), LED_SK6812));
    }
};
