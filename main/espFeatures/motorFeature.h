#pragma once

#include <jac/features/types/streams.h>
#include <jac/machine/class.h>
#include <jac/machine/machine.h>
#include <jac/machine/values.h>

#include <dcmotor.h>

#include <format>
#include <optional>
#include <vector>


template<typename Feature>
struct MotorPins {
    gpio_num_t motA;
    gpio_num_t motB;
    gpio_num_t encA;
    gpio_num_t encB;
};

template<typename Feature>
struct LedcConfig {
    ledc_timer_t timer;
    ledc_channel_t channelA;
    ledc_channel_t channelB;
};

template<typename Feature>
struct jac::ConvTraits<MotorPins<Feature>> {
    static Value to(ContextRef ctx, MotorPins<Feature> conf) {
        auto obj = jac::Object::create(ctx);
        obj.set("motA", jac::Value::from(ctx, static_cast<int>(conf.motA)));
        obj.set("motB", jac::Value::from(ctx, static_cast<int>(conf.motB)));
        obj.set("encA", jac::Value::from(ctx, static_cast<int>(conf.encA)));
        obj.set("encB", jac::Value::from(ctx, static_cast<int>(conf.encB)));
        return obj;
    }

    static MotorPins<Feature> from(ContextRef ctx, ValueWeak val) {
        auto obj = val.to<jac::ObjectWeak>();
        MotorPins<Feature> conf;
        conf.motA = Feature::getDigitalPin(obj.get<int>("motA"));  // TODO: use direct conversion
        conf.motB = Feature::getDigitalPin(obj.get<int>("motB"));
        conf.encA = Feature::getDigitalPin(obj.get<int>("encA"));
        conf.encB = Feature::getDigitalPin(obj.get<int>("encB"));
        return conf;
    }
};

template<typename Feature>
struct jac::ConvTraits<LedcConfig<Feature>> {
    static Value to(ContextRef ctx, LedcConfig<Feature> conf) {
        auto obj = jac::Object::create(ctx);
        obj.set("timer", jac::Value::from(ctx, static_cast<int>(conf.timer)));
        obj.set("channelA", jac::Value::from(ctx, static_cast<int>(conf.channelA)));
        obj.set("channelB", jac::Value::from(ctx, static_cast<int>(conf.channelB)));
        return obj;
    }

    static LedcConfig<Feature> from(ContextRef ctx, ValueWeak val) {
        auto obj = val.to<jac::ObjectWeak>();
        LedcConfig<Feature> conf;
        conf.timer = static_cast<ledc_timer_t>(obj.get<int>("timer"));
        conf.channelA = static_cast<ledc_channel_t>(obj.get<int>("channelA"));
        conf.channelB = static_cast<ledc_channel_t>(obj.get<int>("channelB"));

        if (conf.timer >= LEDC_TIMER_MAX) {
            throw std::runtime_error("Invalid timer");
        }
        if (conf.channelA >= LEDC_CHANNEL_MAX || conf.channelB >= LEDC_CHANNEL_MAX) {
            throw std::runtime_error("Invalid channel");
        }

        return conf;
    }
};

template<>
struct jac::ConvTraits<RegParams> {
    static Value to(ContextRef ctx, RegParams reg) {
        auto obj = jac::Object::create(ctx);
        obj.set("kp", jac::Value::from(ctx, reg.kp));
        obj.set("ki", jac::Value::from(ctx, reg.ki));
        obj.set("kd", jac::Value::from(ctx, reg.kd));
        obj.set("kv", jac::Value::from(ctx, reg.kv));
        obj.set("ka", jac::Value::from(ctx, reg.ka));
        obj.set("kc", jac::Value::from(ctx, reg.kc));
        obj.set("maxIOut", jac::Value::from(ctx, reg.maxIOut));
        obj.set("unwindFactor", jac::Value::from(ctx, reg.unwindFactor));
        return obj;
    }

    static RegParams from(ContextRef ctx, ValueWeak val) {
        auto obj = val.to<jac::ObjectWeak>();
        RegParams reg = {
            .kp = 0,
            .ki = 0,
            .kd = 0,
            .kv = 0,
            .ka = 0,
            .kc = 0,
            .maxIOut = 0,
            .unwindFactor = 0
        };
        if (obj.hasProperty("kp")) {
            reg.kp = obj.get<int>("kp");
        }
        if (obj.hasProperty("ki")) {
            reg.ki = obj.get<int>("ki");
        }
        if (obj.hasProperty("kd")) {
            reg.kd = obj.get<int>("kd");
        }
        if (obj.hasProperty("kv")) {
            reg.kv = obj.get<int>("kv");
        }
        if (obj.hasProperty("ka")) {
            reg.ka = obj.get<int>("ka");
        }
        if (obj.hasProperty("kc")) {
            reg.kc = obj.get<int>("kc");
        }
        if (obj.hasProperty("maxIOut")) {
            reg.maxIOut = obj.get<int>("maxIOut");
        }
        if (obj.hasProperty("unwindFactor")) {
            reg.unwindFactor = obj.get<int>("unwindFactor");
        }

        return reg;
    }
};

struct PromiseFunctions {
    jac::Function resolve;
    jac::Function reject;
};


struct JSDCMotor {
    std::optional<DCMotor> motor;
    std::optional<PromiseFunctions> pendingPromise;
    int encTicks;
    double circumference;

    using Report = std::tuple<uint64_t, int64_t, int64_t, int, int, int, int, int, int, int, int>;
    std::deque<Report> reports;
    int reportCounter = 0;

    template<typename Feature>
    JSDCMotor(MotorPins<Feature> pins, LedcConfig<Feature> ledcConf, RegParams reg, int encTicks, double circumference):
        motor(std::in_place, pins.motA, pins.motB, pins.encA, pins.encB, reg, ledcConf.timer, ledcConf.channelA, ledcConf.channelB),
        encTicks(encTicks), circumference(circumference)
    {
        motor->startTicker();
    }

    void reportStart(int everyNth) {
        if (motor->isReporting()) {
            throw std::runtime_error("Already logging");
        }
        if (everyNth <= 0) {
            throw std::runtime_error("Invalid argument");
        }

        reportCounter = 1;
        motor->setReporter([this, everyNth](uint64_t time, int64_t pos, int64_t targetPos, int speed, int error, int power, int pout, int iout, int dout, int vout, int aout) {
            if (reportCounter < 0 || (--reportCounter) != 0) {
                return;
            }
            reportCounter = everyNth;
            try {
                reports.emplace_back(time, pos, targetPos, speed, error, power, pout, iout, dout, vout, aout);
            }
            catch (...) {
                reports.clear();
                this->reportStop();
                reportCounter = -1;
            }
        });
    }

    void reportStop() {
        motor->setReporter(nullptr);
    }

    void reportClear() {
        reports.clear();
    }

    void reportDump(jac::Writable& out) {
        if (reportCounter < 0) {
            throw std::runtime_error("Report failed (out of memory)");
        }

        out.write("time,pos,targetPos,speed,error,power,pout,iout,dout,vout,aout\n");
        for (const auto& report : reports) {
            auto row = std::format("{},{},{},{},{},{},{},{},{},{},{}\n",
                std::get<0>(report), std::get<1>(report), std::get<2>(report), std::get<3>(report), std::get<4>(report),
                std::get<5>(report), std::get<6>(report), std::get<7>(report), std::get<8>(report), std::get<9>(report), std::get<10>(report));
            out.write(row);
        }
    }

    ~JSDCMotor() {
        clear();
    }

    void clear() {
        if (motor) {
            motor->stopTicker();
            motor.reset();
        }
        pendingPromise.reset();
    }
};


template<typename Feature>
class MotorProtoBuilder : public jac::ProtoBuilder::Opaque<JSDCMotor>, public jac::ProtoBuilder::Properties, jac::ProtoBuilder::LifetimeHandles {
public:
    static JSDCMotor* constructOpaque(jac::ContextRef ctx, std::vector<jac::ValueWeak> args) {
        if (args.size() != 1) {
            throw std::runtime_error("Expected 2 arguments");
        }

        auto options = args[0].to<jac::Object>();
        auto pins = options.get<MotorPins<Feature>>("pins");
        auto encTicks = options.get<int>("encTicks");
        auto circumference = options.get<double>("circumference");

        auto ledcConf = options.get<LedcConfig<Feature>>("ledc");
        auto reg = options.get<RegParams>("reg");

        auto machine = static_cast<Feature*>(JS_GetContextOpaque(ctx));
        std::unique_ptr<JSDCMotor> mot = std::make_unique<JSDCMotor>(pins, ledcConf, reg, encTicks, circumference);
        mot->motor->onTarget([mot = mot.get(), machine]() {
            static constexpr auto resolve = +[](void* data) {
                JSDCMotor* mot = static_cast<JSDCMotor*>(data);
                if (mot->pendingPromise) {
                    mot->pendingPromise->resolve.call<void>();
                    mot->pendingPromise.reset();
                }
            };

            if (mot->pendingPromise) {
                machine->scheduleEventISR(resolve, mot);  // FIXME: potential out-of-memory access
            }
        });
        mot->motor->setEndPosTolerance(3);


        return mot.release();
    }

    static void addProperties(jac::ContextRef ctx, jac::Object proto) {
        jac::FunctionFactory ff(ctx);

        proto.defineProperty("setSpeed", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, double speed) {
            auto& motor = *getOpaque(ctx, thisVal);
            if (!motor.motor) {
                throw jac::Exception::create(jac::Exception::Type::InternalError, "Motor is closed");
            }

            int ticksPerSec = motor.encTicks * speed / (motor.circumference);
            motor.motor->setSpeed(ticksPerSec);
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("move", ff.newFunctionThisVariadic([](jac::ContextRef ctx, jac::ValueWeak thisVal, std::vector<jac::ValueWeak> args) {
            auto& motor = *getOpaque(ctx, thisVal);
            if (!motor.motor) {
                throw jac::Exception::create(jac::Exception::Type::InternalError, "Motor is closed");
            }

            if (motor.pendingPromise) {
                throw jac::Exception::create(jac::Exception::Type::InternalError, "Motor is already moving");
            }

            std::optional<jac::Promise> pms;

            if (args.size() == 0) {
                auto [ promise, resolve, reject ] = jac::Promise::create(ctx);
                motor.pendingPromise = PromiseFunctions{resolve, reject};
                pms = promise;

                motor.motor->moveInfinite();
            }
            else {
                jac::Object duration = args[0].to<jac::Object>();

                bool hasDistance = duration.hasProperty("distance");
                bool hasTime = duration.hasProperty("time");

                if (hasDistance && hasTime) {
                    throw jac::Exception::create(jac::Exception::Type::TypeError, "Cannot specify both distance and time");
                }

                auto [ promise, resolve, reject ] = jac::Promise::create(ctx);
                motor.pendingPromise = PromiseFunctions{resolve, reject};
                pms = promise;

                if (hasDistance) {
                    double distance = duration.get<double>("distance");
                    int64_t ticks = motor.encTicks * distance / (motor.circumference);
                    motor.motor->moveDistance(ticks);
                }
                else if (hasTime) {
                    int64_t time = duration.get<int64_t>("time");
                    motor.motor->moveTime(time);
                }
                else {
                    motor.motor->moveInfinite();
                }
            }

            return pms.value();
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("setRamp", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, int ramp) {
            auto& motor = *getOpaque(ctx, thisVal);
            if (!motor.motor) {
                throw jac::Exception::create(jac::Exception::Type::InternalError, "Motor is closed");
            }

            motor.motor->setRamp(ramp);
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("setEndPosTolerance", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, unsigned tolerance) {
            auto& motor = *getOpaque(ctx, thisVal);
            if (!motor.motor) {
                throw jac::Exception::create(jac::Exception::Type::InternalError, "Motor is closed");
            }

            motor.motor->setEndPosTolerance(tolerance);
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("reportStart", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, unsigned everyNth) {
            auto& motor = *getOpaque(ctx, thisVal);
            if (!motor.motor) {
                throw jac::Exception::create(jac::Exception::Type::InternalError, "Motor is closed");
            }

            motor.reportStart(everyNth);
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("reportStop", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal) {
            auto& motor = *getOpaque(ctx, thisVal);
            if (!motor.motor) {
                throw jac::Exception::create(jac::Exception::Type::InternalError, "Motor is closed");
            }

            motor.reportStop();
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("reportClear", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal) {
            auto& motor = *getOpaque(ctx, thisVal);
            if (!motor.motor) {
                throw jac::Exception::create(jac::Exception::Type::InternalError, "Motor is closed");
            }

            motor.reportClear();
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("reportDump", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal) {
            auto& motor = *getOpaque(ctx, thisVal);
            if (!motor.motor) {
                throw jac::Exception::create(jac::Exception::Type::InternalError, "Motor is closed");
            }

            Feature& machine = *static_cast<Feature*>(JS_GetContextOpaque(ctx));

            motor.reportDump(*(machine.stdio.out));
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("setRaw", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, int power) {
            auto& motor = *getOpaque(ctx, thisVal);
            if (!motor.motor) {
                throw jac::Exception::create(jac::Exception::Type::InternalError, "Motor is closed");
            }

            motor.motor->setRawPower(power);
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("stop", ff.newFunctionThisVariadic([](jac::ContextRef ctx, jac::ValueWeak thisVal, std::vector<jac::ValueWeak> args) {
            auto& motor = *getOpaque(ctx, thisVal);
            if (!motor.motor) {
                throw jac::Exception::create(jac::Exception::Type::InternalError, "Motor is closed");
            }

            if (args.size() == 0) {
                motor.motor->stop(false);
            }
            else {
                motor.motor->stop(args[0].to<bool>());
            }

            auto [ promise, resolve, reject ] = jac::Promise::create(ctx);
            resolve.call<void>();
            return promise;
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("getPosition", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal) {
            auto& motor = *getOpaque(ctx, thisVal);
            if (!motor.motor) {
                throw jac::Exception::create(jac::Exception::Type::InternalError, "Motor is closed");
            }

            int64_t ticks = motor.motor->getPosition();
            return jac::Value::from(ctx, static_cast<double>(ticks) * motor.circumference / motor.encTicks);
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("close", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal) {
            auto& motor = *getOpaque(ctx, thisVal);
            if (!motor.motor) {
                throw jac::Exception::create(jac::Exception::Type::InternalError, "Motor is closed");
            }

            motor.clear();

            auto machine = static_cast<Feature*>(JS_GetContextOpaque(ctx));
            machine->unregisterMotor(thisVal);
        }), jac::PropFlags::Enumerable);
    }

    static void postConstruction(jac::ContextRef ctx, jac::Object thisVal, std::vector<jac::ValueWeak> args) {
        auto machine = static_cast<Feature*>(JS_GetContextOpaque(ctx));
        machine->registerMotor(thisVal);
    }
};


struct NonexistentMotorProtoBuilder {
    static void postConstruction(jac::ContextRef ctx, jac::Object thisVal, std::vector<jac::ValueWeak> args) {
        throw jac::Exception::create(jac::Exception::Type::InternalError, "Motor driver is not available on this device");
    }
};


template<typename Next>
class MotorFeature : public Next {
    std::vector<jac::Object> _motors;
public:

#if defined(CONFIG_IDF_TARGET_ESP32C3)
    using MotorClass = jac::Class<NonexistentMotorProtoBuilder>;
#else
    using MotorClass = jac::Class<MotorProtoBuilder<MotorFeature<Next>>>;
#endif


    MotorFeature() {
        MotorClass::init("Motor");
    }

#if !defined(CONFIG_IDF_TARGET_ESP32C3)
    ~MotorFeature() {
        for(auto& motor : _motors) {
            auto& jsdcMotor = *MotorProtoBuilder<MotorFeature<Next>>::getOpaque(this->context(), motor);
            jsdcMotor.clear();
        }
    }

    void registerMotor(jac::Object motor) {
        _motors.emplace_back(motor);
    }

    void unregisterMotor(jac::ValueWeak motor) {
        for(auto itr = _motors.begin(); itr != _motors.end();) {
            if(itr->getVal() == motor.getVal()) {
                _motors.erase(itr);
                return;
            }
            ++itr;
        }
    }
#endif

    void initialize() {
        Next::initialize();

        auto& mod = this->newModule("motor");

        jac::Object motorCtor = MotorClass::getConstructor(this->context());

        mod.addExport("Motor", motorCtor);
    }
};
