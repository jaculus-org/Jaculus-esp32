#pragma once

#include <memory>
#include <esp_log.h>
#include <map>

#include <jac/machine/machine.h>
#include <jac/machine/functionFactory.h>
#include <noal_func.h>

#include <gridui.h>

#include "widgets/_common.h"

class GridUiHolder {
    TaskHandle_t _webServerTask;
    std::unique_ptr<rb::Protocol> _protocol;
    std::string _ownerName;
    std::string _deviceName;

    void createClasses(jac::ContextRef context);

public:
    GridUiHolder() : _webServerTask(nullptr) { }

    void begin(jac::ContextRef context, std::string ownerName, std::string deviceName, jac::Function builderCallback);
    void end(jac::ContextRef context);

    void log(const std::string& msg) {
        if(_protocol) {
            _protocol->send_log(msg);
        }
    }
};

template<class Next>
class GridUiFeature : public Next {
    GridUiHolder _holder;

public:
    GridUiFeature() {
    }

    void initialize() {
        Next::initialize();

        using namespace gridui;
        using namespace std::placeholders;

        gridui_jac::GridUiContext::get().setScheduleEvent([this](std::function<void()> event) {
            this->scheduleEvent(event);
        });

        this->newModule("gridui", [this](jac::Module& griduiModule) {
            jac::FunctionFactory ff(this->context());

            griduiModule.addExport("begin", ff.newFunction(noal::function([this](std::string ownerName, std::string deviceName, jac::Function builderCallback){
                _holder.begin(this->context(), ownerName, deviceName, builderCallback);
            })));
            griduiModule.addExport("end", ff.newFunction(noal::function([this]() {
                _holder.end(this->context());
            })));
            griduiModule.addExport("changeTab", ff.newFunction(noal::function([](int tab){
                UI.changeTab(tab);
            })));
            griduiModule.addExport("log", ff.newFunction(noal::function([this](std::string message){
                _holder.log(message);
            })));
            griduiModule.addExport("version", ff.newFunction(noal::function([](){
                return RB_GRIDUI_VERSION;
            })));
        });
    }

    ~GridUiFeature() {
        _holder.end(this->context());
        gridui_jac::GridUiContext::get().setScheduleEvent(nullptr);
    }
};
