#include <memory>
#include <esp_log.h>
#include <esp_netif.h>

#include <gridui.h>
#include <rbdns.h>

#include "../../platform/espWifi.h"

#include "gridUiFeature.h"

using namespace gridui_jac;



class GridUiBuilderProtoBuilder : public jac::ProtoBuilder::Opaque<GridUiHolder>, public jac::ProtoBuilder::Properties {
    friend class GridUiHolder;

    template<typename BuilderT, typename WidgetT>
    static jac::Object builder(jac::ContextRef ctx, jac::ValueWeak thisVal, jac::ValueVectorWeak args) {
        if (args.size() < 4) {
            throw jac::Exception::create(jac::Exception::Type::TypeError, "expected at least 4 arguments");
        }

        auto x = args[0].to<float>();
        auto y = args[1].to<float>();
        auto w = args[2].to<float>();
        auto h = args[3].to<float>();
        uint16_t uuid = args.size() >= 5 ? args[4].to<uint16_t>() : 0;
        uint16_t tab = args.size() >= 6 ? args[5].to<uint16_t>() : 0;

        auto widget = gridui::UI.newWidget<BuilderT>(x, y, w, h, uuid, tab);

        auto proto = jac::Value(ctx, JS_GetClassProto(ctx.get(), GridUiContext::builderClassId()));

        jac::Value obj(ctx, JS_NewObjectProtoClass(ctx.get(), proto.getVal(), GridUiContext::get().builderClassId()));
        JS_SetOpaque(obj.getVal(), widget);
        return obj;
    }

public:
    static void destroyOpaque(JSRuntime* rt, GridUiHolder* ptr) noexcept { }

    static void addProperties(JSContext* ctx, jac::Object proto) {
        jac::FunctionFactory ff(ctx);

        proto.defineProperty("arm", ff.newFunctionThisVariadic(std::function(&builder<gridui::builder::Arm, gridui::Arm>)), jac::PropFlags::Enumerable);
        proto.defineProperty("bar", ff.newFunctionThisVariadic(std::function(&builder<gridui::builder::Bar, gridui::Bar>)), jac::PropFlags::Enumerable);
        proto.defineProperty("button", ff.newFunctionThisVariadic(std::function(&builder<gridui::builder::Button, gridui::Button>)), jac::PropFlags::Enumerable);
        proto.defineProperty("camera", ff.newFunctionThisVariadic(std::function(&builder<gridui::builder::Camera, gridui::Camera>)), jac::PropFlags::Enumerable);
        proto.defineProperty("checkbox", ff.newFunctionThisVariadic(std::function(&builder<gridui::builder::Checkbox, gridui::Checkbox>)), jac::PropFlags::Enumerable);
        proto.defineProperty("circle", ff.newFunctionThisVariadic(std::function(&builder<gridui::builder::Circle, gridui::Circle>)), jac::PropFlags::Enumerable);
        proto.defineProperty("input", ff.newFunctionThisVariadic(std::function(&builder<gridui::builder::Input, gridui::Input>)), jac::PropFlags::Enumerable);
        proto.defineProperty("joystick", ff.newFunctionThisVariadic(std::function(&builder<gridui::builder::Joystick, gridui::Joystick>)), jac::PropFlags::Enumerable);
        proto.defineProperty("led", ff.newFunctionThisVariadic(std::function(&builder<gridui::builder::Led, gridui::Led>)), jac::PropFlags::Enumerable);
        proto.defineProperty("orientation", ff.newFunctionThisVariadic(std::function(&builder<gridui::builder::Orientation, gridui::Orientation>)), jac::PropFlags::Enumerable);
        proto.defineProperty("select", ff.newFunctionThisVariadic(std::function(&builder<gridui::builder::Select, gridui::Select>)), jac::PropFlags::Enumerable);
        proto.defineProperty("slider", ff.newFunctionThisVariadic(std::function(&builder<gridui::builder::Slider, gridui::Slider>)), jac::PropFlags::Enumerable);
        proto.defineProperty("spinEdit", ff.newFunctionThisVariadic(std::function(&builder<gridui::builder::SpinEdit, gridui::SpinEdit>)), jac::PropFlags::Enumerable);
        proto.defineProperty("switcher", ff.newFunctionThisVariadic(std::function(&builder<gridui::builder::Switcher, gridui::Switcher>)), jac::PropFlags::Enumerable);
        proto.defineProperty("text", ff.newFunctionThisVariadic(std::function(&builder<gridui::builder::Text, gridui::Text>)), jac::PropFlags::Enumerable);
    }
};

void GridUiHolder::begin(jac::ContextRef context, std::string ownerName, std::string deviceName, jac::Function builderCallback) {
    using namespace gridui;
    if(_webServerTask) {
        throw jac::Exception::create(jac::Exception::Type::Error, "gridui.begin was already called");
    }

    jac::Class<GridUiBuilderProtoBuilder>::init("guiBuilder");

    // Initialize the TCP/IP stack - without this, the socket() call inside rb::Protocol
    // will abort if the WiFi is disabled.
    esp_netif_init();

    rb_web_set_not_found_callback(gridui::webserverNotFoundCallback);

    // Start serving the web page
    _webServerTask = rb_web_start_no_spiffs(80, "/data");
    if(_webServerTask == NULL) {
        throw jac::Exception::create(jac::Exception::Type::InternalError, "failed to call rb_web_start_no_spiffs");
    }

    _ownerName = ownerName;
    _deviceName = deviceName;
    _protocol.reset(
        new rb::Protocol(_ownerName.c_str(), _deviceName.c_str(), "Jaculus",
            [](const std::string& cmd, rbjson::Object* pkt) {
                UI.handleRbPacket(cmd, pkt);
            }
        )
    );
    _protocol->start();

    // Disabled due to memory constraints
    /*if(EspWifiController::get().mode() == EspWifiController::AP) {
        rb::DnsServer::get().start("esp32.local", [](){
            return EspWifiController::get().currentIp().addr;
        });
    }*/

    UI.begin(_protocol.get());
    GridUiContext::get().createClasses(context);
    builderCallback.call<void>(jac::Class<GridUiBuilderProtoBuilder>::createInstance(context, this));
    GridUiContext::get().clearProtos();
    UI.commit();
}

void GridUiHolder::end(jac::ContextRef context) {
    if(!_webServerTask) {
        return;
    }

    gridui::UI.end();
    GridUiContext::get().clear(context);

    rb_web_stop(_webServerTask);
    _webServerTask = nullptr;

    if(_protocol) {
        _protocol->stop();
        _protocol.reset();

        _ownerName = std::string();
        _deviceName = std::string();
    }

    rb::DnsServer::get().stop();
}
