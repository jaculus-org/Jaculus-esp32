#include <thread>

#include "esp_netif.h"
#include "esp_wifi.h"

#include "espWifi.h"


EspWifiController::EspWifiController() :
        _mode(Mode::DISABLED),
        _staMode(StaMode::BEST_SIGNAL),
        _apFallback(true),
        _currentIp({}),
        _eventLoopOurs(false),
        _handler_wifi(nullptr),
        _handler_ip(nullptr),
        _netIf(nullptr, &esp_netif_destroy_default_wifi),
        _kvModifyTimer(nullptr)
{
    esp_timer_create_args_t args;
    args.callback = onKvModifiedTimer;
    args.arg = static_cast<void*>(this);
    args.dispatch_method = ESP_TIMER_TASK;

    auto err = esp_timer_create(&args, &_kvModifyTimer);
    if(err != ESP_OK) {
        jac::Logger::error("EspWifiController esp_timer_create: " + std::string(esp_err_to_name(err)));
    }
}

bool EspWifiController::initialize() {
    auto kvMain = EspNvsKeyValue::open(KvNsWifiMain);
    if(!kvMain) {
        return false;
    }

    _staMode = static_cast<StaMode>(kvMain->getInt(KeyWifiStaMode, StaMode::BEST_SIGNAL));

    _apFallback = kvMain->getInt(KeyWifiStaApFallback, 1) != 0;

    return switchMode(static_cast<Mode>(kvMain->getInt(KeyWifiMode, Mode::DISABLED)), std::move(kvMain));
}

bool EspWifiController::initGlobalsLocked() {
    auto err = esp_netif_init();
    if(err != ESP_OK) {
        jac::Logger::error("esp_netif_init: " + std::string(esp_err_to_name(err)));
        return false;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.nvs_enable = 0;
    cfg.nano_enable = 1;

    err = esp_event_loop_create_default();
    _eventLoopOurs = (err == ESP_OK);
    if(err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        jac::Logger::error("esp_event_loop_create_default: " + std::string(esp_err_to_name(err)));
        goto fail_netif;
    }

    err = esp_wifi_init(&cfg);
    if(err != ESP_OK) {
        jac::Logger::error("esp_wifi_init: " + std::string(esp_err_to_name(err)));
        goto fail_netif;
    }
    return true;

fail_netif:
    if(_eventLoopOurs) {
        esp_event_loop_delete_default();
        _eventLoopOurs = false;
    }
    esp_netif_deinit();
    return false;
}

bool EspWifiController::registerEventHandlersLocked() {
    auto err = esp_event_handler_instance_register(WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            &EspWifiController::eventHandlerWifi,
            static_cast<void*>(this),
            &_handler_wifi);
    if(err != ESP_OK) {
        jac::Logger::error("esp_event_handler_instance_register(WIFI_EVENT): " + std::string(esp_err_to_name(err)));
        return false;
    }

    err = esp_event_handler_instance_register(IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            &EspWifiController::eventHandlerIp,
            this,
            &_handler_ip);
    if(err != ESP_OK) {
        jac::Logger::error("esp_event_handler_instance_register(IP_EVENT): " + std::string(esp_err_to_name(err)));
        esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, _handler_wifi);
        _handler_wifi = nullptr;
        return false;
    }
    return true;
}

void EspWifiController::unregisterEventHandlersLocked() {
    if(_handler_wifi) {
        esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, _handler_wifi);
        _handler_wifi = nullptr;
    }
    if(_handler_ip) {
        esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, _handler_ip);
        _handler_ip = nullptr;
    }
}

void EspWifiController::stopWifiLocked() {
    unregisterEventHandlersLocked();

    SemaphoreHandle_t stopSem = xSemaphoreCreateBinary();

    esp_event_handler_instance_t handler = nullptr;
    esp_event_handler_instance_register(WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            &EspWifiController::eventHandlerWifiStop,
            static_cast<void*>(stopSem),
            &handler);

    if(esp_wifi_stop() == ESP_OK) {
        if(xSemaphoreTake(stopSem, pdMS_TO_TICKS(1000)) == pdFALSE) {
           jac::Logger::error("Wait for Wifi Stop event timed out.");
        }
    }

    if(handler) {
        esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, handler);
    }
    vSemaphoreDelete(stopSem);

    _netIf.reset();

    _currentIp.addr = 0;
}

void EspWifiController::deinitGlobalsLocked() {
    stopWifiLocked();
    esp_wifi_deinit();

    if(_eventLoopOurs) {
        esp_event_loop_delete_default();
        _eventLoopOurs = false;
    }

    esp_netif_deinit();
}


void EspWifiController::onKeyValueModified(const std::string& nsname, const std::string& key) {
    if(nsname != KvNsWifiMain && nsname != KvNsWifiSsid) {
        return;
    }

    esp_timer_stop(_kvModifyTimer);

    auto err = esp_timer_start_once(_kvModifyTimer, 1000000);
    if(err != ESP_OK) {
        jac::Logger::error("EspWifiController esp_timer_start_once: " + std::string(esp_err_to_name(err)));
    }
}

void EspWifiController::onKvModifiedTimer(void *selfVoid) {
    auto *self = static_cast<EspWifiController*>(selfVoid);

    jac::Logger::debug("Reloading WiFi settings from NVS");

    self->switchMode(Mode::DISABLED);
    self->initialize();
}

bool EspWifiController::switchMode(Mode newMode, std::unique_ptr<jac::KeyValueNamespace> mainNs) {
    std::lock_guard<std::mutex> lg(_modeLock);

    if(newMode >= Mode::MAX) {
        newMode = Mode::DISABLED;
    }

    if(_mode == newMode) {
        return true;
    }

    jac::Logger::debug("Wifi mode switch from " + std::to_string(_mode) + " to " + std::to_string(newMode));

    if(newMode == Mode::DISABLED) {
        deinitGlobalsLocked();
        _mode = newMode;
        return true;
    }

    if(!mainNs) {
        mainNs = EspNvsKeyValue::open(KvNsWifiMain);
        if(!mainNs) {
            return false;
        }
    }

    if(_mode == Mode::DISABLED) {
        // Init for NetIf, Wifi Driver
        if(!initGlobalsLocked()) {
            return false;
        }
    }
    else {
        // stop previous mode
        stopWifiLocked();
    }

    _mode = newMode;

    switch(_mode) {
    case Mode::STATION: {
        _netIf.reset(esp_netif_create_default_wifi_sta());

        if(!registerEventHandlersLocked()) {
            return false;
        }

        auto err = esp_wifi_set_mode(WIFI_MODE_STA);
        if(err != ESP_OK) {
            jac::Logger::error("esp_wifi_set_mode(WIFI_MODE_STA): " + std::string(esp_err_to_name(err)));
            return false;
        }

        wifi_config_t cfg = {};
        cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;

        if(_staMode == StaMode::SPECIFIC_SSID) {
            auto wifiNs = EspNvsKeyValue::open(KvNsWifiSsid);
            if(!wifiNs) {
                return false;
            }

            auto spec_ssid = mainNs->getString(KeyWifiStaSpecific);
            snprintf(reinterpret_cast<char*>(cfg.sta.ssid), 32, "%s", spec_ssid.c_str());  // NOLINT
            snprintf(reinterpret_cast<char*>(cfg.sta.password), 64, "%s", wifiNs->getString(spec_ssid).c_str());  // NOLINT
        }

        err = esp_wifi_set_config(WIFI_IF_STA, &cfg);
        if(err != ESP_OK) {
            jac::Logger::error("esp_wifi_set_config(WIFI_IF_STA): " + std::string(esp_err_to_name(err)));
            return false;
        }
        break;
    }
    case Mode::AP: {
        _netIf.reset(esp_netif_create_default_wifi_ap());

        if(!registerEventHandlersLocked()) {
            return false;
        }

        auto err = esp_wifi_set_mode(WIFI_MODE_AP);
        if(err != ESP_OK) {
            jac::Logger::error("esp_wifi_set_mode(WIFI_MODE_AP): " + std::string(esp_err_to_name(err)));
            return false;
        }

        const auto ssid = mainNs->getString(KeyWifiApSsid, "Jaculus-esp32");
        const auto pass = mainNs->getString(KeyWifiApPass);

        wifi_config_t cfg = {};
        if (pass.size() >= 8) {
            snprintf(reinterpret_cast<char*>(cfg.ap.password), 64, "%s", pass.c_str());  // NOLINT
            cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
        }
        else {
            if(pass.size() != 0) {
                jac::Logger::error("The WiFi password is too short, 8 characters required, leaving the WiFI open!");
            }
            cfg.ap.authmode = WIFI_AUTH_OPEN;
        }
        snprintf(reinterpret_cast<char*>(cfg.ap.ssid), 32, "%s", ssid.c_str());  // NOLINT
        cfg.ap.beacon_interval = 400;
        cfg.ap.max_connection = 4;

        err = esp_wifi_set_config(WIFI_IF_AP, &cfg);
        if(err != ESP_OK) {
            jac::Logger::error("esp_wifi_set_config: " + std::string(esp_err_to_name(err)));
            return false;
        }
        break;
    }
    default:
        break;
    }

    auto err = esp_wifi_start();
    if(err != ESP_OK) {
        jac::Logger::error("esp_wifi_start: " + std::string(esp_err_to_name(err)));
        return false;
    }

    return true;
}

void EspWifiController::eventHandlerWifi(void* selfVoid, esp_event_base_t event_base,
        int32_t event_id, void* event_data) {
    auto *self = static_cast<EspWifiController*>(selfVoid);

    switch(event_id) {
        case WIFI_EVENT_STA_START:
        case WIFI_EVENT_STA_DISCONNECTED: {
            std::lock_guard<std::mutex> lg(self->_modeLock);

            if(self->_mode != Mode::STATION) {
                break;
            }

            if(self->_staMode == StaMode::SPECIFIC_SSID) {
                auto err = esp_wifi_connect();
                if(err != ESP_OK) {
                    jac::Logger::error("esp_wifi_connect: " + std::string(esp_err_to_name(err)));
                }
            }
            else if(self->_staMode == StaMode::BEST_SIGNAL) {
                auto err = esp_wifi_scan_start(NULL, false);
                if(err != ESP_OK) {
                    jac::Logger::error("esp_wifi_scan_start: " + std::string(esp_err_to_name(err)));
                }
            }
            break;
        }
        case WIFI_EVENT_AP_START: {
            std::lock_guard<std::mutex> lg(self->_modeLock);
            esp_netif_ip_info_t ip_info = {};
            esp_netif_get_ip_info(self->_netIf.get(), &ip_info);
            self->_currentIp = ip_info.ip;
            break;
        }
        case WIFI_EVENT_SCAN_DONE: {
            std::lock_guard<std::mutex> lg(self->_modeLock);

            if(self->_mode != Mode::STATION || self->_staMode != StaMode::BEST_SIGNAL) {
                esp_wifi_clear_ap_list();
                break;
            }

            auto wifiNs = EspNvsKeyValue::open(KvNsWifiSsid);
            if(!wifiNs) {
                esp_wifi_clear_ap_list();
                break;
            }

            wifi_ap_record_t rec;
            const std::string not_exists_sentinel = "a";
            bool connected = false;
            while(!connected) {
                auto err = esp_wifi_scan_get_ap_record(&rec);
                if(err != ESP_OK) {
                    break;
                }

                // kv key len limits us, use prefix match
                rec.ssid[15] = 0;
                auto pass = wifiNs->getString(reinterpret_cast<const char*>(rec.ssid), not_exists_sentinel);  // NOLINT
                if(pass == not_exists_sentinel) {
                    continue;
                }

                wifi_config_t cfg = {};
                snprintf(reinterpret_cast<char*>(cfg.sta.ssid), 32, "%.31s", reinterpret_cast<const char*>(rec.ssid));  // NOLINT
                snprintf(reinterpret_cast<char*>(cfg.sta.password), 64, "%s", pass.c_str());  // NOLINT
                err = esp_wifi_set_config(WIFI_IF_STA, &cfg);
                if(err != ESP_OK) {
                    jac::Logger::error("esp_wifi_set_config: " + std::string(esp_err_to_name(err)));
                    continue;
                }

                err = esp_wifi_connect();
                if(err != ESP_OK) {
                    jac::Logger::error("esp_wifi_connect: " + std::string(esp_err_to_name(err)));
                    continue;
                }

                connected = true;
            }

            esp_wifi_clear_ap_list();

            if(!connected && self->_apFallback) {
                std::thread([self](){
                    self->switchMode(Mode::AP);
                }).detach();
            }
            break;
        }
    }
}

void EspWifiController::eventHandlerWifiStop(void* semVoid, esp_event_base_t event_base,
        int32_t event_id, void* event_data) {
    switch(event_id) {
        case WIFI_EVENT_STA_STOP:
        case WIFI_EVENT_AP_STOP: {
            auto *sem = static_cast<SemaphoreHandle_t>(semVoid);
            xSemaphoreGive(sem);
            break;
        }
    }
}

void EspWifiController::eventHandlerIp(void* selfVoid, esp_event_base_t event_base,
        int32_t event_id, void* event_data) {
    auto *self = static_cast<EspWifiController*>(selfVoid);

    auto event = static_cast<ip_event_got_ip_t*>(event_data);

    self->_currentIp = event->ip_info.ip;

    char buf[16];
    esp_ip4addr_ntoa(&event->ip_info.ip, buf, sizeof(buf));
    jac::Logger::debug("SYSTEM_EVENT_STA_GOT_IP: " + std::string(buf));
}
