#include <jac/machine/machine.h>
#include <jac/machine/values.h>
#include <jac/features/eventLoopFeature.h>
#include <jac/features/timersFeature.h>
#include <jac/features/moduleLoaderFeature.h>
#include <jac/features/filesystemFeature.h>
#include <jac/features/basicStreamFeature.h>
#include <jac/features/stdioFeature.h>

#include <jac/device/device.h>
#include <jac/device/logger.h>
#include <jac/features/keyvalueFeature.h>
#include <jac/features/util/linkIo.h>

#include <jac/link/mux.h>
#include <jac/link/encoders/cobs.h>

#include "espFeatures/smartLedFeature.h"
#include "espFeatures/gpioFeature.h"
#include "espFeatures/freeRTOSEventQueue.h"
#include "espFeatures/ledcFeature.h"
#include "espFeatures/adcFeature.h"
#include "espFeatures/i2cFeature.h"
#include "espFeatures/simpleRadioFeature.h"
#include "espFeatures/extendLifetimeFeature.h"
#include "espFeatures/pulseCounterFeature.h"
#include "espFeatures/timestampFeature.h"
#include "espFeatures/wifiFeature.h"
#include "espFeatures/gridui/gridUiFeature.h"
#include "espFeatures/motorFeature.h"

#include "platform/espNvsKeyValue.h"
#include "platform/espWifi.h"

#include "util/uartStream.h"
#include "util/tcpStream.h"
#ifdef CONFIG_JAC_ESP32_ENABLE_BLE
#include "util/bleStream.h"
#endif

#include "resources/resources.h"

#include <string>
#include <filesystem>
#include <sstream>

#include "esp_vfs_fat.h"
#include "nvs_flash.h"
#include "freertos/task.h"

#include "esp_pthread.h"


#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
    #include "util/jtagStream.h"
#endif


#if defined(CONFIG_IDF_TARGET_ESP32)
    #include "platform/esp32.h"
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    #include "platform/esp32s3.h"
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
    #include "platform/esp32c3.h"
#endif

wl_handle_t storage_wl_handle = WL_INVALID_HANDLE;

using Machine = jac::ComposeMachine<
    jac::MachineBase,
    FreeRTOSEventQueueFeature,
    jac::BasicStreamFeature,
    jac::StdioFeature,
    PlatformInfoFeature,
    jac::EventLoopFeature,
    jac::FilesystemFeature,
    jac::ModuleLoaderFeature,
    jac::TimersFeature,
    TimestampFeature,
    ExtendLifetimeFeature,
    GpioFeature,
    LedcFeature,
    AdcFeature,
    I2CFeature,
    SmartLedFeature,
    PulseCounterFeature,
    SimpleRadioFeature,
    MotorFeature,
    WifiFeature,
    GridUiFeature,
    jac::KeyValueFeature,
    jac::EventLoopTerminal
>;

jac::Device<Machine> device(
    "/data",
    []() { // get memory stats
        std::stringstream oss;
        oss << esp_get_free_heap_size() << "; min " << esp_get_minimum_free_heap_size()
            << "\nIRAM " << heap_caps_get_free_size(MALLOC_CAP_INTERNAL) << "; IRAM min " << heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
        return oss.str();
    },
    []() { // get storage stats
        // std::stringstream oss;
        // auto stats = std::filesystem::space("/data");
        // oss << "Storage usage: \n  " << stats.available << "/" << stats.capacity << "\n";
        // return oss.str();
        return "not implemented";
    },
    {{"esp32", JAC_ESP32_VERSION}}, // version info
    [](std::filesystem::path path) { // format storage
        jac::Logger::debug("Formatting storage");

        esp_vfs_fat_spiflash_unmount_rw_wl("/data", storage_wl_handle);

        auto* partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
        if (partition == nullptr) {
            return;
        }
        esp_partition_erase_range(partition, 0, partition->size);
    },
    { // resources
        {"ts-examples", resources::tsExamplesTgz}
    },
    EspNvsKeyValue::open
);

using Mux_t = jac::Mux<jac::CobsEncoder>;
std::unique_ptr<Mux_t> muxUart;
std::unique_ptr<Mux_t> muxTcp;
#ifdef CONFIG_JAC_ESP32_ENABLE_BLE
std::unique_ptr<Mux_t> muxBle;
#endif

#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
    std::unique_ptr<Mux_t> muxJtag;
#endif

void reportMuxError(jac::MuxError error, std::any ctx) {
    std::string message = "Mux error: ";
    switch (error) {
        case jac::MuxError::INVALID_RECEIVE:
            {
                auto& ref = std::any_cast<std::tuple<int, uint8_t>&>(ctx);
                message += "INVALID_RECEIVE " + std::to_string(std::get<0>(ref));
                message += " [" + std::to_string(std::get<1>(ref)) + "]";
                jac::Logger::debug(message);
            }
            break;
        case jac::MuxError::PACKETIZER:
            {
                auto& ref = std::any_cast<int&>(ctx);
                message += "PACKETIZER_ERROR " + std::to_string(ref);
                jac::Logger::debug(message);
            }
            break;
        case jac::MuxError::PROCESSING:
            {
                auto& ref = std::any_cast<std::string&>(ctx);
                message += "PROCESSING_ERROR '" + ref + "'";
                jac::Logger::error(message);
            }
            break;
    }
}

int main() {
    // Initialize vfs
    esp_vfs_fat_mount_config_t conf = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE,
        .disk_status_check_enable = false
    };

    ESP_ERROR_CHECK(esp_vfs_fat_spiflash_mount_rw_wl("/data", "storage", &conf, &storage_wl_handle));

    // Initialize nvs
    auto err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err == ESP_OK) {
            err = nvs_flash_init();
        }
    }
    ESP_ERROR_CHECK(err);

    // initialize wifi
    auto& wifi = EspWifiController::get();
    wifi.initialize();
    device.onKeyValueModified(std::bind(&EspWifiController::onKeyValueModified, &wifi, std::placeholders::_1, std::placeholders::_2));

    // initialize uart connection
    auto uartStream = std::make_unique<UartStream>(UART_NUM_0, 921600, 4096, 0);
    uartStream->start();

    muxUart = std::make_unique<Mux_t>(std::move(uartStream));
    muxUart->setErrorHandler(reportMuxError);
    auto handleUart = device.router().subscribeTx(1, *muxUart);
    muxUart->bindRx(std::make_unique<decltype(handleUart)>(std::move(handleUart)));

#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
    // initialize usb connection
    auto jtagStream = std::make_unique<JtagStream>(4096, 1024);
    jtagStream->start();

    muxJtag = std::make_unique<Mux_t>(std::move(jtagStream));
    muxJtag->setErrorHandler(reportMuxError);
    auto handleUsb = device.router().subscribeTx(2, *muxJtag);
    muxJtag->bindRx(std::make_unique<decltype(handleUsb)>(std::move(handleUsb)));
#endif

    if (wifi.mode() != EspWifiController::DISABLED) {
        // initialize tcp stream
        auto tcpStream = std::make_unique<TcpStream>();
        tcpStream->start();

        muxTcp = std::make_unique<Mux_t>(std::move(tcpStream));
        muxTcp->setErrorHandler(reportMuxError);
        auto handleTcp = device.router().subscribeTx(3, *muxTcp);
        muxTcp->bindRx(std::make_unique<decltype(handleTcp)>(std::move(handleTcp)));
    }

#ifdef CONFIG_JAC_ESP32_ENABLE_BLE
    // initialize BLE stream
    auto bleStream = std::make_unique<BleStream>("ESP32_JAC_BLE");
    bleStream->start();

    muxBle = std::make_unique<Mux_t>(std::move(bleStream));
    muxBle->setErrorHandler(reportMuxError);
    auto handleBle = device.router().subscribeTx(4, *muxBle);
    muxBle->bindRx(std::make_unique<decltype(handleBle)>(std::move(handleBle)));
#endif


    device.onConfigureMachine([&](Machine &machine) {
        device.machineIO().in->clear();

        machine.stdio.out = std::make_unique<jac::LinkWritable>(device.machineIO().out.get());
        machine.stdio.err = std::make_unique<jac::LinkWritable>(device.machineIO().err.get());
        machine.stdio.in = std::make_unique<jac::LinkReadable<Machine>>(&machine, device.machineIO().in.get());

        machine.kvOpener = device.getKeyValueOpener();
        machine.setMallocFunctions(&JsEspMallocFunctions<MALLOC_CAP_DEFAULT>::js_esp_malloc_functions);

        esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
        cfg.stack_size = 2 * 1024;
        cfg.inherit_cfg = true;
        esp_pthread_set_cfg(&cfg);
    });

    esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
    cfg.stack_size = 10 * 1024;
    cfg.inherit_cfg = true;
    esp_pthread_set_cfg(&cfg);

    device.start();

    esp_reset_reason_t resetReason = esp_reset_reason();
    const char* resetReasonStr = nullptr;
    bool startMachine = false;

    switch (resetReason) {
        case ESP_RST_PANIC:     //!< Software reset due to exception/panic
            resetReasonStr = "Software reset due to exception/panic";
            break;
        case ESP_RST_INT_WDT:   //!< Reset (software or hardware) due to interrupt watchdog
            resetReasonStr = "Reset (software or hardware) due to interrupt watchdog";
            break;
        case ESP_RST_TASK_WDT:  //!< Reset due to task watchdog
            resetReasonStr = "Reset due to task watchdog";
            break;
        case ESP_RST_WDT:       //!< Reset due to other watchdogs
            resetReasonStr = "Reset due to other watchdogs";
            break;
        default:
            startMachine = true;
            break;
    }

    if (resetReasonStr != nullptr) {
        jac::Logger::error(std::string(resetReasonStr));
    }

    if (startMachine) {
        device.startMachine("index.js");
    }
}


extern "C" void app_main() {
    main();

    vTaskDelete(NULL);
}
