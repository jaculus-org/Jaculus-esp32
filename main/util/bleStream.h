#pragma once

#include <jac/link/stream.h>
#include <jac/device/logger.h>

#include <atomic>
#include <array>
#include <cstdint>
#include <deque>
#include <functional>
#include <span>
#include <string>
#include <mutex>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_gatt_common_api.h"
#include "esp_system.h"
#include "esp_mac.h"

// BLE Debug Logging Control
// Can be controlled via Kconfig (menuconfig) or by defining BLE_STREAM_DEBUG_LOGS
#if defined(CONFIG_JAC_ESP32_BLE_DEBUG_LOGS) || defined(BLE_STREAM_DEBUG_LOGS)
    #define BLE_LOG_DEBUG(msg) jac::Logger::debug(msg)
    #define BLE_LOG_ERROR(msg) jac::Logger::error(msg)
    #define BLE_LOG_INFO(msg) jac::Logger::debug(msg)   // Use debug for info when enabled
#else
    #define BLE_LOG_DEBUG(msg) ((void)0)
    #define BLE_LOG_ERROR(msg) jac::Logger::error(msg)  // Keep errors always visible
    #define BLE_LOG_INFO(msg) ((void)0)                 // Disable info messages when debugging is off
#endif

/**
 * @brief BLE Stream implementation for ESP32
 *
 * Provides a BLE GATT server that can be used as a duplex stream for communication.
 * Integrates with the Jaculus-esp32 project as an additional transport method.
 */
class BleStream : public jac::Duplex {
private:
    // Member variables
    std::function<void()> _onData;
    std::deque<uint8_t> _buffer;
    std::mutex _bufferMutex;
    std::atomic<bool> _isConnected{false};
    std::atomic<bool> _notifyEnabled{false};
    std::atomic<bool> _isInitialized{false};

    // BLE configuration constants
    static constexpr uint16_t GATTS_SERVICE_UUID = CONFIG_JAC_ESP32_BLE_SERVICE_UUID;
    static constexpr uint16_t GATTS_CHAR_UUID = CONFIG_JAC_ESP32_BLE_CHARACTERISTIC_UUID;
    static constexpr uint16_t GATTS_DESCR_UUID = 0x2902; // Client Characteristic Configuration
    static constexpr uint16_t GATTS_NUM_HANDLE = 4;
    static constexpr const char* DEVICE_NAME_PREFIX = CONFIG_JAC_ESP32_BLE_DEVICE_NAME;
    static constexpr size_t MAX_CHUNK_SIZE = CONFIG_JAC_ESP32_BLE_MTU_SIZE; // Configurable MTU size

    // BLE handles - shared across all instances
    static inline esp_gatt_if_t gatts_if = ESP_GATT_IF_NONE;
    static inline uint16_t service_handle = 0;
    static inline uint16_t char_handle = 0;
    static inline uint16_t descr_handle = 0;
    static inline uint16_t conn_id = 0;

    // Global instance pointer for callbacks (singleton pattern)
    static inline BleStream* instance = nullptr;

    // BLE advertising configuration
    static esp_ble_adv_data_t adv_data;
    static esp_ble_adv_params_t adv_params;

public:
    /**
     * @brief Constructor
     * @param deviceName Device name for BLE advertising (currently unused, uses DEVICE_NAME constant)
     * @throws std::runtime_error if another instance already exists (singleton pattern)
     */
    explicit BleStream(const std::string& deviceName = "BLE_STREAM") {
        if (instance != nullptr) {
            throw std::runtime_error("Only one BleStream instance is allowed");
        }
        instance = this;
    }

    // Disable copy and move operations
    BleStream(const BleStream&) = delete;
    BleStream(BleStream&&) = delete;
    BleStream& operator=(const BleStream&) = delete;
    BleStream& operator=(BleStream&&) = delete;

    /**
     * @brief Destructor - cleans up BLE resources
     */
    ~BleStream() override {
        cleanup();
        if (instance == this) {
            instance = nullptr;
        }
    }

    /**
     * @brief Initialize and start the BLE stack
     * @note This method is idempotent - calling it multiple times is safe
     */
    void start();

    // Duplex interface implementation
    bool put(uint8_t c) override;
    size_t write(std::span<const uint8_t> data) override;
    int get() override;
    size_t read(std::span<uint8_t> data) override;
    bool flush() override;
    void onData(std::function<void(void)> callback) override;

private:
    /**
     * @brief Clean up BLE resources
     */
    void cleanup();

    /**
     * @brief Add received data to buffer and trigger callback
     */
    void addReceivedData(const uint8_t* data, size_t len);

    /**
     * @brief Generate device name with MAC address suffix
     */
    static std::string generateDeviceName();

    /**
     * @brief GAP event handler
     */
    static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

    /**
     * @brief GATTS event handler
     */
    static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if_param, esp_ble_gatts_cb_param_t *param);
};
