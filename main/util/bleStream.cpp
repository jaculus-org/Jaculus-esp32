#include "bleStream.h"
#include <cstring>

// Static member initialization
esp_ble_adv_data_t BleStream::adv_data = {};
esp_ble_adv_params_t BleStream::adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .peer_addr = {},
    .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

void BleStream::start() {
    if (_isInitialized) {
        return;
    }

    BLE_LOG_INFO("Starting BLE Stream initialization...");
    esp_err_t ret;

    // Release Classic BT memory (only if not already released)
    static bool bt_mem_released = false;
    if (!bt_mem_released) {
        ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
        if (ret == ESP_OK) {
            bt_mem_released = true;
        } else {
            BLE_LOG_ERROR("Failed to release Classic BT memory: " + std::string(esp_err_to_name(ret)));
        }
    }

    // Initialize BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        BLE_LOG_ERROR("BLE controller init failed: " + std::string(esp_err_to_name(ret)));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        BLE_LOG_ERROR("BLE controller enable failed: " + std::string(esp_err_to_name(ret)));
        return;
    }

    // Initialize Bluedroid
    ret = esp_bluedroid_init();
    if (ret) {
        BLE_LOG_ERROR("Bluedroid init failed: " + std::string(esp_err_to_name(ret)));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        BLE_LOG_ERROR("Bluedroid enable failed: " + std::string(esp_err_to_name(ret)));
        return;
    }

    // Register callbacks
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        BLE_LOG_ERROR("GATTS register callback failed: " + std::string(esp_err_to_name(ret)));
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        BLE_LOG_ERROR("GAP register callback failed: " + std::string(esp_err_to_name(ret)));
        return;
    }

    // Register GATT application
    ret = esp_ble_gatts_app_register(0);
    if (ret) {
        BLE_LOG_ERROR("GATTS app register failed: " + std::string(esp_err_to_name(ret)));
        return;
    }

    // Set MTU
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(MAX_CHUNK_SIZE);
    if (local_mtu_ret) {
        BLE_LOG_ERROR("Set local MTU failed: " + std::string(esp_err_to_name(local_mtu_ret)));
    }

    _isInitialized = true;
    BLE_LOG_INFO("BLE Stream initialized successfully");
}

bool BleStream::put(uint8_t c) {
    std::array<uint8_t, 1> arr{c};
    return write(std::span<const uint8_t>(arr)) == 1;
}

size_t BleStream::write(std::span<const uint8_t> data) {
    if (!_isConnected || !_notifyEnabled) {
        return data.size(); // Pretend write succeeded when not connected
    }

    size_t written = 0;
    while (written < data.size()) {
        size_t chunkSize = std::min(MAX_CHUNK_SIZE, data.size() - written);

        esp_err_t ret = esp_ble_gatts_send_indicate(
            gatts_if,
            conn_id,
            char_handle,
            chunkSize,
            const_cast<uint8_t*>(data.data() + written),
            false // notification, not indication
        );

        if (ret != ESP_OK) {
            BLE_LOG_ERROR("BLE notification failed: " + std::string(esp_err_to_name(ret)));
            break;
        }

        written += chunkSize;

        // Small delay to avoid overwhelming the BLE stack
        if (written < data.size()) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    return written;
}

int BleStream::get() {
    std::lock_guard<std::mutex> lock(_bufferMutex);
    if (_buffer.empty()) {
        return -1;
    }
    uint8_t c = _buffer.front();
    _buffer.pop_front();
    return c;
}

size_t BleStream::read(std::span<uint8_t> data) {
    std::lock_guard<std::mutex> lock(_bufferMutex);
    size_t len = std::min(data.size(), _buffer.size());
    std::copy_n(_buffer.begin(), len, data.begin());
    _buffer.erase(_buffer.begin(), _buffer.begin() + len);
    return len;
}

bool BleStream::flush() {
    return true; // BLE notifications are sent immediately
}

void BleStream::onData(std::function<void(void)> callback) {
    _onData = callback;
}

void BleStream::cleanup() {
    if (_isInitialized) {
        esp_bluedroid_disable();
        esp_bluedroid_deinit();
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        _isInitialized = false;
    }
}

void BleStream::addReceivedData(const uint8_t* data, size_t len) {
    {
        std::lock_guard<std::mutex> lock(_bufferMutex);
        _buffer.insert(_buffer.end(), data, data + len);
    }

    if (_onData) {
        _onData();
    }
}

std::string BleStream::generateDeviceName() {
    uint8_t mac[6];
    esp_err_t ret = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (ret != ESP_OK) {
        BLE_LOG_ERROR("Failed to read MAC address: " + std::string(esp_err_to_name(ret)));
        return std::string(DEVICE_NAME_PREFIX);
    }

    // Use last 2 bytes of MAC address for uniqueness
    char name_buffer[32];
    snprintf(name_buffer, sizeof(name_buffer), "%s_%02X%02X",
            DEVICE_NAME_PREFIX, mac[4], mac[5]);
    return std::string(name_buffer);
}

void BleStream::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            BLE_LOG_ERROR("Advertising start failed with status: " + std::to_string(param->adv_start_cmpl.status));
        } else {
            BLE_LOG_INFO("BLE advertising started successfully! Device should be discoverable now.");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            BLE_LOG_ERROR("Advertising stop failed");
        }
        break;
    default:
        break;
    }
}

void BleStream::gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if_param, esp_ble_gatts_cb_param_t *param) {
    if (!instance) return;

    switch (event) {
    case ESP_GATTS_REG_EVT:
        {
            gatts_if = gatts_if_param;

            // Generate device name with MAC address
            std::string device_name = generateDeviceName();
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(device_name.c_str());
            if (set_dev_name_ret) {
                BLE_LOG_ERROR("Set device name failed: " + std::string(esp_err_to_name(set_dev_name_ret)));
            }

            // Configure advertising data
            esp_ble_adv_data_t adv_data = {
                .set_scan_rsp = false,
                .include_name = true,
                .include_txpower = true,
                .min_interval = 0x0006,
                .max_interval = 0x0010,
                .appearance = 0x00,
                .manufacturer_len = 0,
                .p_manufacturer_data = nullptr,
                .service_data_len = 0,
                .p_service_data = nullptr,
                .service_uuid_len = 0,
                .p_service_uuid = nullptr,
                .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
            };

            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret) {
                BLE_LOG_ERROR("Config adv data failed: " + std::string(esp_err_to_name(ret)));
            }

            // Create service
            esp_gatt_srvc_id_t service_id;
            service_id.is_primary = true;
            service_id.id.inst_id = 0x00;
            service_id.id.uuid.len = ESP_UUID_LEN_16;
            service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID;
            esp_err_t create_ret = esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLE);
            if (create_ret) {
                BLE_LOG_ERROR("Create service failed: " + std::string(esp_err_to_name(create_ret)));
            }
            break;
        }

    case ESP_GATTS_CREATE_EVT:
        {
            service_handle = param->create.service_handle;

            esp_err_t start_ret = esp_ble_gatts_start_service(service_handle);
            if (start_ret) {
                BLE_LOG_ERROR("Start service failed: " + std::string(esp_err_to_name(start_ret)));
            }

            // Add characteristic
            esp_bt_uuid_t char_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = GATTS_CHAR_UUID}
            };

            esp_gatt_char_prop_t char_property = ESP_GATT_CHAR_PROP_BIT_READ |
                                                ESP_GATT_CHAR_PROP_BIT_WRITE |
                                                ESP_GATT_CHAR_PROP_BIT_NOTIFY;

            esp_err_t add_char_ret = esp_ble_gatts_add_char(service_handle, &char_uuid,
                                 ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                 char_property, nullptr, nullptr);
            if (add_char_ret) {
                BLE_LOG_ERROR("Add characteristic failed: " + std::string(esp_err_to_name(add_char_ret)));
            }
            break;
        }

    case ESP_GATTS_ADD_CHAR_EVT:
        {
            char_handle = param->add_char.attr_handle;

            // Add descriptor for notifications
            esp_bt_uuid_t descr_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = GATTS_DESCR_UUID}
            };

            esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(service_handle, &descr_uuid,
                                       ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                       nullptr, nullptr);
            if (add_descr_ret) {
                BLE_LOG_ERROR("Add descriptor failed: " + std::string(esp_err_to_name(add_descr_ret)));
            }
            break;
        }

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        {
            descr_handle = param->add_char_descr.attr_handle;
            // Now that everything is set up, start advertising
            esp_err_t adv_start_ret = esp_ble_gap_start_advertising(&adv_params);
            if (adv_start_ret) {
                BLE_LOG_ERROR("Failed to start advertising: " + std::string(esp_err_to_name(adv_start_ret)));
            }
            break;
        }

    case ESP_GATTS_START_EVT:
        break;

    case ESP_GATTS_CONNECT_EVT:
        {
            BLE_LOG_INFO("BLE client connected");
            conn_id = param->connect.conn_id;
            instance->_isConnected = true;

            // Update connection parameters
            esp_ble_conn_update_params_t conn_params = {};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.latency = 0;
            conn_params.max_int = 0x20;
            conn_params.min_int = 0x10;
            conn_params.timeout = 400;
            esp_ble_gap_update_conn_params(&conn_params);
            break;
        }

    case ESP_GATTS_DISCONNECT_EVT:
        BLE_LOG_INFO("BLE client disconnected");
        instance->_isConnected = false;
        instance->_notifyEnabled = false;
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_WRITE_EVT:
        {
            if (param->write.handle == descr_handle && param->write.len == 2) {
                // Client Characteristic Configuration descriptor written
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                if (descr_value == 0x0001) {
                    BLE_LOG_INFO("BLE notifications enabled");
                    instance->_notifyEnabled = true;
                } else {
                    instance->_notifyEnabled = false;
                }
            } else if (param->write.handle == char_handle) {
                // Data written to characteristic
                instance->addReceivedData(param->write.value, param->write.len);
            }

            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                          param->write.trans_id, ESP_GATT_OK, nullptr);
            }
            break;
        }

    case ESP_GATTS_READ_EVT:
        {
            // Respond to read requests with empty data
            esp_gatt_rsp_t rsp = {};
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = 0;
            rsp.attr_value.value[0] = 0x00; // Set at least one byte

            esp_err_t rsp_err = esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
                                      param->read.trans_id, ESP_GATT_OK, &rsp);
            if (rsp_err != ESP_OK) {
                BLE_LOG_ERROR("Failed to send read response: " + std::string(esp_err_to_name(rsp_err)));
            }
            break;
        }

    // Handle other events without error
    case ESP_GATTS_EXEC_WRITE_EVT:
    case ESP_GATTS_MTU_EVT:
    case ESP_GATTS_CONF_EVT:
    case ESP_GATTS_UNREG_EVT:
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
    case ESP_GATTS_DELETE_EVT:
    case ESP_GATTS_STOP_EVT:
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    case ESP_GATTS_RESPONSE_EVT:
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    case ESP_GATTS_SET_ATTR_VAL_EVT:
    case ESP_GATTS_SEND_SERVICE_CHANGE_EVT:
    default:
        break;
    }
}
