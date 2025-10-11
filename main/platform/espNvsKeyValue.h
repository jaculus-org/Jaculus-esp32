#pragma once

#include <jac/device/keyvalue.h>
#include <memory>

#include "nvs_flash.h"

class EspNvsKeyValue : public jac::KeyValueNamespace {
    std::string _nsname;
    nvs_handle_t _handle;
public:
    static std::unique_ptr<EspNvsKeyValue> open(const std::string& nsname);

    EspNvsKeyValue(const std::string& nsname, nvs_handle_t handle);
    ~EspNvsKeyValue() override;

    bool erase(const std::string& name) override;

    void setInt(const std::string& name, int64_t value) override;
    void setFloat(const std::string& name, float value) override;
    void setString(const std::string& name, const std::string& value) override;

    int64_t getInt(const std::string& name, int64_t def_value = 0) override;
    float getFloat(const std::string& name, float def_value = 0.f) override;
    std::string getString(const std::string& name, std::string def_value = "") override;
    std::vector<std::string> keys() override;

    DataType getType(const std::string& name) override;

    bool commit() override;
};
