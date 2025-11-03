set(JAC_ESP32_VERSION "0.0.21")

idf_component_register(
    SRCS "main.cpp" "platform/espWifi.cpp" "platform/espNvsKeyValue.cpp"
        "espFeatures/gridui/gridUiFeature.cpp" "espFeatures/gridui/widgets/_common.cpp"
    INCLUDE_DIRS ""
    REQUIRES jac-dcore jac-machine jac-link
             driver pthread spiffs vfs fatfs
             SmartLeds esp_timer Esp32-RBGridUI
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE JAC_ESP32_VERSION="${JAC_ESP32_VERSION}")

add_subdirectory(resources)
target_link_libraries(${COMPONENT_LIB} PRIVATE resources)
