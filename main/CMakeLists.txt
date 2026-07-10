set(JAC_ESP32_VERSION "0.2.0")

idf_component_register(
    SRCS "main.cpp"
         "platform/espWifi.cpp"
         "platform/espNvsKeyValue.cpp"
         "espFeatures/gridui/gridUiFeature.cpp"
         "espFeatures/gridui/widgets/_common.cpp"
    INCLUDE_DIRS ""
    REQUIRES jac-dcore jac-machine jac-link
             driver pthread spiffs vfs fatfs
             SmartLeds esp_timer Esp32-RBGridUI
             renderer
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE JAC_ESP32_VERSION="${JAC_ESP32_VERSION}")

if(IDF_TARGET STREQUAL "esp32" OR IDF_TARGET STREQUAL "esp32s3")
    target_compile_options(${COMPONENT_LIB} PRIVATE "-mtext-section-literals")
endif()

add_subdirectory(resources)
target_link_libraries(${COMPONENT_LIB} PRIVATE resources)
