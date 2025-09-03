set(JAC_ESP32_VERSION "0.0.20")

# Base source files
set(COMPONENT_SRCS
    "main.cpp"
    "platform/espWifi.cpp"
    "platform/espNvsKeyValue.cpp"
    "espFeatures/gridui/gridUiFeature.cpp"
    "espFeatures/gridui/widgets/_common.cpp"
)

# Base requirements
set(COMPONENT_REQUIRES
    jac-dcore jac-machine jac-link
    driver pthread spiffs vfs fatfs
    SmartLeds esp_timer Esp32-RBGridUI
)

# Conditionally add BLE support
if(CONFIG_JAC_ESP32_ENABLE_BLE)
    list(APPEND COMPONENT_SRCS "util/bleStream.cpp")
    list(APPEND COMPONENT_REQUIRES "bt")
    message(STATUS "BLE Stream support enabled")
else()
    message(STATUS "BLE Stream support disabled")
endif()

idf_component_register(
    SRCS ${COMPONENT_SRCS}
    INCLUDE_DIRS ""
    REQUIRES ${COMPONENT_REQUIRES}
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE JAC_ESP32_VERSION="${JAC_ESP32_VERSION}")

add_subdirectory(resources)
target_link_libraries(${COMPONENT_LIB} PRIVATE resources)
