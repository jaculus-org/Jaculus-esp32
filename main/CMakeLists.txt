set(JAC_ESP32_VERSION "0.1.1")

idf_component_register(
    SRCS "main.cpp" 
        "platform/espWifi.cpp" 
        "platform/espNvsKeyValue.cpp"
        "espFeatures/gridui/gridUiFeature.cpp" 
        "espFeatures/gridui/widgets/_common.cpp"
        "espFeatures/renderer/rendererFeature.cpp"
        "espFeatures/hub75/hub75Feature.cpp"
    INCLUDE_DIRS "" "espFeatures/hub75"
    REQUIRES jac-dcore jac-machine jac-link
             driver pthread spiffs vfs fatfs
             SmartLeds esp_timer Esp32-RBGridUI
			 ESP32-HUB75-MatrixPanel-DMA
             renderer esp_timer
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC NO_GFX)

target_compile_definitions(${COMPONENT_LIB} PRIVATE JAC_ESP32_VERSION="${JAC_ESP32_VERSION}" SPIRAM_DMA_BUFFER)

target_compile_options(${COMPONENT_LIB} PRIVATE "-mtext-section-literals")

add_subdirectory(resources)
target_link_libraries(${COMPONENT_LIB} PRIVATE resources)
