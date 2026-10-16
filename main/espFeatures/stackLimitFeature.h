#pragma once

#include <jac/machine/machine.h>

#include <cstddef>
#include <cstdint>

#include "esp_cpu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


template<class Next>
class StackLimitFeature : public Next {
public:
    size_t jsStackMargin = 0;

    void initialize() {
        Next::initialize();
        if (jsStackMargin == 0) {
            return;
        }
        auto sp = reinterpret_cast<uintptr_t>(esp_cpu_get_sp());
        auto start = reinterpret_cast<uintptr_t>(pxTaskGetStackStart(nullptr));
        if (sp - start > jsStackMargin) {
            JS_SetMaxStackSize(this->runtime(), sp - start - jsStackMargin);
        }
    }
};
