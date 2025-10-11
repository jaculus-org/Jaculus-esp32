#pragma once

#include <jac/machine/machine.h>

#include "hal/adc_types.h"
#include "soc/adc_channel.h"


template<class PlatformInfo_, class Next>
class EspCommon : public Next {
public:
    using PlatformInfo = PlatformInfo_;

    static gpio_num_t getDigitalPin(int pin) {
        if (PlatformInfo::PinConfig::DIGITAL_PINS.find(pin) == PlatformInfo::PinConfig::DIGITAL_PINS.end()) {
            throw std::runtime_error("Invalid digital pin");
        }
        return static_cast<gpio_num_t>(pin);
    }

    static gpio_num_t getInterruptPin(int pin) {
        if (PlatformInfo::PinConfig::INTERRUPT_PINS.find(pin) == PlatformInfo::PinConfig::INTERRUPT_PINS.end()) {
            throw std::runtime_error("Invalid interrupt pin");
        }
        return static_cast<gpio_num_t>(pin);
    }

    void initialize() {
        Next::initialize();

        jac::ContextRef ctx = this->context();

        jac::Object platformInfo = jac::Object::create(ctx);
        platformInfo.defineProperty("name", jac::Value::from(ctx, PlatformInfo::NAME), jac::PropFlags::Enumerable);

        ctx.getGlobalObject().defineProperty("PlatformInfo", platformInfo, jac::PropFlags::Enumerable);
    }
};


template<uint32_t CAPS>
struct JsEspMallocFunctions {
    static inline constexpr int MALLOC_OVERHEAD = 8;

    /* default memory allocation functions with memory limitation */
    static size_t js_esp_malloc_usable_size(const void* ptr) {
        return heap_caps_get_allocated_size(const_cast<void*>(ptr));  // NOLINT
    }

    static void *js_esp_malloc(JSMallocState* s, size_t size) {
        void *ptr;

        /* Do not allocate zero bytes: behavior is platform dependent */
        assert(size != 0);

        if (unlikely(s->malloc_size + size > s->malloc_limit))
            return NULL;

        ptr = heap_caps_malloc(size, CAPS);
        if (!ptr)
            return NULL;

        s->malloc_count++;
        s->malloc_size += js_esp_malloc_usable_size(ptr) + MALLOC_OVERHEAD;
        return ptr;
    }

    static void js_esp_free(JSMallocState* s, void* ptr) {
        if (!ptr)
            return;

        s->malloc_count--;
        s->malloc_size -= js_esp_malloc_usable_size(ptr) + MALLOC_OVERHEAD;
        heap_caps_free(ptr);
    }

    static void *js_esp_realloc(JSMallocState* s, void* ptr, size_t size) {
        size_t old_size;

        if (!ptr) {
            if (size == 0)
                return NULL;
            return js_esp_malloc(s, size);
        }
        old_size = js_esp_malloc_usable_size(ptr);
        if (size == 0) {
            s->malloc_count--;
            s->malloc_size -= old_size + MALLOC_OVERHEAD;
            heap_caps_free(ptr);
            return NULL;
        }
        if (s->malloc_size + size - old_size > s->malloc_limit)
            return NULL;

        ptr = heap_caps_realloc(ptr, size, CAPS);
        if (!ptr)
            return NULL;

        s->malloc_size += js_esp_malloc_usable_size(ptr) - old_size;
        return ptr;
    }

    static constexpr JSMallocFunctions js_esp_malloc_functions = {
        .js_malloc = js_esp_malloc,
        .js_free = js_esp_free,
        .js_realloc = js_esp_realloc,
        .js_malloc_usable_size = js_esp_malloc_usable_size
    };
};
