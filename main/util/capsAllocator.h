#pragma once

#include "esp_heap_caps.h"
#include <memory>

using caps_t = decltype(MALLOC_CAP_DEFAULT);

template<caps_t CAPS, typename T>
struct EspCapsAllocator {
    using pointer = T*;
    using const_pointer = const T*;
    using void_pointer = void*;
    using const_void_pointer = const void*;
    using value_type = T;
    using size_type = size_t;
    using difference_type = std::ptrdiff_t;

    using is_always_equal = std::true_type;

    template<typename U>
    struct rebind {
        using other = EspCapsAllocator<CAPS, U>;
    };

    EspCapsAllocator() = default;
    template<typename U>
    EspCapsAllocator(const EspCapsAllocator<CAPS, U>&) noexcept {}

    pointer allocate(size_type size) {
        auto ptr = heap_caps_malloc(size * sizeof(T), CAPS);
        if (!ptr) {
            throw std::bad_alloc();
        }
        return static_cast<pointer>(ptr);
    }

    void deallocate(pointer ptr, size_type size) noexcept {
        heap_caps_free(ptr);
    }

    template<typename U, typename... Args>
    void construct(U* xp, Args&&... args) {
        std::construct_at(xp, std::forward<Args>(args)...);
    }

    template<typename U>
    void destroy(U* xp) {
        std::destroy_at(xp);
    }
};

template<caps_t CAPS, typename T>
struct EspCapsDeleter {
    void operator()(T* ptr) const {
        if (ptr) {
            std::destroy_at(ptr);
            heap_caps_free(ptr);
        }
    }
};
