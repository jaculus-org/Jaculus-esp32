#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include <chrono>
#include <functional>
#include <optional>
#include <tuple>
#include <variant>

template<class Next>
class FreeRTOSEventQueueFeature : public Next {
public:
    using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;

    struct Event {
        using EvStdF = std::function<void()>*;
        using EvFreeF = std::tuple<void(*)(void*), void*>;
        using EvTmrStdF = std::tuple<std::function<void(TimePoint)>*, uint64_t>;
        using EvTmrFreeF = std::tuple<void(*)(void*, TimePoint), void*, uint64_t>;


        std::variant<
            std::monostate,
            EvStdF,
            EvFreeF,
            EvTmrStdF,
            EvTmrFreeF
        > event = std::monostate();

        Event() : event(std::monostate()) {}

        Event(std::function<void()> func) : event(new std::function<void()>(std::move(func))) {}
        Event(void(*func)(void*), void* arg) : event(std::make_tuple(func, arg)) {}
        Event(std::function<void(TimePoint)> func):
            event(std::make_tuple(
                new std::function<void(TimePoint)>(std::move(func)),
                // TODO: use efficient time source, maybe add time argument?
                std::chrono::steady_clock::now().time_since_epoch().count()
            ))
        {}
        Event(void(*func)(void*, TimePoint), void* arg):
            event(std::make_tuple(
                func,
                arg,
                std::chrono::steady_clock::now().time_since_epoch().count()
            ))
        {}

        Event& operator=(const Event& other) = delete;
        Event(const Event& other) = delete;
        Event& operator=(Event&& other) {
            std::swap(event, other.event);
            return *this;
        }
        Event(Event&& other) {
            *this = std::move(other);
        }

        ~Event() {
            std::visit([](auto& arg) {
                using T = std::decay_t<decltype(arg)>;
                if constexpr (std::is_same_v<T, EvStdF>) {
                    delete arg;
                }
                else if constexpr (std::is_same_v<T, EvTmrStdF>) {
                    delete std::get<0>(arg);
                }
                else {
                    // no cleanup
                }
            }, event);
        }

        void operator()() {
            std::visit([](auto& arg) {
                using T = std::decay_t<decltype(arg)>;
                if constexpr (std::is_same_v<T, EvStdF>) {
                    if (arg) {
                        (*arg)();
                    }
                }
                else if constexpr (std::is_same_v<T, EvFreeF>) {
                    auto [func, a] = arg;
                    if (func) {
                        func(a);
                    }
                }
                else if constexpr (std::is_same_v<T, EvTmrStdF>) {
                    auto [func, time] = arg;
                    if (func) {
                        (*func)(std::chrono::steady_clock::time_point(std::chrono::steady_clock::duration(time)));
                    }
                }
                else if constexpr (std::is_same_v<T, EvTmrFreeF>) {
                    auto [func, a, time] = arg;
                    if (func) {
                        func(a, std::chrono::steady_clock::time_point(std::chrono::steady_clock::duration(time)));
                    }
                }
                else {
                    // empty event
                }
            }, event);
        }

        operator bool() const {
            return !std::holds_alternative<std::monostate>(event);
        }

        void release() {
            event = std::monostate();
        }
    };
private:
    QueueHandle_t _eventQueue;

    Event dequeue(bool wait) {
        Event e;
        if (xQueueReceive(_eventQueue, &e, wait ? portMAX_DELAY : 0)) {
            return e;
        }
        else {
            return Event();
        }
    }

    void drain() {
        while(true) {
            Event e;
            if(xQueueReceive(_eventQueue, &e, 0) == pdFALSE) {
                break;
            }
        }
    }

    void _scheduleImpl(auto&&... args) {
        Event e(std::forward<decltype(args)>(args)...);
        auto res = xQueueSend(_eventQueue, &e, portMAX_DELAY);
        if (res != pdPASS) {
            // TODO: handle error
            return;
        }
        e.release();
    }

    void _scheduleIsrImpl(auto&&... args) {
        Event e(std::forward<decltype(args)>(args)...);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        auto res = xQueueSendFromISR(_eventQueue, &e, &xHigherPriorityTaskWoken);
        if (res != pdPASS) {
            // TODO: handle error
            return;
        }
        e.release();
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
public:

    FreeRTOSEventQueueFeature() {
        _eventQueue = xQueueCreate(64, sizeof(Event));
    }

    /**
     * @brief Check the event queue and return the first event
     * @param wait Wait for event if no event is available
     * @return Event or std::nullopt if no event is available
     */
    std::optional<Event> getEvent(bool wait) {
        auto e(dequeue(wait));
        if (!e) {
            return std::nullopt;
        }
        return e;
    }

    /**
     * @brief Schedule an event to be run
     * @param func Function to be run
     */
    void scheduleEvent(std::function<void()> func) { _scheduleImpl(std::move(func)); }

    /**
     * @brief Schedule an event to be run
     * @param func Function to be run
     * @param arg Argument to be passed to function
     */
    void scheduleEvent(void(*func)(void*), void* arg) { _scheduleImpl(func, arg); }

    /**
     * @brief Schedule an event to be run
     * @param func Function to be run
     */
    void scheduleEvent(std::function<void(TimePoint)> func) { _scheduleImpl(std::move(func)); }

    /**
     * @brief Schedule an event to be run
     * @param func Function to be run
     * @param arg Argument to be passed to function
     */
    void scheduleEvent(void(*func)(void*, TimePoint), void* arg) { _scheduleImpl(func, arg); }

    /**
     * @brief Schedule an event to be run from ISR
     * @param func Function to be run
     * @param arg Argument to be passed to function
     */
    void scheduleEventISR(void(*func)(void*), void* arg) { _scheduleIsrImpl(func, arg); }

    /**
     * @brief Schedule an event to be run from ISR
     * @param func Function to be run
     * @param arg Argument to be passed to function
     */
    void scheduleEventISR(void(*func)(void*, TimePoint), void* arg) { _scheduleIsrImpl(func, arg); }

    /**
     * @brief Wake up event loop if it is waiting for events
     */
    void notifyEventLoop() {
        Event e;
        xQueueSend(_eventQueue, &e, 0);
    }

    ~FreeRTOSEventQueueFeature() {
        drain();
        vQueueDelete(_eventQueue);
    }
};
