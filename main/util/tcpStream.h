#pragma once

#include <jac/device/logger.h>
#include <jac/link/stream.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <span>
#include <string>
#include <thread>

#include <sys/param.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

namespace detail {


template<typename Func>
struct Defer {
    Func func;
    bool cancelled = false;

    Defer(Func func) : func(func) {}
    ~Defer() {
        if (!cancelled) {
            func();
        }
    }

    void cancel() {
        cancelled = true;
    }
};


} // namespace detail


class TcpStream : public jac::Duplex {
    std::function<void()> _onData;
    std::thread _eventThread;
    std::atomic<bool> _stopThread = false;
    std::deque<uint8_t> _buffer;

    uint16_t _port;
    int _keepIdle;
    int _keepInterval;
    int _keepCount;

    int _listenSock;
    bool _openListen = false;
    int _clientSock;
    bool _openClient = false;
    bool _connClose = false;
public:
    TcpStream(uint16_t port = 17531, int keepIdle = 3000, int keepInterval = 500, int keepCount = 5):
            _port(port),
            _keepIdle(keepIdle),
            _keepInterval(keepInterval),
            _keepCount(keepCount)
        {
        int ip_protocol = 0;
        struct sockaddr_storage dest_addr;

        auto *dest_addr_ip4 = reinterpret_cast<sockaddr_in*>(&dest_addr);  // NOLINT
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(_port);
        ip_protocol = IPPROTO_IP;

        _listenSock = socket(AF_INET, SOCK_STREAM, ip_protocol);
        if (_listenSock < 0) {
            jac::Logger::error(std::string("Unable to create socket: errno ") + std::to_string(errno));
            return;
        }

        detail::Defer cleanup([&] {
            close(_listenSock);
        });

        int opt = 1;
        setsockopt(_listenSock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        jac::Logger::debug("Socket created");

        int err = bind(_listenSock, reinterpret_cast<sockaddr*>(&dest_addr), sizeof(dest_addr));  // NOLINT
        if (err != 0) {
            jac::Logger::error(std::string("Socket unable to bind: errno ") + std::to_string(errno));
            jac::Logger::error(std::string("IPPROTO: ") + std::to_string(AF_INET));
            return;
        }
        jac::Logger::debug(std::string("Socket bound, port ") + std::to_string(_port));

        cleanup.cancel();
        _openListen = true;
    }
    TcpStream(TcpStream&&) = delete;
    TcpStream(const TcpStream&) = delete;
    TcpStream& operator=(TcpStream&&) = delete;
    TcpStream& operator=(const TcpStream&) = delete;

    void start() {
        _eventThread = std::thread([this]() noexcept {
            detail::Defer cleanup([&] {
                _openListen = false;
                close(_listenSock);
            });

            int err = listen(_listenSock, 1);
            if (err != 0) {
                jac::Logger::error(std::string("Error occurred during listen: errno ") + std::to_string(errno));
                return;
            }

            while (!_stopThread) {
                jac::Logger::debug("Socket listening");

                struct sockaddr_storage source_addr;
                socklen_t addr_len = sizeof(source_addr);
                _clientSock = accept(_listenSock, (struct sockaddr *)&source_addr, &addr_len);
                if (_clientSock < 0) {
                    jac::Logger::debug(std::string("Unable to accept connection: errno ") + std::to_string(errno));
                    continue;
                }
                jac::Logger::debug("Got client");

                _openClient = true;
                detail::Defer clientCleanup([&] {
                    _openClient = false;
                    shutdown(_clientSock, 0);
                    close(_clientSock);
                });

                // Set tcp keepalive option
                int keepAlive = 1;
                setsockopt(_clientSock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
                setsockopt(_clientSock, IPPROTO_TCP, TCP_KEEPIDLE, &_keepIdle, sizeof(int));
                setsockopt(_clientSock, IPPROTO_TCP, TCP_KEEPINTVL, &_keepInterval, sizeof(int));
                setsockopt(_clientSock, IPPROTO_TCP, TCP_KEEPCNT, &_keepCount, sizeof(int));
                {
                    // Convert ip address to string
                    char addr_str[128];
                    if (source_addr.ss_family == PF_INET) {
                        inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
                    }
                    jac::Logger::debug(std::string("Socket accepted ip address: ") + std::string(addr_str));
                }

                std::array<char, 128> rx_buffer;
                _connClose = false;
                do {
                    int len = recv(_clientSock, &(*rx_buffer.begin()), rx_buffer.size(), 0);
                    if (len < 0) {
                        jac::Logger::error(std::string("Error occurred during receiving: errno ") + std::to_string(errno));
                    }
                    else if (len == 0) {
                        jac::Logger::debug("Connection closed");
                        _connClose = true;
                    }
                    else {
                        this->_buffer.insert(this->_buffer.end(), rx_buffer.begin(), rx_buffer.begin() + len);

                        if (len > 0) {
                            if (_onData) {
                                _onData();
                            }
                        }
                    }
                } while (!_connClose && !_stopThread);
            }
        });
    }

    bool put(uint8_t c) override {
        return write(std::span<const uint8_t>(&c, 1));
    }

    size_t write(std::span<const uint8_t> data) override {
        if (!_openListen || !_openClient) {
            return data.size();
        }

        // send() can return less bytes than supplied length.
        // Walk-around for robust implementation.
        int to_write = data.size();
        while (to_write > 0 && _openListen && _openClient) {
            int written = send(_clientSock, data.data() + (data.size() - to_write), to_write, 0);
            if (written < 0) {
                // Failed to retransmit, giving up
                _connClose = true;
                break;
            }
            to_write -= written;
        }

        return data.size();
    }

    int get() override {
        if (_buffer.empty()) {
            return -1;
        }
        uint8_t c = _buffer.front();
        _buffer.pop_front();
        return c;
    }

    size_t read(std::span<uint8_t> data) override {
        size_t len = std::min(data.size(), _buffer.size());
        std::copy_n(_buffer.begin(), len, data.begin());
        _buffer.erase(_buffer.begin(), _buffer.begin() + len);
        return len;
    }

    bool flush() override {
        return true;
    }

    void onData(std::function<void(void)> callback) override {
        _onData = callback;
    }

    ~TcpStream() override {
        _stopThread = true;
        _eventThread.join();
    }
};
