#pragma once

#include <jac/machine/class.h>
#include <jac/machine/functionFactory.h>
#include <jac/machine/machine.h>
#include <jac/machine/values.h>

#include <atomic>
#include <cerrno>
#include <condition_variable>
#include <cstring>
#include <functional>
#include <list>
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "../platform/espWifi.h"


template<class UdpFeature>
class UdpSocket {
public:
    struct RxInfo {
        std::vector<uint8_t> data;
        std::string address;
        uint16_t port;
    };

private:
    std::string _address;
    uint16_t _port;
    std::function<void(void)> _onError;
    std::function<void(uint32_t)> _onReadable;  // uint32_t: number of datagrams available
    UdpFeature* _feature;

    int _sockfd{-1};
    std::mutex _readMutex;
    std::list<RxInfo> _rxQueue;

    void error() {
        if (_onError) _onError();
    }

public:
    UdpSocket(UdpFeature* feature, std::string address, uint16_t port, std::function<void(void)> onError, std::function<void(uint32_t)> onReadable):
        _address(std::move(address)),
        _port(port),
        _onError(std::move(onError)),
        _onReadable(std::move(onReadable)),
        _feature(feature)
    {
        _sockfd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (_sockfd < 0) {
            error();
            return;
        }

        int one = 1;
        setsockopt(_sockfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(_port);
        if (_address.empty()) {
            addr.sin_addr.s_addr = INADDR_ANY;
        }
        else if (inet_pton(AF_INET, _address.c_str(), &addr.sin_addr) != 1) {
            ::close(_sockfd);
            _sockfd = -1;
            error();
            return;
        }

        if (::bind(_sockfd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {  // NOLINT
            ::close(_sockfd);
            _sockfd = -1;
            error();
            return;
        }

        bool res = _feature->addSelectFd(_sockfd, {
            +[](void* t, std::vector<uint8_t> data, std::string addr, uint16_t port) {
                UdpSocket& self = *static_cast<UdpSocket*>(t);

                if (data.empty()) {
                    // socket closed
                    self.close();
                    self.error();
                    return;
                }

                std::lock_guard<std::mutex> lk(self._readMutex);
                self._rxQueue.push_back({ std::move(data), std::move(addr), port });

                if (self._onReadable) {
                    self._feature->scheduleEvent([&self, count = self._rxQueue.size()]() {
                        if (self._onReadable) {
                            self._onReadable(static_cast<uint32_t>(count));
                        }
                    });
                }
            },
            +[](void* t) {
                UdpSocket& self = *static_cast<UdpSocket*>(t);
                self.close();
            },
            this
        });

        if (!res) {
            ::close(_sockfd);
            _sockfd = -1;
            error();
            return;
        }
    }

    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    ~UdpSocket() {
        close();
    }

    RxInfo read() {
        std::lock_guard<std::mutex> lk(_readMutex);
        if (_rxQueue.empty()) {
            throw jac::Exception::create(jac::Exception::Type::Error, "No data available");
        }
        auto v = std::move(_rxQueue.front());
        _rxQueue.pop_front();
        return v;
    }

    void write(std::span<const uint8_t> data, std::string destAddress, uint16_t destPort) {
        if (_sockfd < 0) {
            throw jac::Exception::create(jac::Exception::Type::Error, "Socket is closed");
        }

        sockaddr_in dest{};
        dest.sin_family = AF_INET;
        dest.sin_port = htons(destPort);
        if (destAddress.empty()) {
            dest.sin_addr.s_addr = INADDR_LOOPBACK;
        }
        else if (inet_pton(AF_INET, destAddress.c_str(), &dest.sin_addr) != 1) {
            throw jac::Exception::create(jac::Exception::Type::Error, "Invalid destination address");
        }

        ssize_t sent = ::sendto(_sockfd, data.data(), static_cast<int>(data.size()), 0,
                                reinterpret_cast<sockaddr*>(&dest), sizeof(dest));  // NOLINT
        if (sent < 0) {
            throw jac::Exception::create(jac::Exception::Type::Error, std::string("sendto failed: ") + std::strerror(errno));
        }
    }

    void close() {
        if (_sockfd >= 0) {
            _feature->removeSelectFd(_sockfd);
            ::close(_sockfd);
            _sockfd = -1;

            _onReadable = nullptr;
            _onError = nullptr;
        }
    }
};

template<class UdpFeature>
struct UdpSocketProtoBuilder : public jac::ProtoBuilder::Opaque<UdpSocket<UdpFeature>>, public jac::ProtoBuilder::Properties {
    static UdpSocket<UdpFeature>* constructOpaque(jac::ContextRef ctx, std::vector<jac::ValueWeak> args) {
        if (args.size() != 1) {
            throw jac::Exception::create(jac::Exception::Type::TypeError, "Expected exactly one argument");
        }

        if (EspWifiController::get().mode() == EspWifiController::DISABLED) {
            throw jac::Exception::create(jac::Exception::Type::Error, "WiFi is disabled");
        }

        auto options = args[0].to<jac::Object>();
        std::string address;
        if (options.hasProperty("address")) {
            address = options.get<std::string>("address");
        }
        uint32_t port = 0;
        if (options.hasProperty("port")) {
            port = options.get<uint32_t>("port");
            if (port > 0xFFFF) {
                throw jac::Exception::create(jac::Exception::Type::RangeError, "Port out of range");
            }
        }
        std::function<void(void)> onError;
        if (options.hasProperty("onError")) {
            auto fn = options.get<jac::Function>("onError");
            onError = [fn]() mutable {
                fn.call<void>();
            };
        }
        std::function<void(uint32_t)> onReadable;
        if (options.hasProperty("onReadable")) {
            auto fn = options.get<jac::Function>("onReadable");
            onReadable = [fn](uint32_t n) mutable {
                fn.call<void>(n);
            };
        }
        auto feature = static_cast<UdpFeature*>(JS_GetContextOpaque(ctx));

        return new UdpSocket<UdpFeature>(feature, address, static_cast<uint16_t>(port), std::move(onError), std::move(onReadable));
    }

    static void addProperties(jac::ContextRef ctx, jac::Object proto) {
        UdpSocketProtoBuilder::template addMethodMember<void(UdpSocket<UdpFeature>::*)(void), &UdpSocket<UdpFeature>::close>(ctx, proto, "close");

        jac::FunctionFactory ff(ctx);
        proto.defineProperty("read", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal) {
            auto& self = *UdpSocketProtoBuilder::getOpaque(ctx, thisVal);
            auto data = self.read();
            auto res = jac::ArrayBuffer::create(ctx, data.data);
            res.defineProperty("address", jac::Value::from(ctx, data.address));
            res.defineProperty("port", jac::Value::from(ctx, data.port));
            return res;
        }));

        proto.defineProperty("write", ff.newFunctionThisVariadic([](jac::ContextRef ctx, jac::ValueWeak thisVal, std::vector<jac::ValueWeak> args) {
            if (args.size() != 3) {
                throw jac::Exception::create(jac::Exception::Type::TypeError, "Expected three arguments");
            }

            auto& self = *UdpSocketProtoBuilder::getOpaque(ctx, thisVal);
            auto data = args[0].to<jac::ArrayBuffer>();
            std::string destAddress = args[1].to<std::string>();
            uint32_t destPort = args[2].to<uint32_t>();
            if (destPort > 0xFFFF) {
                throw jac::Exception::create(jac::Exception::Type::RangeError, "Port out of range");
            }
            self.write(data.typedView<const uint8_t>(), destAddress, static_cast<uint16_t>(destPort));
        }));
    }
};


template<class Next>
class UdpSocketFeature : public Next {
public:
    using UdpSocketClass = jac::Class<UdpSocketProtoBuilder<Next>>;

    UdpSocketFeature() {
        UdpSocketClass::init("UdpSocket");
    }

    void initialize() {
        Next::initialize();

        jac::Function ctor = UdpSocketClass::getConstructor(this->context());
        jac::Module& mod = this->newModule("udp");
        mod.addExport("UdpSocket", ctor);
    }
};
