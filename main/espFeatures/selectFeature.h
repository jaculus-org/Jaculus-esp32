#pragma once

#include <jac/machine/class.h>
#include <jac/machine/functionFactory.h>
#include <jac/machine/machine.h>

#include <atomic>
#include <cerrno>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <fcntl.h>
#include <netdb.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <unistd.h>


template<class Next>
class SelectFeature : public Next {
public:
    using DataCloseCallbacks = std::tuple<void(*)(void*, std::vector<uint8_t> /*data*/, std::string /*addr*/, uint16_t /*port*/), void(*)(void*), void*>;
private:
    mutable std::mutex _mutex;
    std::condition_variable _cv;
    std::unordered_map<int, DataCloseCallbacks> _callbacks;
    std::thread _workerThread;
    std::atomic<bool> _running{false};

    void startWorker() {
        bool expected = false;
        if (_running.compare_exchange_strong(expected, true)) {
            _workerThread = std::thread(&SelectFeature::workerLoop, this);
        }
    }

    void stopWorker() {
        bool expected = true;
        if (_running.compare_exchange_strong(expected, false)) {
            _cv.notify_one();
            if (_workerThread.joinable()) {
                _workerThread.join();
            }
        }
    }

    void workerLoop() {
        static constexpr int POLL_TIMEOUT_MS = 500;
        static constexpr int BUFFER_SIZE = 2048;

        std::vector<pollfd> fds;
        std::vector<int> toRemove;
        std::vector<uint8_t> buffer;

        while (_running) {
            {
                std::unique_lock<std::mutex> lock(_mutex);
                // wait if no fds and still running
                _cv.wait(lock, [this]() { return !_callbacks.empty() || !_running; });
                if (!_running) {
                    break;
                }

                if (fds.size() != _callbacks.size()) {
                    fds.clear();
                    fds.reserve(_callbacks.size());
                    for (const auto &p : _callbacks) {
                        pollfd pfd;
                        pfd.fd = p.first;
                        pfd.events = POLLIN;
                        pfd.revents = 0;
                        fds.push_back(pfd);
                    }
                }
            }

            int ret = ::poll(fds.data(), fds.size(), POLL_TIMEOUT_MS);
            if (ret == 0) {
                continue;
            }
            else if (ret < 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }

            for (const auto &pfd : fds) {
                if (pfd.revents == 0) {
                    continue;
                }
                int fd = pfd.fd;

                DataCloseCallbacks cb;
                {
                    std::lock_guard<std::mutex> lock(_mutex);
                    auto it = _callbacks.find(fd);
                    if (it != _callbacks.end()) {
                        cb = it->second;
                    }
                    else {
                        continue;  // fd was removed
                    }
                }

                ssize_t n = -1;

                int sockType = 0;
                socklen_t optlen = sizeof(sockType);
                bool isDgram = false;
                if (getsockopt(fd, SOL_SOCKET, SO_TYPE, &sockType, &optlen) == 0) {
                    isDgram = (sockType == SOCK_DGRAM);
                }

                if (isDgram) {
                    int available = 0;
                    if (ioctl(fd, FIONREAD, &available) >= 0 && available > 0) {
                        buffer.resize(static_cast<size_t>(available));
                        buffer.shrink_to_fit();
                    }
                    else {
                        buffer.resize(BUFFER_SIZE);
                        buffer.shrink_to_fit();
                    }
                }
                else {
                    buffer.resize(BUFFER_SIZE);
                    buffer.shrink_to_fit();
                }
                std::string addr;
                uint16_t port = 0;

                sockaddr_storage sa{};
                socklen_t sa_len = sizeof(sa);
                n = ::recvfrom(fd, buffer.data(), buffer.size(), 0, reinterpret_cast<sockaddr*>(&sa), &sa_len);  // NOLINT

                if (n >= 0 && sa_len > 0) {
                    if (sa.ss_family == AF_INET) {
                        auto *s4 = reinterpret_cast<sockaddr_in*>(&sa);  // NOLINT
                        char buf[INET_ADDRSTRLEN] = {0};
                        if (inet_ntop(AF_INET, &s4->sin_addr, buf, sizeof(buf))) {
                            addr = buf;
                        }
                        port = ntohs(s4->sin_port);
                    }
                    #if LWIP_IPV6
                    else if (sa.ss_family == AF_INET6) {
                        auto *s6 = reinterpret_cast<sockaddr_in6*>(&sa);  // NOLINT
                        char buf[INET6_ADDRSTRLEN] = {0};
                        if (inet_ntop(AF_INET6, &s6->sin6_addr, buf, sizeof(buf))) {
                            addr = buf;
                        }
                        port = ntohs(s6->sin6_port);
                    }
                    #endif /* LWIP_IPV6 */
                    else {
                        addr.clear();
                        port = 0;
                    }
                }

                if (n > 0) {
                    buffer.resize(static_cast<size_t>(n));
                    auto& [dataCb, _, opaque] = cb;

                    dataCb(opaque, std::move(buffer), std::move(addr), port);
                }
                else if (n == 0) {
                    removeSelectFd(fd);
                }
                else {  // n < 0
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        continue;
                    }
                    else {
                        // treat other errors as terminal for this fd
                        removeSelectFd(fd);
                    }
                }
            }
        }

        _running = false;
    }

public:
    void initialize() {
        Next::initialize();
    }

    ~SelectFeature() {
        stopWorker();
        while (true) {
            DataCloseCallbacks cb;
            {
                std::lock_guard<std::mutex> g(_mutex);
                auto it = _callbacks.begin();
                if (it == _callbacks.end()) {
                    break;
                }
                cb = it->second;
            }
            auto& [_, closeCb, opaque] = cb;
            if (closeCb) {
                closeCb(opaque);
            }
        }
    }

    bool addSelectFd(int fd, DataCloseCallbacks cb) {
        if (fd < 0) {
            return false;
        }
        startWorker();

        {
            std::lock_guard<std::mutex> g(_mutex);
            auto it = _callbacks.find(fd);
            if (it != _callbacks.end()) return false;
            _callbacks.emplace(fd, std::move(cb));
        }
        _cv.notify_one();
        return true;
    }

    void removeSelectFd(int fd) {
        {
            std::unique_lock<std::mutex> g(_mutex);
            _callbacks.erase(fd);
        }
        _cv.notify_one();
    }
};
