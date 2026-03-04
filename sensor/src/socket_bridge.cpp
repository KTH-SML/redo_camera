#include <iostream>
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <shared_mutex>
#include <vector>
#include <thread>
#include <mutex>

#include "socket_bridge.h"

SocketBridge::SocketBridge(const std::string &ip, int port)
    : sockfd_(-1), localAddr_()
{
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0)
    {
        std::cerr << "[socket bridge] Cannot create socket." << std::endl;
        return;
    }

    int optval = 1;
    if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval)) < 0)
    {
        std::cerr << "[socket bridge] setsockopt(SO_REUSEPORT) failed." << std::endl;
        close(sockfd_);
        sockfd_ = -1;
        return;
    }

    std::memset(&localAddr_, 0, sizeof(localAddr_));
    localAddr_.sin_family = AF_INET;
    localAddr_.sin_addr.s_addr = inet_addr(ip.c_str());
    localAddr_.sin_port = htons(port);

    if (bind(sockfd_, reinterpret_cast<const sockaddr *>(&localAddr_), sizeof(localAddr_)) < 0)
    {
        std::cerr << "[socket bridge] Port bind failed." << std::endl;
        close(sockfd_);
        sockfd_ = -1;
        return;
    }
}

SocketBridge::~SocketBridge()
{
    if (sockfd_ != -1)
    {
        close(sockfd_);
    }
}

bool SocketBridge::isValid() const
{
    return sockfd_ != -1;
}

ssize_t SocketBridge::receiveData(char *buffer, const size_t bufferSize) const
{
    if (sockfd_ < 0)
    {
        return -1;
    }
    sockaddr_in serverAddr{};
    socklen_t senderLen = sizeof(serverAddr);
    return recvfrom(sockfd_, buffer, bufferSize, 0,
                    reinterpret_cast<sockaddr *>(&serverAddr), &senderLen);
}

void receive_data_loop(const SocketBridge *bridge, char *buffer, const size_t bufferSize,
                       std::shared_mutex &bufferMutex, bool &signal, bool &isRunning)
{
    isRunning = true;
    auto localBuffer = new char[bufferSize];
    while (!signal && bridge && bridge->isValid())
    {
        bridge->receiveData(localBuffer, bufferSize);
        std::lock_guard lock(bufferMutex);
        std::memcpy(buffer, localBuffer, bufferSize);
    }
    delete[] localBuffer;
    isRunning = false;
}

void TP_status_receive_loop(int fd,
                            std::atomic<bool>& stop,
                            std::atomic<bool>& isRunning,
                            std::vector<XYVV>& out_vec,
                            std::shared_mutex& mtx)
{
    isRunning = true;
    std::vector<uint8_t> buf(65536);

    while(!stop){
        sockaddr_in src{};
        socklen_t slen = sizeof(src);
        ssize_t n = ::recvfrom(fd, buf.data(), buf.size(), 0, reinterpret_cast<sockaddr*>(&src), &slen);

        if(n <=0){
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        if (static_cast<size_t>(n) < XYVV_HEADER_SIZE) {
            continue;
        }

        uint32_t count_le;
        std::memcpy(&count_le, buf.data(), 4);
        const uint32_t count = count_le;

        const size_t expected = XYVV_HEADER_SIZE + static_cast<size_t>(count) * sizeof(XYVV);
        if(static_cast<size_t>(n) != expected){
            continue;
        }
        const uint8_t* payload = buf.data() + XYVV_HEADER_SIZE;
        std::vector<XYVV> tmp(count);
        std::memcpy(tmp.data(), payload, count * sizeof(XYVV));

        {
            std::unique_lock lock(mtx);
            out_vec.swap(tmp);
        }
    }
    isRunning = false;
}

static inline uint16_t read_u16_be(const uint8_t* p){
    return (uint16_t(p[0]) << 8) | uint16_t(p[1]);
}
static inline uint32_t read_u32_be(const uint8_t* p){
    return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}
static inline uint64_t read_u64_be(const uint8_t* p){
    uint64_t v = 0;
    for(int i=0;i<8;i++) v = (v<<8) | uint64_t(p[i]);
    return v;
}

void MPC_receive_loop(int fd,
                      std::atomic<bool>& stop,
                      std::atomic<bool>& isRunning,
                      std::vector<std::pair<float,float>>& out_pts,
                      MPCPacketMeta& out_meta,
                      std::shared_mutex& mtx)
{
    isRunning = true;
    std::vector<uint8_t> buf(65536);

    uint32_t last_seq = 0;
    bool has_last = false;

    while(!stop){
        sockaddr_in src{};
        socklen_t slen = sizeof(src);
        ssize_t n = ::recvfrom(fd, buf.data(), buf.size(), 0, reinterpret_cast<sockaddr*>(&src), &slen);

        if(n <= 0){
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        if(static_cast<size_t>(n) < MPC_HEADER_SIZE) continue;

        const uint8_t* p = buf.data();

        const uint32_t magic   = read_u32_be(p + 0);
        const uint16_t version = read_u16_be(p + 4);
        const uint16_t flags   = read_u16_be(p + 6);
        const uint32_t seq     = read_u32_be(p + 8);
        const uint64_t t_ns    = read_u64_be(p + 12);
        const uint16_t count   = read_u16_be(p + 20);
        // reserved at p+22

        if(magic != MPC_MAGIC) continue;
        if(version != MPC_VERSION) continue;

        const size_t payload_bytes = size_t(count) * 2u * sizeof(float);
        const size_t expected = MPC_HEADER_SIZE + payload_bytes;
        if(static_cast<size_t>(n) != expected) continue;

        // optional: drop old seq (simple)
        if(has_last){
            if(seq == last_seq) continue;
        }
        last_seq = seq;
        has_last = true;

        const uint8_t* payload = p + MPC_HEADER_SIZE;

        std::vector<std::pair<float,float>> tmp;
        tmp.resize(count);

        // payload = [x0,y0,x1,y1,...] float32
        for(uint16_t i=0;i<count;i++){
            float xy[2];
            std::memcpy(&xy[0], payload + (size_t(i)*2u + 0u)*sizeof(float), sizeof(float));
            std::memcpy(&xy[1], payload + (size_t(i)*2u + 1u)*sizeof(float), sizeof(float));
            tmp[i] = {xy[0], xy[1]};
        }

        MPCPacketMeta meta;
        meta.seq = seq;
        meta.t_ns = t_ns;
        meta.ok = (flags & 0x0001) != 0;
        meta.count = count;

        {
            std::unique_lock lock(mtx);
            out_pts.swap(tmp);
            out_meta = meta;
        }
    }

    isRunning = false;
}