#ifndef SOCKET_BRIDGE_H
#define SOCKET_BRIDGE_H

#include <shared_mutex>
#include <arpa/inet.h>
#include <atomic>
#include <vector>
#include <cstdint>
#include <netinet/in.h>
#include <sys/types.h>
#include <utility> 
#include <mutex>

static constexpr uint16_t MPC_PORT = 50052;
static constexpr uint32_t MPC_MAGIC = 0x4D504350;    // 'MPCP'
static constexpr uint16_t MPC_VERSION = 1;
static constexpr size_t   MPC_HEADER_SIZE = 24;

class SocketBridge
{
public:
    SocketBridge(const std::string& ip, int port);
    ~SocketBridge();

    [[nodiscard]] bool isValid() const;

    ssize_t receiveData(char* buffer, size_t bufferSize) const;

    void shutdownAndClose();

private:
    int sockfd_ = -1;
    sockaddr_in localAddr_{};
    mutable std::mutex fd_mtx_;
};

void receive_data_loop(const SocketBridge* bridge,
                       char* buffer,
                       size_t bufferSize,
                       std::shared_mutex& bufferMutex,
                       std::atomic_bool& stop,
                       std::atomic_bool& isRunning);

struct XYVV {
    float x;
    float y;
    float vx;
    float vy;
};

static constexpr uint16_t XYVV_PORT = 50051;
static constexpr size_t XYVV_HEADER_SIZE = 4;

void TP_status_receive_loop(int fd, std::atomic<bool>& stop, std::atomic<bool>& isRunning,
                            std::vector<XYVV>& out_vec, std::shared_mutex& mtx);

struct MPCPacketMeta {
    uint32_t seq = 0;
    uint64_t t_ns = 0;
    bool ok = false;
    uint16_t count = 0;
};

void MPC_receive_loop(int fd,
                      std::atomic<bool>& stop,
                      std::atomic<bool>& isRunning,
                      std::vector<std::pair<float,float>>& out_pts,
                      MPCPacketMeta& out_meta,
                      std::shared_mutex& mtx);


#endif // SOCKET_BRIDGE_H
