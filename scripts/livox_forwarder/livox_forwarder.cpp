// livox_forwarder.cpp
//
// Standalone UDP forwarder for Livox MID360 point + IMU data.
// Runs on the Jetson alongside livox_ros_driver2, independent of ROS.
//
// Uses SO_REUSEPORT so both this process and the ROS driver bind the
// same ports simultaneously. The kernel delivers each datagram to both
// independently — the ROS driver is completely unaware of this process.
//
// Build:
//   g++ -O2 -o livox_forwarder livox_forwarder.cpp
//
// Run:
//   ./livox_forwarder

// ═════════════════════════════════════════════════════════════════════════════
// CONFIGURATION — edit this section only
// ═════════════════════════════════════════════════════════════════════════════

#define DEST_IP          "192.168.2.2"  // Pi IP

// Each entry: { listen_port, dest_port }
// listen_port : port the lidars send to on the Jetson (shared with ROS driver)
// dest_port   : port the Pi Rust app listens on
static const struct { uint16_t listen; uint16_t dest; const char* label; } kPorts[] = {
    { 56301, 56301, "points" },  // point data  → Pi :56301
    { 56401, 56401, "imu"    },  // IMU data    → Pi :56401
};

// ═════════════════════════════════════════════════════════════════════════════
// END OF CONFIGURATION
// ═════════════════════════════════════════════════════════════════════════════

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <atomic>
#include <vector>

static const int kNumPorts  = static_cast<int>(sizeof(kPorts) / sizeof(kPorts[0]));
static const int kMaxEvents = 16;
static const int kBufSize   = 65536;

static std::atomic<bool> g_running{true};
static void handle_signal(int) { g_running = false; }

struct Forwarder {
    int         recv_fd{-1};
    int         send_fd{-1};
    sockaddr_in dest{};
    uint16_t    listen_port{};
    uint16_t    dest_port{};
    const char* label{nullptr};
    uint8_t     buf[kBufSize]{};
    uint64_t    pkt_count{0};
};

static bool init_forwarder(Forwarder& f, uint16_t listen_port, uint16_t dest_port, const char* label) {
    f.listen_port = listen_port;
    f.dest_port   = dest_port;
    f.label       = label;

    f.recv_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (f.recv_fd < 0) {
        fprintf(stderr, "[%s] socket(recv): %s\n", label, strerror(errno));
        return false;
    }

    int reuse = 1;
    if (::setsockopt(f.recv_fd, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse)) < 0) {
        fprintf(stderr, "[%s] SO_REUSEPORT: %s\n", label, strerror(errno));
        return false;
    }

    sockaddr_in bind_addr{};
    bind_addr.sin_family      = AF_INET;
    bind_addr.sin_port        = htons(listen_port);
    bind_addr.sin_addr.s_addr = INADDR_ANY;
    if (::bind(f.recv_fd, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) < 0) {
        fprintf(stderr, "[%s] bind(:%u): %s\n", label, listen_port, strerror(errno));
        return false;
    }

    f.send_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (f.send_fd < 0) {
        fprintf(stderr, "[%s] socket(send): %s\n", label, strerror(errno));
        return false;
    }

    f.dest.sin_family = AF_INET;
    f.dest.sin_port   = htons(dest_port);
    if (::inet_pton(AF_INET, DEST_IP, &f.dest.sin_addr) != 1) {
        fprintf(stderr, "invalid DEST_IP: %s\n", DEST_IP);
        return false;
    }

    printf("[fwd] %-8s  :%u  ->  %s:%u\n", label, listen_port, DEST_IP, dest_port);
    return true;
}

static void forward_one(Forwarder& f) {
    ssize_t n = ::recv(f.recv_fd, f.buf, kBufSize, 0);
    if (n <= 0) return;
    ::sendto(f.send_fd, f.buf, static_cast<size_t>(n), 0,
             reinterpret_cast<const sockaddr*>(&f.dest), sizeof(f.dest));
    ++f.pkt_count;
}

int main() {
    signal(SIGINT,  handle_signal);
    signal(SIGTERM, handle_signal);

    std::vector<Forwarder> fwds(kNumPorts);
    for (int i = 0; i < kNumPorts; ++i) {
        if (!init_forwarder(fwds[i], kPorts[i].listen, kPorts[i].dest, kPorts[i].label))
            return 1;
    }

    int epfd = ::epoll_create1(0);
    if (epfd < 0) { perror("epoll_create1"); return 1; }

    for (int i = 0; i < kNumPorts; ++i) {
        epoll_event ev{};
        ev.events   = EPOLLIN;
        ev.data.u32 = static_cast<uint32_t>(i);
        ::epoll_ctl(epfd, EPOLL_CTL_ADD, fwds[i].recv_fd, &ev);
    }

    printf("[fwd] running — %d channel(s) — Ctrl-C to stop\n", kNumPorts);

    uint64_t    last_stats = 0;
    epoll_event events[kMaxEvents];

    while (g_running) {
        int n = ::epoll_wait(epfd, events, kMaxEvents, 200);
        for (int i = 0; i < n; ++i) {
            if (events[i].events & EPOLLIN)
                forward_one(fwds[events[i].data.u32]);
        }
        uint64_t now = static_cast<uint64_t>(time(nullptr));
        if (now - last_stats >= 10) {
            last_stats = now;
            for (auto& f : fwds)
                printf("[fwd] %-8s  :%u -> %s:%u  |  %lu pkts\n",
                       f.label, f.listen_port, DEST_IP, f.dest_port, f.pkt_count);
        }
    }

    ::close(epfd);
    for (auto& f : fwds) {
        if (f.recv_fd >= 0) ::close(f.recv_fd);
        if (f.send_fd >= 0) ::close(f.send_fd);
    }
    printf("[fwd] stopped\n");
    return 0;
}