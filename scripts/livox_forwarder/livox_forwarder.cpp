// livox_forwarder.cpp
//
// Standalone UDP forwarder for Livox MID360 point + IMU data.
// Runs on the Jetson alongside livox_ros_driver2, independent of ROS.
//
// Network layout:
//   lidar1  192.168.2.40  ──┐
//   lidar2  192.168.2.41  ──┤──▶  Jetson 192.168.2.3
//                                      │
//                                      ├──▶ livox_ros_driver2  (its own socket, untouched)
//                                      └──▶ Pi 192.168.2.2:5600  (this process)
//
// Uses SO_REUSEPORT so both this process and the ROS driver can bind
// the same ports simultaneously. The kernel delivers each incoming
// datagram to BOTH sockets independently — the ROS driver is completely
// unaware of this process and vice versa. No localhost relay needed.
//
// Ports shared with ROS driver:
//   56301  point_data  (both lidars send here)
//   56401  imu_data    (both lidars send here)
//
// cmd_data (56100) is never touched — ROS driver owns that exclusively.
//
// Usage:
//   ./livox_forwarder <pi_ip> <pi_port>
//
// Example:
//   ./livox_forwarder 192.168.2.2 5600

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
#include <string>
#include <vector>

// ── Port configuration ────────────────────────────────────────────────────────

struct PortEntry {
    uint16_t    port;
    const char* label;
};

static const PortEntry kPorts[] = {
    { 56301, "points (both lidars)" },
    { 56401, "imu    (both lidars)" },
};
static const int kNumPorts = static_cast<int>(sizeof(kPorts) / sizeof(kPorts[0]));

// ── Constants ─────────────────────────────────────────────────────────────────

static const int kMaxEvents = 16;
static const int kBufSize   = 65536;

// ── Globals ───────────────────────────────────────────────────────────────────

static std::atomic<bool> g_running{true};

static void handle_signal(int) { g_running = false; }

// ── Per-port state ────────────────────────────────────────────────────────────

struct Forwarder {
    int         recv_fd{-1};   // shares port with ROS driver via SO_REUSEPORT
    int         send_fd{-1};   // sends only to Pi
    sockaddr_in pi_dest{};
    const char* label{nullptr};
    uint8_t     buf[kBufSize]{};
    uint64_t    pkt_count{0};
};

static bool init_forwarder(Forwarder&         f,
                           const PortEntry&   port,
                           const std::string& pi_ip,
                           uint16_t           pi_port)
{
    f.label = port.label;

    f.recv_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (f.recv_fd < 0) {
        fprintf(stderr, "[%s] socket(recv): %s\n", port.label, strerror(errno));
        return false;
    }

    // SO_REUSEPORT lets both this process and livox_ros_driver2 bind the
    // same port — the kernel delivers each datagram to both independently
    int reuse = 1;
    if (::setsockopt(f.recv_fd, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse)) < 0) {
        fprintf(stderr, "[%s] setsockopt(SO_REUSEPORT): %s\n", port.label, strerror(errno));
        return false;
    }

    sockaddr_in bind_addr{};
    bind_addr.sin_family      = AF_INET;
    bind_addr.sin_port        = htons(port.port);
    bind_addr.sin_addr.s_addr = INADDR_ANY;

    if (::bind(f.recv_fd,
               reinterpret_cast<sockaddr*>(&bind_addr),
               sizeof(bind_addr)) < 0) {
        fprintf(stderr, "[%s] bind(:%u): %s\n",
                port.label, port.port, strerror(errno));
        return false;
    }

    // Dedicated send socket — only used to forward to the Pi
    f.send_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (f.send_fd < 0) {
        fprintf(stderr, "[%s] socket(send): %s\n", port.label, strerror(errno));
        return false;
    }

    f.pi_dest.sin_family = AF_INET;
    f.pi_dest.sin_port   = htons(pi_port);
    if (::inet_pton(AF_INET, pi_ip.c_str(), &f.pi_dest.sin_addr) != 1) {
        fprintf(stderr, "invalid Pi IP: %s\n", pi_ip.c_str());
        return false;
    }

    printf("[fwd] %-24s  :%u  ->  %s:%u\n",
           port.label, port.port, pi_ip.c_str(), pi_port);
    return true;
}

static void forward_one(Forwarder& f) {
    ssize_t n = ::recv(f.recv_fd, f.buf, kBufSize, 0);
    if (n <= 0) return;

    // Forward verbatim to Pi only — ROS driver handles its own copy
    ::sendto(f.send_fd, f.buf, static_cast<size_t>(n), 0,
             reinterpret_cast<const sockaddr*>(&f.pi_dest),
             sizeof(f.pi_dest));

    ++f.pkt_count;
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr,
            "Usage: %s <pi_ip> <pi_port>\n"
            "Example: %s 192.168.2.2 5600\n",
            argv[0], argv[0]);
        return 1;
    }

    const std::string pi_ip   = argv[1];
    const uint16_t    pi_port = static_cast<uint16_t>(std::stoi(argv[2]));

    signal(SIGINT,  handle_signal);
    signal(SIGTERM, handle_signal);

    std::vector<Forwarder> fwds(kNumPorts);
    for (int i = 0; i < kNumPorts; ++i) {
        if (!init_forwarder(fwds[i], kPorts[i], pi_ip, pi_port)) {
            return 1;
        }
    }

    int epfd = ::epoll_create1(0);
    if (epfd < 0) { perror("epoll_create1"); return 1; }

    for (int i = 0; i < kNumPorts; ++i) {
        epoll_event ev{};
        ev.events   = EPOLLIN;
        ev.data.u32 = static_cast<uint32_t>(i);
        ::epoll_ctl(epfd, EPOLL_CTL_ADD, fwds[i].recv_fd, &ev);
    }

    printf("[fwd] running — forwarding to Pi %s:%u — Ctrl-C to stop\n",
           pi_ip.c_str(), pi_port);

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
                printf("[fwd] %-24s  %lu pkts forwarded\n",
                       f.label, f.pkt_count);
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