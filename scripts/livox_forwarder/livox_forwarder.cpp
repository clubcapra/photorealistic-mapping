// livox_forwarder.cpp
//
// Standalone UDP proxy for Livox MID360 point + IMU data.
// Runs on the Jetson as a user systemd service, independent of ROS.
//
// The MID360 sends UDP datagrams to the Jetson on the ports configured
// in MID360_config.json. This process binds those ports and replicates
// every datagram verbatim to two destinations:
//
//   1. 127.0.0.1:<same port>  →  livox_ros_driver2 receives normally
//   2. <PI_IP>:<PI_PORT>      →  Pi Rust app receives raw Livox packets
//
// No parsing is done — every byte arrives on the Pi exactly as the
// lidar sent it, including Livox SDK2 headers, timestamps, and flags.
//
// Ports forwarded (per lidar, configure below):
//   point_data_port  56301 / 56302
//   imu_data_port    56401 / 56402
//
// push_msg (56200/56201) and log_data (56500/56501) are omitted by
// default — uncomment them in kPorts if needed.
// cmd_data (56100) is NEVER proxied: the ROS driver must keep a direct
// connection to the lidars for handshake and configuration.
//
// Usage:
//   ./livox_forwarder <pi_ip> <pi_port>
//
// Example:
//   ./livox_forwarder 192.168.1.50 5600
//
// The ROS driver config must point host_net_info.point_data_ip and
// imu_data_ip at 127.0.0.1 so it receives from this forwarder.
// The lidar config (MID360_config.json) keeps the Jetson's real IP.

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <string>
#include <vector>

// ── Port configuration ────────────────────────────────────────────────────────
//
// listen_port : port the lidar sends to (bound by this process)
// local_port  : port the ROS driver listens on (forwarded to 127.0.0.1)
// label       : human-readable name for logs
//
// Both ports are the same in a standard setup. They can differ if you
// need to run the ROS driver on non-standard ports.

struct PortEntry {
    uint16_t    listen_port;
    uint16_t    local_port;
    const char* label;
};

static const PortEntry kPorts[] = {
    // ── lidar .41 ─────────────────────────────────────────────────────────
    { 56301, 56301, "lidar.41 points" },
    { 56401, 56401, "lidar.41 imu"    },
    // { 56201, 56201, "lidar.41 push_msg" },  // uncomment if needed
    // { 56501, 56501, "lidar.41 log"       },

    // ── lidar .40 ─────────────────────────────────────────────────────────
    { 56302, 56302, "lidar.40 points" },
    { 56402, 56402, "lidar.40 imu"    },
    // { 56202, 56202, "lidar.40 push_msg" },
    // { 56502, 56502, "lidar.40 log"       },
};
static const int kNumPorts = static_cast<int>(sizeof(kPorts) / sizeof(kPorts[0]));

// ── Constants ─────────────────────────────────────────────────────────────────

static const int kMaxEvents = 16;
static const int kBufSize   = 65536;

// ── Globals ───────────────────────────────────────────────────────────────────

static std::atomic<bool> g_running{true};

static void handle_signal(int) {
    g_running = false;
}

// ── Per-port state ────────────────────────────────────────────────────────────

struct Forwarder {
    int         recv_fd{-1};    // binds listen_port, receives from lidar
    int         send_fd{-1};    // sends to pi_dest and local_dest
    sockaddr_in pi_dest{};      // Pi
    sockaddr_in local_dest{};   // localhost → ROS driver
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

    // Receiving socket
    f.recv_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (f.recv_fd < 0) {
        fprintf(stderr, "[%s] socket(recv): %s\n", port.label, strerror(errno));
        return false;
    }

    int reuse = 1;
    ::setsockopt(f.recv_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in bind_addr{};
    bind_addr.sin_family      = AF_INET;
    bind_addr.sin_port        = htons(port.listen_port);
    bind_addr.sin_addr.s_addr = INADDR_ANY;

    if (::bind(f.recv_fd,
               reinterpret_cast<sockaddr*>(&bind_addr),
               sizeof(bind_addr)) < 0) {
        fprintf(stderr, "[%s] bind(:%u): %s\n",
                port.label, port.listen_port, strerror(errno));
        return false;
    }

    // Sending socket
    f.send_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (f.send_fd < 0) {
        fprintf(stderr, "[%s] socket(send): %s\n", port.label, strerror(errno));
        return false;
    }

    // Pi destination — all channels land on the same Pi port so the Rust
    // app can demux by Livox packet header (data_type field)
    f.pi_dest.sin_family = AF_INET;
    f.pi_dest.sin_port   = htons(pi_port);
    if (::inet_pton(AF_INET, pi_ip.c_str(), &f.pi_dest.sin_addr) != 1) {
        fprintf(stderr, "invalid Pi IP: %s\n", pi_ip.c_str());
        return false;
    }

    // Localhost destination for ROS driver
    f.local_dest.sin_family      = AF_INET;
    f.local_dest.sin_port        = htons(port.local_port);
    f.local_dest.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    printf("[fwd] %-22s  :%u  ->  localhost:%-5u  +  %s:%u\n",
           port.label, port.listen_port, port.local_port,
           pi_ip.c_str(), pi_port);
    return true;
}

static void forward_one(Forwarder& f) {
    sockaddr_in src{};
    socklen_t   src_len = sizeof(src);

    ssize_t n = ::recvfrom(f.recv_fd, f.buf, kBufSize, 0,
                           reinterpret_cast<sockaddr*>(&src), &src_len);
    if (n <= 0) return;

    ::sendto(f.send_fd, f.buf, static_cast<size_t>(n), 0,
             reinterpret_cast<const sockaddr*>(&f.local_dest),
             sizeof(f.local_dest));

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
            "Example: %s 192.168.1.50 5600\n",
            argv[0], argv[0]);
        return 1;
    }

    const std::string pi_ip   = argv[1];
    const uint16_t    pi_port = static_cast<uint16_t>(std::stoi(argv[2]));

    signal(SIGINT,  handle_signal);
    signal(SIGTERM, handle_signal);

    // Init all forwarders
    std::vector<Forwarder> fwds(kNumPorts);
    for (int i = 0; i < kNumPorts; ++i) {
        if (!init_forwarder(fwds[i], kPorts[i], pi_ip, pi_port)) {
            return 1;
        }
    }

    // epoll — single thread handles all ports
    int epfd = ::epoll_create1(0);
    if (epfd < 0) { perror("epoll_create1"); return 1; }

    for (int i = 0; i < kNumPorts; ++i) {
        epoll_event ev{};
        ev.events   = EPOLLIN;
        ev.data.u32 = static_cast<uint32_t>(i);
        ::epoll_ctl(epfd, EPOLL_CTL_ADD, fwds[i].recv_fd, &ev);
    }

    printf("[fwd] running — %d channels active — Ctrl-C to stop\n", kNumPorts);

    // Stats printout every 10 seconds
    uint64_t last_stats = 0;
    epoll_event events[kMaxEvents];

    while (g_running) {
        int n = ::epoll_wait(epfd, events, kMaxEvents, 200);
        for (int i = 0; i < n; ++i) {
            if (events[i].events & EPOLLIN)
                forward_one(fwds[events[i].data.u32]);
        }

        // Periodic stats to stdout (visible via journalctl)
        uint64_t now = static_cast<uint64_t>(time(nullptr));
        if (now - last_stats >= 10) {
            last_stats = now;
            for (auto& f : fwds)
                printf("[fwd] %-22s  %lu pkts forwarded\n",
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