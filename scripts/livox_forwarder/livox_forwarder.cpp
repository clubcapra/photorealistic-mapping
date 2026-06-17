#include <cerrno>
#include <cstdint>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <atomic>
#include <vector>

// =============================================================================
// CONFIGURATION
// =============================================================================
#define DEST_IP "192.168.2.2"  // Pi IP

static const struct { uint16_t port; const char* label; } kPorts[] = {
    { 56301, "points" },
    { 56401, "imu"    },
};
// =============================================================================

static const int kNumPorts  = static_cast<int>(sizeof(kPorts) / sizeof(kPorts[0]));
static const int kMaxEvents = 16;
static const int kBufSize   = 65536;
static std::atomic<bool> g_running{true};
static void handle_signal(int) { g_running = false; }

static uint16_t checksum(const void* data, int len) {
    const uint16_t* ptr = reinterpret_cast<const uint16_t*>(data);
    uint32_t sum = 0;
    while (len > 1) { sum += *ptr++; len -= 2; }
    if (len) sum += *reinterpret_cast<const uint8_t*>(ptr);
    while (sum >> 16) sum = (sum & 0xffff) + (sum >> 16);
    return static_cast<uint16_t>(~sum);
}

static uint16_t udp_checksum(const struct iphdr* iph, const struct udphdr* udph,
                             const uint8_t* payload, int payload_len) {
    struct { uint32_t src, dst; uint8_t zero, proto; uint16_t udp_len; } pseudo{};
    pseudo.src     = iph->saddr;
    pseudo.dst     = iph->daddr;
    pseudo.proto   = IPPROTO_UDP;
    pseudo.udp_len = udph->len;
    int total = sizeof(pseudo) + sizeof(struct udphdr) + payload_len;
    std::vector<uint8_t> buf(total);
    memcpy(buf.data(),                                                    &pseudo, sizeof(pseudo));
    memcpy(buf.data() + sizeof(pseudo),                                   udph,    sizeof(struct udphdr));
    memcpy(buf.data() + sizeof(pseudo) + sizeof(struct udphdr), payload, payload_len);
    return checksum(buf.data(), total);
}

struct Forwarder {
    int         recv_fd{-1};
    int         raw_fd{-1};
    sockaddr_in pi_addr{};
    uint16_t    port{};
    const char* label{nullptr};
    uint8_t     buf[kBufSize]{};
    uint64_t    pkt_count{0};
};

static bool init_forwarder(Forwarder& f, uint16_t port, const char* label) {
    f.port  = port;
    f.label = label;

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
    bind_addr.sin_port        = htons(port);
    bind_addr.sin_addr.s_addr = INADDR_ANY;
    if (::bind(f.recv_fd, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) < 0) {
        fprintf(stderr, "[%s] bind(:%u): %s\n", label, port, strerror(errno));
        return false;
    }

    f.raw_fd = ::socket(AF_INET, SOCK_RAW, IPPROTO_RAW);
    if (f.raw_fd < 0) {
        fprintf(stderr, "[%s] socket(raw): %s\n  -> run: sudo setcap cap_net_raw+ep ./livox_forwarder\n",
                label, strerror(errno));
        return false;
    }

    int on = 1;
    if (::setsockopt(f.raw_fd, IPPROTO_IP, IP_HDRINCL, &on, sizeof(on)) < 0) {
        fprintf(stderr, "[%s] IP_HDRINCL: %s\n", label, strerror(errno));
        return false;
    }

    f.pi_addr.sin_family = AF_INET;
    f.pi_addr.sin_port   = 0;
    if (::inet_pton(AF_INET, DEST_IP, &f.pi_addr.sin_addr) != 1) {
        fprintf(stderr, "invalid DEST_IP: %s\n", DEST_IP);
        return false;
    }

    printf("[fwd] %-8s  :%u  ->  %s:%u  (source IP preserved)\n", label, port, DEST_IP, port);
    return true;
}

static void forward_one(Forwarder& f) {
    sockaddr_in src{};
    socklen_t src_len = sizeof(src);
    ssize_t n = ::recvfrom(f.recv_fd, f.buf, kBufSize, 0,
                           reinterpret_cast<sockaddr*>(&src), &src_len);
    if (n <= 0) return;

    const int payload_len = static_cast<int>(n);
    const int udp_len     = sizeof(struct udphdr) + payload_len;
    const int total_len   = sizeof(struct iphdr) + udp_len;

    std::vector<uint8_t> pkt(total_len, 0);
    auto* iph        = reinterpret_cast<struct iphdr*>(pkt.data());
    auto* udph       = reinterpret_cast<struct udphdr*>(pkt.data() + sizeof(struct iphdr));
    uint8_t* payload = pkt.data() + sizeof(struct iphdr) + sizeof(struct udphdr);
    memcpy(payload, f.buf, payload_len);

    udph->source = src.sin_port;
    udph->dest   = htons(f.port);
    udph->len    = htons(static_cast<uint16_t>(udp_len));
    udph->check  = 0;

    iph->ihl      = 5;
    iph->version  = 4;
    iph->tos      = 0;
    iph->tot_len  = htons(static_cast<uint16_t>(total_len));
    iph->id       = 0;
    iph->frag_off = 0;
    iph->ttl      = 64;
    iph->protocol = IPPROTO_UDP;
    iph->check    = 0;
    iph->saddr    = src.sin_addr.s_addr;
    iph->daddr    = f.pi_addr.sin_addr.s_addr;

    iph->check  = checksum(iph, sizeof(struct iphdr));
    udph->check = udp_checksum(iph, udph, payload, payload_len);

    ::sendto(f.raw_fd, pkt.data(), pkt.size(), 0,
             reinterpret_cast<const sockaddr*>(&f.pi_addr), sizeof(f.pi_addr));
    ++f.pkt_count;
}

int main() {
    signal(SIGINT,  handle_signal);
    signal(SIGTERM, handle_signal);

    std::vector<Forwarder> fwds(kNumPorts);
    for (int i = 0; i < kNumPorts; ++i)
        if (!init_forwarder(fwds[i], kPorts[i].port, kPorts[i].label)) return 1;

    int epfd = ::epoll_create1(0);
    if (epfd < 0) { perror("epoll_create1"); return 1; }

    for (int i = 0; i < kNumPorts; ++i) {
        epoll_event ev{};
        ev.events   = EPOLLIN;
        ev.data.u32 = static_cast<uint32_t>(i);
        ::epoll_ctl(epfd, EPOLL_CTL_ADD, fwds[i].recv_fd, &ev);
    }

    printf("[fwd] running — %d channel(s) — Ctrl-C to stop\n", kNumPorts);
    uint64_t last_stats = 0;
    epoll_event events[kMaxEvents];

    while (g_running) {
        int n = ::epoll_wait(epfd, events, kMaxEvents, 200);
        for (int i = 0; i < n; ++i)
            if (events[i].events & EPOLLIN) forward_one(fwds[events[i].data.u32]);

        uint64_t now = static_cast<uint64_t>(time(nullptr));
        if (now - last_stats >= 10) {
            last_stats = now;
            for (auto& f : fwds)
                printf("[fwd] %-8s  :%u  |  %lu pkts\n", f.label, f.port, f.pkt_count);
        }
    }

    ::close(epfd);
    for (auto& f : fwds) {
        if (f.recv_fd >= 0) ::close(f.recv_fd);
        if (f.raw_fd  >= 0) ::close(f.raw_fd);
    }
    printf("[fwd] stopped\n");
    return 0;
}