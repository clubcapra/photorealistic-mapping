// Fast merger for the two MID360 lidar clouds.
//
// Both units publish at 10 Hz in the same frame with an identical point
// layout (the driver applies each unit's extrinsics), so merging is just a
// byte-concatenation of the two data buffers — no per-point work. This C++
// node holds 10 Hz under load where the Python equivalent collapsed to ~3 Hz
// (GIL + per-message Python (de)serialization of ~1 MB clouds).
//
// The two streams are paired by timestamp (ApproximateTime), and the merged
// cloud keeps the lidar capture stamp (the later of the pair) rather than
// wall-clock-now, so the lidar timeline stays aligned with the cameras/odom.
//
// Unix socket forwarding:
//   After merging, each frame is forwarded to a Unix domain socket
//   (/tmp/livox_merged.sock by default, configurable via "uds_path" param).
//   Protocol: a compact 32-byte header struct followed by raw point bytes.
//   The sender runs on a dedicated thread with a single-slot drop queue so
//   the ROS executor is never blocked by a slow Rust consumer.
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using PointCloud2 = sensor_msgs::msg::PointCloud2;

// ---------------------------------------------------------------------------
// Wire format (little-endian throughout)
// ---------------------------------------------------------------------------
// Every frame on the socket is:
//   [FrameHeader]  (32 bytes, fixed)
//   [point bytes]  (frame_header.data_bytes bytes)
//
// The Rust side reads 32 bytes, then reads data_bytes more — done.
// ---------------------------------------------------------------------------
#pragma pack(push, 1)
struct FrameHeader {
    uint32_t magic;        // 0x4C4F5856 ("LOXV") — sanity check
    uint32_t data_bytes;   // byte length of the point buffer that follows
    uint32_t width;        // total point count  (height is always 1)
    uint32_t point_step;   // bytes per point
    int64_t  stamp_ns;     // header stamp in nanoseconds
    uint32_t seq;          // monotonic frame counter (wraps at 2^32)
    uint32_t _pad;         // reserved, always 0
};
#pragma pack(pop)
static_assert(sizeof(FrameHeader) == 32, "FrameHeader must be exactly 32 bytes");

constexpr uint32_t FRAME_MAGIC = 0x4C4F5856u;

// ---------------------------------------------------------------------------
// UnixSender — owns the socket fd and the sender thread
// ---------------------------------------------------------------------------
class UnixSender {
public:
    struct Frame {
        FrameHeader hdr;
        std::vector<uint8_t> data;
    };

    explicit UnixSender(const std::string & path, rclcpp::Logger logger)
    : path_(path), logger_(logger)
    {
        sender_thread_ = std::thread(&UnixSender::run, this);
    }

    ~UnixSender() {
        {
            std::lock_guard<std::mutex> lk(mu_);
            stop_ = true;
        }
        cv_.notify_one();
        if (sender_thread_.joinable()) sender_thread_.join();
        if (fd_ >= 0) ::close(fd_);
    }

    // Called from the ROS callback thread.
    // Drops the frame if the sender is still busy (single-slot queue).
    void enqueue(Frame frame) {
        {
            std::lock_guard<std::mutex> lk(mu_);
            if (pending_) {
                ++dropped_;
                if (dropped_ % 10 == 1) {
                    RCLCPP_WARN(logger_,
                        "[uds] sender busy — dropped %u frame(s) so far", dropped_);
                }
                return;
            }
            pending_ = std::move(frame);
        }
        cv_.notify_one();
    }

private:
    void run() {
        while (true) {
            std::optional<Frame> frame;
            {
                std::unique_lock<std::mutex> lk(mu_);
                cv_.wait(lk, [this]{ return pending_.has_value() || stop_; });
                if (stop_ && !pending_) break;
                frame = std::move(pending_);
                pending_.reset();
            }
            if (frame) send_frame(*frame);
        }
    }

    // Attempt (re)connect if not connected, then write the frame.
    void send_frame(const Frame & f) {
        if (fd_ < 0 && !try_connect()) return;  // no consumer yet — silently drop

        // Write header then data in two calls; writev would be cleaner but
        // this avoids a temporary iovec and is fine at 10 Hz.
        if (!write_all(reinterpret_cast<const uint8_t*>(&f.hdr), sizeof(f.hdr)) ||
            !write_all(f.data.data(), f.data.size()))
        {
            RCLCPP_WARN(logger_, "[uds] write failed (fd=%d), resetting connection", fd_);
            ::close(fd_);
            fd_ = -1;
        }
    }

    bool try_connect() {
        int sock = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if (sock < 0) {
            RCLCPP_ERROR(logger_, "[uds] socket() failed: %s", strerror(errno));
            return false;
        }
        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::strncpy(addr.sun_path, path_.c_str(), sizeof(addr.sun_path) - 1);

        if (::connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            ::close(sock);
            // Expected when Rust app isn't running yet — log only occasionally.
            if (++connect_attempts_ == 1 || connect_attempts_ % 50 == 0) {
                RCLCPP_INFO(logger_,
                    "[uds] waiting for consumer on %s (attempt %u)",
                    path_.c_str(), connect_attempts_);
            }
            return false;
        }
        fd_ = sock;
        connect_attempts_ = 0;
        RCLCPP_INFO(logger_, "[uds] connected to %s", path_.c_str());
        return true;
    }

    bool write_all(const uint8_t * buf, size_t len) {
        size_t written = 0;
        while (written < len) {
            ssize_t n = ::write(fd_, buf + written, len - written);
            if (n <= 0) return false;
            written += static_cast<size_t>(n);
        }
        return true;
    }

    const std::string path_;
    rclcpp::Logger logger_;

    int fd_{-1};
    uint32_t connect_attempts_{0};

    std::mutex mu_;
    std::condition_variable cv_;
    std::optional<Frame> pending_;
    bool stop_{false};
    uint32_t dropped_{0};

    std::thread sender_thread_;
};

// ---------------------------------------------------------------------------
// LidarMerger node
// ---------------------------------------------------------------------------
class LidarMerger : public rclcpp::Node
{
public:
    LidarMerger()
    : Node("lidar_merger")
    {
        topic_1_ = declare_parameter<std::string>("topic_1", "/livox/lidar_192_168_2_41");
        topic_2_ = declare_parameter<std::string>("topic_2", "/livox/lidar_192_168_2_40");

        const auto output_topic  = declare_parameter<std::string>("output_topic",  "/livox/lidar");
        output_frame_            = declare_parameter<std::string>("output_frame",   "livox_frame");
        const auto uds_path      = declare_parameter<std::string>("uds_path",       "/tmp/livox_merged.sock");
        uds_enabled_             = declare_parameter<bool>       ("uds_enabled",    true);

        const rclcpp::QoS qos(10);
        pub_ = create_publisher<PointCloud2>(output_topic, qos);

        sub_1_.subscribe(this, topic_1_, qos.get_rmw_qos_profile());
        sub_2_.subscribe(this, topic_2_, qos.get_rmw_qos_profile());
        sync_ = std::make_shared<Sync>(SyncPolicy(20), sub_1_, sub_2_);
        sync_->registerCallback(
            std::bind(&LidarMerger::callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        if (uds_enabled_) {
            sender_ = std::make_unique<UnixSender>(uds_path, get_logger());
            RCLCPP_INFO(get_logger(), "UDS forwarding enabled -> %s", uds_path.c_str());
        }

        RCLCPP_INFO(get_logger(), "Merging %s + %s -> %s",
            topic_1_.c_str(), topic_2_.c_str(), output_topic.c_str());
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2>;
    using Sync       = message_filters::Synchronizer<SyncPolicy>;

    void callback(
        const PointCloud2::ConstSharedPtr & c1,
        const PointCloud2::ConstSharedPtr & c2)
    {
        if (c1->point_step != c2->point_step || c1->fields.size() != c2->fields.size()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Lidar point layouts differ; skipping this frame.");
            return;
        }

        auto out = std::make_unique<PointCloud2>();

        out->header = (rclcpp::Time(c1->header.stamp) >= rclcpp::Time(c2->header.stamp))
            ? c1->header : c2->header;
        out->header.frame_id = output_frame_;
        out->height      = 1;
        out->fields      = c1->fields;
        out->is_bigendian = c1->is_bigendian;
        out->point_step  = c1->point_step;
        out->is_dense    = c1->is_dense && c2->is_dense;
        out->width       = c1->width + c2->width;
        out->row_step    = out->point_step * out->width;
        out->data.resize(c1->data.size() + c2->data.size());
        std::memcpy(out->data.data(),                c1->data.data(), c1->data.size());
        std::memcpy(out->data.data() + c1->data.size(), c2->data.data(), c2->data.size());

        // Forward to Rust consumer before moving out (zero-copy: we copy into
        // the Frame's own buffer so the ROS message can be released immediately).
        if (uds_enabled_ && sender_) {
            UnixSender::Frame f;

            const rclcpp::Time stamp(out->header.stamp);
            f.hdr = FrameHeader{
                .magic      = FRAME_MAGIC,
                .data_bytes = static_cast<uint32_t>(out->data.size()),
                .width      = out->width,
                .point_step = out->point_step,
                .stamp_ns   = stamp.nanoseconds(),
                .seq        = frame_seq_++,
                ._pad       = 0,
            };
            f.data = out->data;   // copy — sender thread outlives this callback

            sender_->enqueue(std::move(f));
        }

        pub_->publish(std::move(out));
    }

    std::string topic_1_, topic_2_, output_frame_;
    bool uds_enabled_{true};
    uint32_t frame_seq_{0};

    message_filters::Subscriber<PointCloud2> sub_1_, sub_2_;
    std::shared_ptr<Sync> sync_;
    rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
    std::unique_ptr<UnixSender> sender_;
};

// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarMerger>());
    rclcpp::shutdown();
    return 0;
}