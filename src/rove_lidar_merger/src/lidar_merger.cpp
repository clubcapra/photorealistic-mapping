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

#include <cstring>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using PointCloud2 = sensor_msgs::msg::PointCloud2;

class LidarMerger : public rclcpp::Node
{
public:
  LidarMerger()
  : Node("lidar_merger")
  {
    topic_1_ = declare_parameter<std::string>("topic_1", "/livox/lidar_192_168_2_41");
    topic_2_ = declare_parameter<std::string>("topic_2", "/livox/lidar_192_168_2_40");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/livox/lidar");
    output_frame_ = declare_parameter<std::string>("output_frame", "livox_frame");

    const rclcpp::QoS qos(10);  // reliable, matches the Livox driver
    pub_ = create_publisher<PointCloud2>(output_topic, qos);

    sub_1_.subscribe(this, topic_1_, qos.get_rmw_qos_profile());
    sub_2_.subscribe(this, topic_2_, qos.get_rmw_qos_profile());
    sync_ = std::make_shared<Sync>(SyncPolicy(20), sub_1_, sub_2_);
    sync_->registerCallback(
      std::bind(&LidarMerger::callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(), "Merging %s + %s -> %s",
      topic_1_.c_str(), topic_2_.c_str(), output_topic.c_str());
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  void callback(
    const PointCloud2::ConstSharedPtr & c1,
    const PointCloud2::ConstSharedPtr & c2)
  {
    if (c1->point_step != c2->point_step || c1->fields.size() != c2->fields.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Lidar point layouts differ; skipping this frame.");
      return;
    }

    auto out = std::make_unique<PointCloud2>();
    // Keep the lidar capture stamp (later of the pair), not wall-clock-now.
    out->header = (rclcpp::Time(c1->header.stamp) >= rclcpp::Time(c2->header.stamp))
      ? c1->header : c2->header;
    out->header.frame_id = output_frame_;
    out->height = 1;
    out->fields = c1->fields;
    out->is_bigendian = c1->is_bigendian;
    out->point_step = c1->point_step;
    out->is_dense = c1->is_dense && c2->is_dense;
    out->width = c1->width + c2->width;
    out->row_step = out->point_step * out->width;

    out->data.resize(c1->data.size() + c2->data.size());
    std::memcpy(out->data.data(), c1->data.data(), c1->data.size());
    std::memcpy(out->data.data() + c1->data.size(), c2->data.data(), c2->data.size());

    pub_->publish(std::move(out));
  }

  std::string topic_1_, topic_2_, output_frame_;
  message_filters::Subscriber<PointCloud2> sub_1_, sub_2_;
  std::shared_ptr<Sync> sync_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarMerger>());
  rclcpp::shutdown();
  return 0;
}
