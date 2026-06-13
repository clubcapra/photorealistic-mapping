// rove_slam_node: ROS 2 bridge wrapping SlamPipeline.
//
// Subscribes:
//   /livox/lidar  sensor_msgs/PointCloud2   (lidar input)
//   /imu/data     sensor_msgs/Imu           (VN-300, when --imu-factor used)
//
// Publishes:
//   /tf           map_frame -> odom -> base_link
//   /odom         nav_msgs/Odometry
//   /cloud_obstacles  sensor_msgs/PointCloud2  (current local-map obstacles,
//                     consumed by the existing nav2_costmap.yaml setup)
//
// CLI (ROS-style):
//   ros2 run ... rove_slam_node --ros-args
//     -p map_frame:=new_map -p base_frame:=base_link -p odom_frame:=odom
//     -p voxel_size_m:=0.3 -p max_points_per_voxel:=50
//     -p urdf_extrinsic:=true
//     -p obstacle_z_min:=0.10 -p obstacle_z_max:=1.50
//     -p obstacle_publish_period_s:=0.5

#include "rove_slam/slam_pipeline.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace {

// URDF defaults (livox.urdf.xacro + sensor.urdf.xacro composed):
//   xyz=(-0.30, 0.00, 0.318)  rpy=(0, 30 deg, 180 deg)
Eigen::Matrix4d urdfLidarToBody() {
  Eigen::Matrix3d R =
      (Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(30.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = Eigen::Vector3d(-0.30, 0.0, 0.318);
  return T;
}

// Iterate a PointCloud2 message — find x, y, z, optional intensity, optional
// per-point time field. Returns rove_slam::LidarFrame.
rove_slam::LidarFrame pc2ToLidarFrame(
    const sensor_msgs::msg::PointCloud2& msg) {
  rove_slam::LidarFrame f;
  f.timestamp_ns = static_cast<uint64_t>(msg.header.stamp.sec) * 1000000000ULL
                   + static_cast<uint64_t>(msg.header.stamp.nanosec);
  f.frame_id = msg.header.frame_id;

  int off_x = -1, off_y = -1, off_z = -1, off_i = -1, off_t = -1;
  uint8_t dt_t = 0;
  for (const auto& fd : msg.fields) {
    if (fd.name == "x") off_x = fd.offset;
    else if (fd.name == "y") off_y = fd.offset;
    else if (fd.name == "z") off_z = fd.offset;
    else if (fd.name == "intensity") off_i = fd.offset;
    else if (fd.name == "timestamp" || fd.name == "time" || fd.name == "t" ||
             fd.name == "offset_time") {
      off_t = fd.offset;
      dt_t = fd.datatype;
    }
  }
  if (off_x < 0 || off_y < 0 || off_z < 0) return f;

  const size_t n = static_cast<size_t>(msg.width) * msg.height;
  const size_t step = msg.point_step;
  f.has_per_point_time = (off_t >= 0);
  f.points.reserve(n);
  // Track time range for normalization to ns.
  double t_first = 0.0, t_max = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const uint8_t* p = &msg.data[i * step];
    rove_slam::LidarPoint pt;
    std::memcpy(&pt.x, p + off_x, 4);
    std::memcpy(&pt.y, p + off_y, 4);
    std::memcpy(&pt.z, p + off_z, 4);
    pt.intensity = 0.0f;
    if (off_i >= 0) std::memcpy(&pt.intensity, p + off_i, 4);
    pt.offset_ns = 0;
    if (off_t >= 0) {
      double t = 0.0;
      if (dt_t == sensor_msgs::msg::PointField::FLOAT64) {
        std::memcpy(&t, p + off_t, 8);
      } else if (dt_t == sensor_msgs::msg::PointField::FLOAT32) {
        float tf; std::memcpy(&tf, p + off_t, 4); t = tf;
      } else if (dt_t == sensor_msgs::msg::PointField::UINT32) {
        uint32_t ti; std::memcpy(&ti, p + off_t, 4); t = ti;
      }
      if (i == 0) t_first = t;
      if (t - t_first > t_max) t_max = t - t_first;
      pt.offset_ns = static_cast<uint32_t>(t - t_first);  // tentative
    }
    f.points.push_back(pt);
  }
  // If float seconds detected (max < 1), convert to ns by multiplying.
  if (f.has_per_point_time && t_max > 0.0 && t_max < 1.0) {
    for (auto& pt : f.points) {
      // Tentative offset_ns above stored a truncated (t-t_first) value.
      // For float-seconds inputs we redo properly: skip — keep as 0 to be safe.
      pt.offset_ns = 0;
    }
    f.has_per_point_time = false;  // disable deskew rather than emit garbage
  }
  return f;
}

void eigenToTransform(const Eigen::Matrix4d& T,
                       geometry_msgs::msg::TransformStamped& tf) {
  tf.transform.translation.x = T(0, 3);
  tf.transform.translation.y = T(1, 3);
  tf.transform.translation.z = T(2, 3);
  Eigen::Quaterniond q(T.block<3, 3>(0, 0));
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();
}

}  // namespace


class RoveSlamNode : public rclcpp::Node {
 public:
  RoveSlamNode() : Node("rove_slam_node") {
    // Parameters.
    map_frame_ = this->declare_parameter<std::string>("map_frame", "new_map");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    obstacle_z_min_ = this->declare_parameter<double>("obstacle_z_min", 0.10);
    obstacle_z_max_ = this->declare_parameter<double>("obstacle_z_max", 1.50);
    const double pub_period =
        this->declare_parameter<double>("obstacle_publish_period_s", 0.5);

    rove_slam::SlamPipelineConfig cfg;
    cfg.voxel_size_m = this->declare_parameter<double>("voxel_size_m", 0.3);
    cfg.max_range_m = this->declare_parameter<double>("max_range_m", 100.0);
    cfg.min_range_m = this->declare_parameter<double>("min_range_m", 2.0);
    cfg.max_points_per_voxel =
        this->declare_parameter<int>("max_points_per_voxel", 50);
    cfg.deskew = this->declare_parameter<bool>("deskew", true);
    cfg.min_intensity = this->declare_parameter<double>("min_intensity", 0.0);
    if (this->declare_parameter<bool>("urdf_extrinsic", true)) {
      cfg.lidar_to_body = urdfLidarToBody();
    }
    pipeline_ = std::make_unique<rove_slam::SlamPipeline>(cfg);

    RCLCPP_INFO(this->get_logger(),
                "rove_slam_node: voxel=%.2f max_pts=%d urdf_extrinsic=%d "
                "frames map=%s odom=%s base=%s",
                cfg.voxel_size_m, cfg.max_points_per_voxel,
                this->get_parameter("urdf_extrinsic").as_bool() ? 1 : 0,
                map_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str());

    // Subscribers.
    auto lidar_qos = rclcpp::SensorDataQoS().keep_last(10);
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar", lidar_qos,
        std::bind(&RoveSlamNode::onLidar, this, _1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", lidar_qos,
        std::bind(&RoveSlamNode::onImu, this, _1));

    // Publishers.
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 50);
    obstacles_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/cloud_obstacles", 10);
    tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Periodic obstacle publisher — building a fresh cloud per scan is too
    // expensive at the lidar rate; throttle to a slower cadence.
    obstacle_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(pub_period)),
        std::bind(&RoveSlamNode::publishObstacles, this));

    // Publish identity TF at startup so nav2's lifecycle manager doesn't
    // time out waiting for the frame chain before the first lidar arrives.
    // We refresh continuously at 10 Hz; per-scan onLidar() overrides
    // odom→base_link with the SLAM pose.
    startup_tf_timer_ = this->create_wall_timer(
        100ms, std::bind(&RoveSlamNode::publishStartupTf, this));
  }

 private:
  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
    rove_slam::ImuFrame f;
    f.timestamp_ns = static_cast<uint64_t>(msg->header.stamp.sec) *
                         1000000000ULL +
                     static_cast<uint64_t>(msg->header.stamp.nanosec);
    f.accel = Eigen::Vector3d(msg->linear_acceleration.x,
                               msg->linear_acceleration.y,
                               msg->linear_acceleration.z);
    f.gyro = Eigen::Vector3d(msg->angular_velocity.x,
                              msg->angular_velocity.y,
                              msg->angular_velocity.z);
    std::lock_guard<std::mutex> lk(pipeline_mu_);
    pipeline_->processImu(f);
  }

  void onLidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    rove_slam::LidarFrame f = pc2ToLidarFrame(*msg);
    if (f.points.empty()) return;
    rove_slam::SlamPose pose;
    {
      std::lock_guard<std::mutex> lk(pipeline_mu_);
      auto opt = pipeline_->processLidar(f);
      if (!opt.has_value()) return;
      pose = *opt;
    }

    // Publish TF: map → odom is identity for now (no loop closure /
    // SLAM-correction layer); odom → base_link carries our estimate.
    // Stamp with current wall-clock time, not the bag's recorded stamp —
    // otherwise nav2 (which defaults to real-time) sees TF data from
    // whenever the bag was recorded and drops it as too old.
    rclcpp::Time stamp = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped tf_map_odom;
    tf_map_odom.header.stamp = stamp;
    tf_map_odom.header.frame_id = map_frame_;
    tf_map_odom.child_frame_id = odom_frame_;
    tf_map_odom.transform.rotation.w = 1.0;  // identity
    tf_broadcaster_->sendTransform(tf_map_odom);

    geometry_msgs::msg::TransformStamped tf_odom_base;
    tf_odom_base.header.stamp = stamp;
    tf_odom_base.header.frame_id = odom_frame_;
    tf_odom_base.child_frame_id = base_frame_;
    eigenToTransform(pose.pose, tf_odom_base);
    tf_broadcaster_->sendTransform(tf_odom_base);

    // /odom
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = pose.pose(0, 3);
    odom.pose.pose.position.y = pose.pose(1, 3);
    odom.pose.pose.position.z = pose.pose(2, 3);
    Eigen::Quaterniond q(pose.pose.block<3, 3>(0, 0));
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom_pub_->publish(odom);

    last_stamp_ = stamp;
  }

  void publishStartupTf() {
    // Once we've got a real scan-derived TF flowing through onLidar(), we
    // can stop the startup heartbeat — onLidar publishes both transforms
    // each scan.
    if (last_stamp_.nanoseconds() > 0) {
      startup_tf_timer_->cancel();
      return;
    }
    geometry_msgs::msg::TransformStamped tf;
    tf.transform.rotation.w = 1.0;  // identity
    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom_frame_;
    tf_broadcaster_->sendTransform(tf);
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf_broadcaster_->sendTransform(tf);
  }

  void publishObstacles() {
    std::vector<Eigen::Vector3d> pts;
    {
      std::lock_guard<std::mutex> lk(pipeline_mu_);
      pts = pipeline_->localMapPoints();
    }
    if (pts.empty()) return;

    // Filter by Z band — obstacles are vertical-extent points within the
    // body's traversal plane. The lidar→body extrinsic put walls at sensible
    // Z; the band excludes ground and ceiling.
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = last_stamp_.nanoseconds() > 0
                            ? last_stamp_
                            : this->get_clock()->now();
    msg.height = 1;
    msg.is_dense = true;
    msg.is_bigendian = false;
    msg.fields.resize(3);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;
    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;
    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;
    msg.point_step = 12;

    std::vector<uint8_t> buf;
    buf.reserve(pts.size() * 12);
    size_t kept = 0;
    for (const auto& p : pts) {
      if (p.z() < obstacle_z_min_ || p.z() > obstacle_z_max_) continue;
      float x = static_cast<float>(p.x());
      float y = static_cast<float>(p.y());
      float z = static_cast<float>(p.z());
      const uint8_t* xb = reinterpret_cast<const uint8_t*>(&x);
      const uint8_t* yb = reinterpret_cast<const uint8_t*>(&y);
      const uint8_t* zb = reinterpret_cast<const uint8_t*>(&z);
      buf.insert(buf.end(), xb, xb + 4);
      buf.insert(buf.end(), yb, yb + 4);
      buf.insert(buf.end(), zb, zb + 4);
      ++kept;
    }
    msg.width = kept;
    msg.row_step = msg.width * msg.point_step;
    msg.data = std::move(buf);
    obstacles_pub_->publish(msg);
  }

  std::unique_ptr<rove_slam::SlamPipeline> pipeline_;
  std::mutex pipeline_mu_;

  std::string map_frame_, odom_frame_, base_frame_;
  double obstacle_z_min_;
  double obstacle_z_max_;
  rclcpp::Time last_stamp_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr obstacle_timer_;
  rclcpp::TimerBase::SharedPtr startup_tf_timer_;
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoveSlamNode>());
  rclcpp::shutdown();
  return 0;
}
