import datetime

from launch import LaunchDescription
import launch.logging
from launch.actions import ExecuteProcess, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
import os
import subprocess
import shutil
import yaml
import rclpy
import rclpy.node

ros_bag_dir = os.getenv("ROSBAG_OUTPUT_PATH", "/mnt/ssd/sftp/bags")

def _dir_size(path) -> int:
    total = 0
    for dirpath, _, filenames in os.walk(path):
        for f in filenames:
            fp = os.path.join(dirpath, f)
            if os.path.isfile(fp):
                total += os.path.getsize(fp)
    return total


def _fmt_size(size_bytes) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if size_bytes < 1024:
            return f"{size_bytes:.2f} {unit}"
        size_bytes /= 1024
    return f"{size_bytes:.2f} TB"


def compress_bag(context, *args, **kwargs):
    ros_log_dir = launch.logging.launch_config.log_dir
    bag_dir = os.path.join(ros_log_dir, "bag")
    compressed_dir = os.path.join(ros_log_dir, "bag_compressed")
    backup_dir = os.path.join(ros_log_dir, "bag_backup")
    config_path = os.path.join(ros_log_dir, "compress_config.yaml")

    rclpy.init()
    node = rclpy.node.Node("bag_compressor")
    log = node.get_logger()

    try:
        if not os.path.isdir(bag_dir):
            log.warn(f"Bag directory {bag_dir} not found, skipping.")
            return

        convert_config = {
            "output_bags": [
                {
                    "uri": compressed_dir,
                    "storage_id": "mcap",
                    "storage_preset_profile": "zstd_fast",
                    "all": True,
                }
            ]
        }

        with open(config_path, "w") as f:
            yaml.dump(convert_config, f)

        original_size = _dir_size(bag_dir)
        log.info(f"Starting compression of {bag_dir} ({_fmt_size(original_size)})...")

        try:
            subprocess.run(
                [
                    "ros2", "bag", "convert",
                    "--input", bag_dir,
                    "--output-options", config_path,
                ],
                check=True,
                timeout=1200,
            )

            if not os.path.isdir(compressed_dir) or not any(
                f.endswith(".mcap") for f in os.listdir(compressed_dir)
            ):
                log.error("Output bag missing or empty — original preserved.")
                _cleanup(compressed_dir, log)
                return

            os.rename(bag_dir, backup_dir)
            os.rename(compressed_dir, bag_dir)
            shutil.rmtree(backup_dir)

            compressed_size = _dir_size(bag_dir)
            ratio = original_size / compressed_size if compressed_size > 0 else 0
            saved = original_size - compressed_size
            log.info(
                f"Compression complete. "
                f"Original: {_fmt_size(original_size)}, "
                f"Compressed: {_fmt_size(compressed_size)}, "
                f"Saved: {_fmt_size(saved)} ({100 * saved / original_size:.1f}%), "
                f"Ratio: {ratio:.2f}x"
            )

        except subprocess.TimeoutExpired:
            log.error("ros2 bag convert timed out — original preserved.")
            _cleanup(compressed_dir, log)
        except subprocess.CalledProcessError as e:
            log.error(f"ros2 bag convert failed (exit {e.returncode}) — original preserved.")
            _cleanup(compressed_dir, log)
        except Exception as e:
            log.error(f"Unexpected error: {e} — original preserved.")
            _cleanup(compressed_dir, log)
        finally:
            if os.path.exists(config_path):
                os.remove(config_path)

    finally:
        node.destroy_node()
        rclpy.shutdown()


def _cleanup(path, log):
    if os.path.exists(path):
        shutil.rmtree(path)
        log.warn(f"Removed incomplete output at {path}")


def generate_launch_description():
    now = datetime.datetime.now()
    output_dir = os.path.join(ros_bag_dir, f"{now.year}-{now.month}-{now.day}-{now.hour}-{now.minute}-{now.second}")

    rosbag = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record",
             "--output", output_dir,
            "--storage", "mcap",
            "--max-cache-size", str(1 * 1024), # 1KB
            "--compression-mode", "file",
            "--storage-preset-profile", "zstd_fast",
            "--max-bag-size", str(1 * 1024 * 1024 * 1024),  # 1GB
            "--max-bag-duration", str(10 * 60),             # 10 minutes in seconds
            "/apriltag/detections",
            "/aruco/detections",
            "/aruco_opencv/detections",
            # "/cam_east/camera_info_rect",
            # "/cam_east/image_rect",
            # "/cam_north/camera_info_rect",
            # "/cam_north/image_rect",
            "/cam_south/camera_info_rect",
            "/cam_south/image_rect",
            # "/cam_west/camera_info_rect",
            # "/cam_west/image_rect",
            "/diagnostics",
            "/goal",
            "/goal_node",
            "/imu/data",
            "/imu/data_raw",
            "/imu/mag",
            "/imu/pressure",
            "/imu/temperature",
            "/initialpose",
            "/labels",
            "/landmark_detection",
            "/landmark_detections",
            "/landmarks",
            # "/livox/imu_192_168_2_40",
            # "/livox/imu_192_168_2_41",
            "/livox/lidar",
            # "/livox/lidar_192_168_2_40",
            # "/livox/lidar_192_168_2_41",
            "/robot_description",
            "/tag_detections",
            "/vectornav/status",
        ],
        output="screen",
    )

    return LaunchDescription([
        rosbag,
        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action=rosbag,
        #         on_exit=[OpaqueFunction(function=compress_bag)]
        #     )
        # ),
    ])