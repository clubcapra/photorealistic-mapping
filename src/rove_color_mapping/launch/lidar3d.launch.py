# Description:
#   LiDAR SLAM adapted for Livox Mid-360 in repetitive scan pattern mode.
#
#   The Livox Mid-360 is a solid-state LiDAR with a non-repetitive scan
#   pattern by default. This launch file assumes the sensor has been
#   configured to use REPETITIVE scan mode (pattern_mode=1 in livox_ros2_driver
#   config), which gives consistent, structured point clouds suitable for
#   frame-to-frame ICP odometry.
#
#   Prerequisites:
#     1. Install livox_ros2_driver2:
#        https://github.com/Livox-SDK/livox_ros_driver2
#
#     2. Configure the Mid-360 for repetitive scan:
#        In your MID360_config.json, set:
#          "lidar_configs": [ { ..., "pattern_mode": 1 } ]
#        pattern_mode 0 = non-repetitive, 1 = repetitive, 2 = low-frame-rate
#
#     3. Launch the Livox driver (publishes /livox/lidar as sensor_msgs/PointCloud2):
#        $ ros2 launch livox_ros_driver2 msg_MID360_launch.py
#
#     4. (Optional) If an IMU is used, the Mid-360 has a built-in IMU.
#        The driver publishes it on /livox/imu.
#        If no external orientation filter is used, pipe it through
#        imu_filter_madgwick_node (with use_mag:=false publish_tf:=false).
#
#     5. Launch this file:
#        $ ros2 launch rtabmap_examples lidar3d_mid360.launch.py \
#            lidar_topic:=/livox/lidar \
#            imu_topic:=/livox/imu \
#            frame_id:=livox_frame

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context: LaunchContext, *args, **kwargs):

    frame_id = LaunchConfiguration('frame_id')

    imu_topic = LaunchConfiguration('imu_topic')
    imu_used = imu_topic.perform(context) != ''

    rgbd_image_topic = LaunchConfiguration('rgbd_image_topic')
    rgbd_images_topic = LaunchConfiguration('rgbd_images_topic')
    rgbd_image_used = (
        rgbd_image_topic.perform(context) != ''
        or rgbd_images_topic.perform(context) != ''
    )
    rgbd_cameras = 0 if rgbd_images_topic.perform(context) != '' else 1

    voxel_size = LaunchConfiguration('voxel_size')
    voxel_size_value = float(voxel_size.perform(context))

    use_sim_time = LaunchConfiguration('use_sim_time')

    lidar_topic = LaunchConfiguration('lidar_topic')
    lidar_topic_value = lidar_topic.perform(context)
    lidar_topic_deskewed = lidar_topic_value + '/deskewed'

    localization = LaunchConfiguration('localization').perform(context)
    localization = localization in ('true', 'True')

    deskewing = LaunchConfiguration('deskewing').perform(context)
    deskewing = deskewing in ('true', 'True')

    deskewing_slerp = LaunchConfiguration('deskewing_slerp').perform(context)
    deskewing_slerp = deskewing_slerp in ('true', 'True')

    fixed_frame_from_imu = False
    fixed_frame_id = LaunchConfiguration('fixed_frame_id').perform(context)
    if not fixed_frame_id and imu_used:
        fixed_frame_from_imu = True
        fixed_frame_id = frame_id.perform(context) + '_stabilized'

    if not fixed_frame_id or not deskewing:
        lidar_topic_deskewed = lidar_topic

    # Rule of thumb: 10× voxel size.
    # Mid-360 repetitive mode gives denser, more uniform coverage than
    # non-repetitive mode, so a tighter correspondence distance works well.
    max_correspondence_distance = voxel_size_value * 10.0

    # =========================================================================
    # Shared parameters (used by icp_odometry, rtabmap, and rtabmap_viz)
    # =========================================================================
    shared_parameters = {

        'use_sim_time': use_sim_time,
        'frame_id': frame_id,

        # ROS 2 QoS profile for all subscriptions.
        # 0 = system default, 1 = reliable, 2 = best effort.
        # ↑ reliable (1) reduces dropped messages at the cost of potential
        #   backpressure; use best effort (2) on lossy/high-rate transports.
        'qos': LaunchConfiguration('qos'),

        # Whether to use approximate (timestamp-tolerant) topic synchronisation.
        # True when RGBD images are fused – their timestamps rarely align
        # exactly with the lidar. False (exact sync) for lidar-only setups.
        'approx_sync': rgbd_image_used,

        # Maximum seconds to wait for a TF transform before giving up (s).
        # ↑ more tolerance on slow TF publishers / high system load;
        # ↓ faster failure detection if a transform is genuinely missing.
        'wait_for_transform': 0.2,

        # ----- ICP shared parameters -----------------------------------------

        # Use point-to-plane error metric instead of point-to-point.
        # true  → more accurate on smooth / planar surfaces (recommended for
        #         indoor environments scanned by the Mid-360).
        # false → point-to-point; more robust on very noisy or sparse clouds.
        'Icp/PointToPlane': 'true',

        # Maximum number of ICP optimisation iterations per registration call.
        # ↑ better convergence on large initial misalignments, higher CPU cost;
        # ↓ faster but may not converge if the initial guess is poor.
        'Icp/Iterations': '10',

        # Voxel leaf size (m) for downsampling the input cloud before ICP.
        # ↑ fewer points → faster ICP but loses fine detail and small features;
        # ↓ denser cloud → more accurate but slower; must stay above sensor noise.
        'Icp/VoxelSize': str(voxel_size_value),

        # Convergence threshold: ICP stops early when the change in fitness
        # score between iterations falls below this value.
        # ↑ stops sooner (faster but less precise);
        # ↓ keeps iterating until very tight convergence (slower, more accurate).
        'Icp/Epsilon': '0.0001',

        # Number of nearest neighbours used to estimate the local surface normal
        # for point-to-plane ICP. Only used when Icp/PointToPlane=true.
        # ↑ smoother, more reliable normals on noisy clouds, higher memory use;
        # ↓ faster normal estimation, noisier normals on sparse areas.
        'Icp/PointToPlaneK': '20',

        # Radius (m) search for neighbours when estimating normals.
        # 0 = use K-nearest only (Icp/PointToPlaneK governs search).
        # ↑ larger neighbourhood → more robust normals on low-density scans;
        # ↓ 0 disables radius search, relying purely on K neighbours.
        'Icp/PointToPlaneRadius': '0',

        # Hard cap on per-frame translation (m). Registrations that would
        # require a larger jump are rejected as outliers.
        # ↑ allows faster motion or recovery from bad frames;
        # ↓ tighter sanity check; reduces risk of odometry jumps.
        'Icp/MaxTranslation': '0.5',

        # Maximum distance (m) between a source point and its target
        # correspondence. Set automatically to 10× voxel size above.
        # ↑ accepts coarser alignments and larger inter-frame gaps;
        # ↓ rejects distant (likely wrong) correspondences, needs good initial guess.
        'Icp/MaxCorrespondenceDistance': str(max_correspondence_distance),

        # ICP back-end engine.
        # 0 = RTAB-Map's built-in point-to-point ICP;
        # 1 = libpointmatcher (recommended – richer outlier filters, point-to-plane);
        # 2 = fast GICP (PCL).
        'Icp/Strategy': '1',

        # Fraction of correspondences treated as outliers and discarded each
        # iteration (trimmed-ICP style).
        # ↑ more aggressive rejection → robust to clutter but fewer inliers;
        # ↓ keeps more correspondences → good when the scene is clean (like
        #   the Mid-360 repetitive mode with consistent coverage).
        'Icp/OutlierRatio': '0.65',
    }

    # =========================================================================
    # ICP odometry parameters
    # =========================================================================
    # Mid-360 in repetitive mode runs at ~10 Hz (one full pattern per frame).
    # Set expected_update_rate slightly above that in the launch argument.
    icp_odometry_parameters = {

        # Expected lidar publish rate (Hz). Used to detect stale/missing scans.
        # ↑ tighter watchdog – warns sooner if the lidar misses frames;
        # ↓ more tolerant of irregular publishing (e.g. USB dropouts).
        'expected_update_rate': LaunchConfiguration('expected_update_rate'),

        # Correct per-point motion distortion caused by sensor movement during
        # the scan integration window (~100 ms for the Mid-360).
        # true  = enabled; strongly recommended at walking speed and above.
        # false = disabled; only safe when the platform is nearly stationary.
        'deskewing': not fixed_frame_id and deskewing,

        # TF frame name published for the odometry estimate.
        'odom_frame_id': 'icp_odom',

        # TF frame used as an initial pose guess fed into ICP each iteration.
        # Typically the IMU-stabilised frame; empty = no external guess.
        # Providing a good guess dramatically reduces ICP iteration count.
        'guess_frame_id': fixed_frame_id,

        # Use SLERP interpolation between first/last scan stamps for deskewing
        # instead of querying TF for every point.
        # true  = faster, lower latency, slightly less accurate;
        # false = per-point TF lookup, more accurate, higher CPU/latency cost.
        'deskewing_slerp': deskewing_slerp,

        # Fraction of the scan that must differ from the current key-frame
        # before a new key-frame is created.
        # ↑ fewer key-frames → less memory / CPU, but map update is coarser;
        # ↓ more frequent key-frames → richer local map, higher CPU/memory use.
        'Odom/ScanKeyFrameThr': '0.4',

        # Radius (m) used to subtract (thin out) the local map around newly
        # observed points. Should equal the voxel size to avoid duplication.
        # ↑ removes more redundant points → smaller local map, less detail;
        # ↓ keeps more points → denser map, higher memory consumption.
        'OdomF2M/ScanSubtractRadius': str(voxel_size_value),

        # Maximum number of points kept in the local "frame-to-map" reference
        # cloud. Caps memory use; older points are pruned when exceeded.
        # ↑ larger reference → better context for loop closure in large spaces;
        # ↓ smaller reference → lower memory, may lose context in large spaces.
        'OdomF2M/ScanMaxSize': '25000',

        # Run bundle adjustment over recent key-frames to jointly refine poses.
        # false = disabled (recommended for real-time lidar-only odometry);
        # true  = more accurate trajectory but significantly higher CPU cost.
        'OdomF2M/BundleAdjustment': 'false',

        # Minimum fraction of source points that must find a valid correspondence
        # in the reference map for the ICP result to be accepted (odometry level).
        # ↑ stricter acceptance → fewer false positives, more rejected frames;
        # ↓ accepts alignments even with sparse overlap (e.g. open corridors).
        'Icp/CorrespondenceRatio': '0.03',

        # Minimum cloud complexity (0–1) required to attempt point-to-plane ICP.
        # 0.0 = always attempt, regardless of cloud flatness.
        # ↑ skips ICP on very flat / featureless scenes to avoid degenerate fits;
        # ↓ always tries ICP; 0.0 is safe when the Mid-360 sees structured scenes.
        'Icp/PointToPlaneMinComplexity': '0.0',
    }

    if imu_used:
        # Block odometry from starting until the first IMU message is received,
        # ensuring the initial orientation is set before any motion estimate.
        icp_odometry_parameters['wait_imu_to_init'] = True

    # =========================================================================
    # RTAB-Map SLAM parameters
    # =========================================================================
    rtabmap_parameters = {

        # Disable depth / RGB subscriptions – this is a lidar-only pipeline.
        'subscribe_depth': False,
        'subscribe_rgb': False,

        # Receive odometry diagnostic info (inlier count, covariance, etc.)
        # from icp_odometry to help RTAB-Map assess registration quality.
        'subscribe_odom_info': True,

        # Receive the lidar PointCloud2 for scan-based loop-closure and mapping.
        'subscribe_scan_cloud': True,

        # Name of the global map TF frame published by RTAB-Map.
        'map_frame_id': 'map',

        # Re-stamp sensor data to align with the closest odometry timestamp,
        # compensating for lidar/camera timing offsets. Useful when a camera
        # is added later; harmless in lidar-only mode.
        'odom_sensor_sync': True,

        # Maximum graph depth explored when searching for proximity (space)
        # loop-closure candidates. 0 = unlimited depth search.
        # ↑ finds loop closures across the entire graph (slow in large maps);
        # ↓ limits search to recent nodes (faster, may miss distant revisits).
        'RGBD/ProximityMaxGraphDepth': '0',

        # Number of neighbouring nodes along the graph path checked for
        # proximity-based loop closures at each step.
        # ↑ more candidate nodes checked → higher recall, higher CPU cost;
        # ↓ fewer checks → faster, may miss nearby revisits.
        'RGBD/ProximityPathMaxNeighbors': '1',

        # Minimum rotation (rad) the robot must travel before a new node
        # is added to the map graph.
        # ↑ sparser graph → less memory, coarser trajectory;
        # ↓ denser graph → finer trajectory, higher memory / CPU use.
        'RGBD/AngularUpdate': '0.1',

        # Minimum translation (m) the robot must travel before a new node
        # is added to the map graph.
        # ↑ sparser graph → less memory, coarser map;
        # ↓ denser graph → finer trajectory, higher memory / CPU use.
        'RGBD/LinearUpdate': '0.1',

        # Build a 2-D occupancy grid from the 3-D lidar scans.
        # true  = publish /map for navigation stacks;
        # false = skip grid generation (useful if only the 3-D map is needed).
        'RGBD/CreateOccupancyGrid': 'true',

        # Keep nodes in memory even after they are no longer linked in the graph.
        # false = prune unlinked nodes → lower memory use (recommended);
        # true  = retain all nodes → useful for debugging / post-processing.
        'Mem/NotLinkedNodesKept': 'false',

        # Size of the Short-Term Memory (STM) – the sliding window of recent
        # nodes kept in working memory for real-time processing.
        # At 10 Hz, 30 nodes ≈ 3 s of recent history.
        # ↑ longer recent context → better local loop closure, more RAM/CPU;
        # ↓ smaller window → lower overhead, may miss short revisits.
        'Mem/STMSize': '30',

        # Registration strategy used when verifying loop-closure candidates.
        # 0 = visual (feature matching), 1 = ICP (lidar), 2 = visual + ICP.
        'Reg/Strategy': '1',

        # Minimum fraction of source scan points with valid correspondences
        # required to accept a loop-closure registration (map level).
        # Stricter than the odometry-level ratio to enforce quality loop closures.
        # ↑ fewer false loop closures, may miss valid ones with sparse overlap;
        # ↓ accepts looser matches, higher risk of incorrect loop closure.
        'Icp/CorrespondenceRatio': '0.1',

        # Maximum range (m) of lidar returns used when building the occupancy
        # grid. Points beyond this distance are ignored for grid construction.
        # ↑ larger mapped area per scan, more noise from distant returns;
        # ↓ only close, reliable returns used → cleaner grid, smaller coverage.
        'Grid/RangeMax': '20.0',

        # Maximum height (m) above the sensor plane for a point to be
        # classified as ground.
        # ↑ thicker ground band → tolerates uneven floors, may miss low obstacles;
        # ↓ thinner band → stricter ground filtering, better for flat floors.
        'Grid/MaxGroundHeight': '0.25',

        # Maximum height (m) for a point to be labelled an obstacle.
        # Points above this are ignored (e.g. ceiling returns).
        # ↑ includes taller structures (walls, doorframes above robot height);
        # ↓ limits grid to obstacles at or below the robot's navigation plane.
        'Grid/MaxObstacleHeight': '1.8',

        # Minimum number of points in a cluster for it to be labelled an
        # obstacle. Smaller clusters are treated as noise and discarded.
        # ↑ removes more spurious points → cleaner grid, may miss small objects;
        # ↓ keeps smaller clusters → more detail but noisier grid.
        'Grid/MinClusterSize': '30',

        # Segment ground vs. obstacles using local surface normals instead of
        # a simple height threshold.
        # true  = more robust on slopes and ramps;
        # false = faster, assumes a flat ground plane.
        'Grid/NormalsSegmentation': 'true',

        # Height (m) of the robot body used to clear cells directly under the
        # robot footprint (prevents the robot from marking itself as an obstacle).
        # 0.0 = no footprint clearing;
        # ↑ clears a taller volume, useful for robots with a tall chassis.
        'Grid/FootprintHeight': '0.0',

        # Resolution (m) of each occupancy grid cell.
        # ↑ coarser grid → less memory, faster updates, less spatial detail;
        # ↓ finer grid → more precise obstacle boundaries, higher memory cost.
        'Grid/CellSize': '0.05',
    }

    arguments = []
    if localization:
        # Freeze the map: load all saved nodes into working memory but do not
        # add new ones. The robot localises against the existing map only.
        rtabmap_parameters['Mem/IncrementalMemory'] = 'False'
        rtabmap_parameters['Mem/InitWMWithAllNodes'] = 'True'
    else:
        arguments.append('-d')  # Delete previous database (~/.ros/rtabmap.db)

    # =========================================================================
    # Topic remappings
    # =========================================================================
    remappings = [('odom', 'icp_odom')]
    if imu_used:
        remappings.append(('imu', LaunchConfiguration('imu_topic')))
    else:
        remappings.append(('imu', 'imu_not_used'))
    if rgbd_image_used:
        if rgbd_cameras == 1:
            remappings.append(('rgbd_image', LaunchConfiguration('rgbd_image_topic')))
        else:
            remappings.append(('rgbd_images', LaunchConfiguration('rgbd_images_topic')))

    # =========================================================================
    # Node definitions
    # =========================================================================
    nodes = [

        # ICP odometry node – estimates frame-to-frame motion from lidar scans.
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            output='screen',
            parameters=[shared_parameters, icp_odometry_parameters],
            remappings=remappings + [('scan_cloud', lidar_topic_deskewed)],
        ),

        # RTAB-Map SLAM node – global mapping, loop closure, and graph optimisation.
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[
                shared_parameters,
                rtabmap_parameters,
                {'subscribe_rgbd': rgbd_image_used, 'rgbd_cameras': rgbd_cameras},
            ],
            remappings=remappings + [('scan_cloud', lidar_topic_deskewed)],
            arguments=arguments,
        ),

        # Visualisation node – displays the 3-D map and odometry in real time.
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=[
                shared_parameters,
                rtabmap_parameters,
                {'odometry_node_name': 'icp_odometry'},
            ],
            remappings=remappings + [('scan_cloud', 'odom_filtered_input_scan')],
        ),
    ]

    # =========================================================================
    # Optional: IMU → stabilised TF for deskewing
    # =========================================================================
    if fixed_frame_from_imu:
        nodes.append(
            Node(
                package='rtabmap_util',
                executable='imu_to_tf',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'fixed_frame_id': fixed_frame_id,
                    'base_frame_id': frame_id,
                    # How long (s) to wait for the base→fixed transform before
                    # giving up. Keep short to avoid delaying the first scan.
                    # ↑ more tolerant of slow TF; ↓ faster failure detection.
                    'wait_for_transform_duration': 0.001,
                }],
                remappings=[('imu/data', imu_topic)],
            )
        )

    # =========================================================================
    # Optional: Lidar deskewing node (when a fixed frame is available)
    # =========================================================================
    # This is especially important for the Mid-360 because one full repetitive
    # scan takes ~100 ms; at walking speed (~1 m/s) that is ~10 cm of motion
    # per scan – significant enough to distort ICP if not corrected.
    if fixed_frame_id and deskewing:
        nodes.append(
            Node(
                package='rtabmap_util',
                executable='lidar_deskewing',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'fixed_frame_id': fixed_frame_id,
                    # Seconds to wait for the TF transform needed to deskew each
                    # scan. Must cover the full scan integration window (~100 ms).
                    # ↑ more tolerant of TF latency; ↓ faster failure detection.
                    'wait_for_transform': 0.2,
                    # true  = SLERP (fast, slightly less accurate);
                    # false = per-point TF lookup (accurate, higher CPU cost).
                    'slerp': deskewing_slerp,
                }],
                remappings=[('input_cloud', lidar_topic)],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([

        # ── Simulation ────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulated clock.'),

        # ── Deskewing ─────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'deskewing', default_value='true',
            description=(
                'Enable lidar deskewing. Strongly recommended for the Mid-360: '
                'one repetitive scan takes ~100 ms, so motion distortion is '
                'significant at typical robot speeds.'
            )),

        DeclareLaunchArgument(
            'deskewing_slerp', default_value='true',
            description=(
                'Use fast SLERP interpolation for deskewing (less accurate than '
                'per-point TF lookups but much faster). Enable if deskewed scan '
                'latency becomes a problem.'
            )),

        # ── Frames ────────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'frame_id', default_value='livox_frame',
            description='TF frame of the Livox Mid-360 sensor.'),

        DeclareLaunchArgument(
            'fixed_frame_id', default_value='',
            description=(
                'Fixed frame for lidar deskewing. Leave empty to auto-generate '
                'one from the IMU (requires imu_topic to be set).'
            )),

        # ── Mode ──────────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Set true to run in localization-only mode (no mapping).'),

        # ── Topics ────────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'lidar_topic', default_value='/livox/lidar',
            description=(
                'PointCloud2 topic published by livox_ros_driver2. '
                'Make sure the driver is configured to publish sensor_msgs/PointCloud2 '
                '(xfer_format=0 or 1 in MID360_config.json).'
            )),

        DeclareLaunchArgument(
            'imu_topic', default_value='/livox/imu',
            description=(
                'IMU topic from the Mid-360 built-in IMU. '
                'Set to empty string to disable IMU usage. '
                'For best deskewing, pre-filter with imu_filter_madgwick_node '
                '(use_mag:=false publish_tf:=false).'
            )),

        DeclareLaunchArgument(
            'rgbd_image_topic', default_value='',
            description=(
                'RGBD image topic (ignored if empty). '
                'Output of rtabmap_sync rgbd_sync, stereo_sync, or rgb_sync.'
            )),

        DeclareLaunchArgument(
            'rgbd_images_topic', default_value='',
            description=(
                'RGBD images topic for multi-camera setups (overrides '
                'rgbd_image_topic if set). Output of rtabmap_sync rgbdx_sync.'
            )),

        # ── Rate & resolution ─────────────────────────────────────────────────
        DeclareLaunchArgument(
            'expected_update_rate', default_value='12.0',
            description=(
                'Expected lidar frame rate in Hz. The Mid-360 repetitive mode '
                'runs at ~10 Hz; set slightly higher (12 Hz) to avoid timeout warnings. '
                '↑ tighter watchdog; ↓ more tolerant of irregular publishing.'
            )),

        DeclareLaunchArgument(
            'voxel_size', default_value='0.1',
            description=(
                'Voxel size (m) for point cloud downsampling. '
                'For indoor use 0.1–0.2 m. For outdoor / large spaces use 0.3–0.5 m. '
                '↑ fewer points, faster ICP, less detail; ↓ denser cloud, slower but more accurate.'
            )),

        DeclareLaunchArgument(
            'min_loop_closure_overlap', default_value='0.2',
            description=(
                'Minimum scan overlap fraction required to accept a loop closure. '
                '↑ fewer false loop closures; ↓ accepts revisits with less common overlap.'
            )),

        # ── QoS ───────────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'qos', default_value='1',
            description=(
                'ROS 2 QoS: 0=system default, 1=reliable, 2=best effort. '
                'Use 1 (reliable) for livox_ros_driver2.'
            )),

        OpaqueFunction(function=launch_setup),
    ])