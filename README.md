# lidar_camera_colorizer

ROS 2 C++ package for projecting LiDAR `PointCloud2` points into the camera image using
`/home/jay/catkin_depth/src/livox_preprocessed/calib.json`, sampling image pixels, and publishing
an XYZRGB point cloud. It also publishes a static TF so the LiDAR and camera placement can be
inspected in RViz.

The node implementation is written against APIs shared by ROS 2 Foxy and Humble. Launch files stay
in Python because that is the standard ROS 2 launch format, but runtime nodes are C++ only.

## Build

```bash
cd /home/jay/catkin_depth
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select lidar_camera_colorizer
source install/setup.bash
```

## Run With The Included Sample

This publishes one of the existing `.ply` and `.png` pairs, starts the colorizer, and opens RViz.

```bash
ros2 launch lidar_camera_colorizer demo.launch.py
```

Headless run without RViz:

```bash
ros2 launch lidar_camera_colorizer demo.launch.py rviz:=false
```

Use a different sample:

```bash
ros2 launch lidar_camera_colorizer demo.launch.py sample_name:=rosbag2_2023_03_09-13_44_10
```

## Run With Live Topics

The default topics are read from `calib.json`:

- points: `/livox/points`
- image: `/image`
- output: `/livox/color_points`

```bash
ros2 launch lidar_camera_colorizer colorize.launch.py
```

If your live point cloud uses a different frame id than `livox_frame`, override it so the TF tree
matches the cloud header:

```bash
ros2 launch lidar_camera_colorizer colorize.launch.py lidar_frame_id:=your_lidar_frame camera_frame_id:=camera
```

Override topics directly:

```bash
ros2 run lidar_camera_colorizer colorize_node --ros-args \
  -p calib_path:=/home/jay/catkin_depth/src/livox_preprocessed/calib.json \
  -p points_topic:=/livox/points \
  -p image_topic:=/image \
  -p output_topic:=/livox/color_points
```

## Important Parameters

- `use_inverse_transform` defaults to `true`. The sample data projects correctly with
  `p_cam = R.T @ (p_lidar - t)`.
- `max_points` defaults to `0`, which means no point-count reduction. Set it to a positive number
  only when you explicitly want downsampling for speed.
- `keep_uncolored` defaults to `true`, so points outside the image are kept and filled in gray
  instead of being removed.
- `max_image_age_sec` enforces timestamp matching only when it is greater than `0`. The default
  `0.0` disables dropping clouds when image and LiDAR stamps are offset.
- `output_in_camera_frame` publishes transformed camera-frame coordinates when `true`. The default
  is `false`, which keeps the original LiDAR XYZ values and only adds RGB.
- `lidar_frame_id` and `camera_frame_id` define the static TF published from calibration so you can
  inspect sensor placement in RViz.
