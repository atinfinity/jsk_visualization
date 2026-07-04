jsk_visualization (ROS 2)
=========================

[![ros2](https://github.com/atinfinity/jsk_visualization/actions/workflows/ros2.yml/badge.svg?branch=ros2)](https://github.com/atinfinity/jsk_visualization/actions/workflows/ros2.yml)

ROS 2 Jazzy port of the JSK visualization packages. This `ros2` branch is a
work-in-progress port of the original ROS 1 packages; the `master` branch
remains ROS 1.

Currently ported: **jsk_rviz_plugins** (48 rviz plugin classes: displays,
panels, tools and a view controller) together with its messages and the
rclpy versions of its scripts and samples. `jsk_rqt_plugins` and
`jsk_interactive_markers` are not ported yet (`COLCON_IGNORE`).

Build
-----

Requires ROS 2 Jazzy.

```bash
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
git clone -b ros2 https://github.com/atinfinity/jsk_visualization.git
# jsk_recognition_msgs (ROS 2 port) and other source dependencies:
vcs import < jsk_visualization/ros2.repos
cd ~/dev_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Try it:

```bash
ros2 launch jsk_rviz_plugins overlay_sample.launch.py
ros2 launch jsk_rviz_plugins boundingbox_sample.launch.py
```

Packages on this branch
-----------------------

| Package | Role |
|---|---|
| `jsk_rviz_plugins` | The rviz2 plugins, scripts and samples |
| `jsk_rviz_plugins_msgs` | Messages/services that used to live inside jsk_rviz_plugins (OverlayText, Pictogram, ...). Definitions are modernized: constants use `ACTION_`/`MODE_`/`SHAPE_` prefixes and command fields are `uint8` |
| `jsk_footstep_msgs`, `jsk_hark_msgs`, `jsk_gui_msgs`, `view_controller_msgs`, `people_msgs` | **Stopgap** interface packages: none of these have a ROS 2 release, so the definitions (identical to their ROS 1 upstreams) are built from source here. Delete each one as soon as an official ROS 2 release appears |

`jsk_recognition_msgs` is consumed from the `ros2` branch of the
[jsk_recognition](https://github.com/atinfinity/jsk_recognition) fork (full
rosidl port of all 76 msgs / 24 srvs / 2 actions); see `ros2.repos`.

Vendored code inside `jsk_rviz_plugins/src/`: `jsk_topic_tools/color_utils`
and a PCL-free minimal `jsk_recognition_utils::Plane/Polygon` (own
ear-clipping triangulation), both to be removed when those packages get
ROS 2 releases.

Port status
-----------

All plugins from the ROS 1 `plugin_description.xml` are ported except:

- **OverlayCamera** — forks the ROS 1 rviz camera display internals, which
  were fully restructured in rviz2; needs a rewrite.
- Scripts/samples depending on `hrpsys_ros_bridge`
  (contact_state_marker, motor_states_temperature_decomposer,
  landing_time_detector, contact_state_sample).
- `classification_result_visualizer.py` lost its
  `~input/ObjectDetection` input (posedetection_msgs has no ROS 2 release).

Notable behavior differences from ROS 1:

- Message constants were renamed in `jsk_rviz_plugins_msgs`
  (e.g. `OverlayText.ADD` → `ACTION_ADD`,
  `Pictogram.PICTOGRAM_MODE` → `MODE_PICTOGRAM`).
- The CancelAction panel calls the `action_msgs/srv/CancelGoal` service of
  ROS 2 actions instead of publishing `actionlib_msgs/GoalID`.
- Screen-capture features (ScreenshotListener, VideoCapture,
  RvizScenePublisher) need an X11 session; on Wayland run rviz2 with
  `QT_QPA_PLATFORM=xcb`.
- dynamic_reconfigure interfaces became ROS 2 node parameters.

Tests
-----

```bash
QT_QPA_PLATFORM=offscreen colcon test --packages-select jsk_rviz_plugins
colcon test-result --verbose
```

The gtest `test_plugin_load` instantiates every exported Display/Panel/Tool
through pluginlib (view controllers are only load-checked:
`rviz_common::ViewController` cannot be destroyed without rviz
initialization).

License
-------

BSD (see individual file headers).
