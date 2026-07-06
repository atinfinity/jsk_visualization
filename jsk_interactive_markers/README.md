jsk_interactive_markers
=======================

Interactive marker tools used in the JSK lab (ROS 2 port).

## Packages

| Package | ROS 2 | Description |
|---|:---:|---|
| `jsk_interactive_marker` | ✅ built | Interactive-marker nodes and rviz plugins: 6-DOF markers, transformable markers, URDF model markers, bounding-box / polygon / footstep markers, a camera-info marker, etc. |
| `jsk_interactive_test` | ✅ built | Teleop / test launch files. |
| `jsk_interactive` | ⛔ `COLCON_IGNORE` | EusLisp-based robot joint/hand teleoperation (`move-joint-interface`, PR2/Atlas control). EusLisp has no ROS 2 equivalent, so this package is not built on ROS 2. |

## Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to jsk_interactive_marker
source install/setup.bash
```

## Samples

Ported sample launch files (rviz opens with the corresponding markers):

| launch | Description |
|---|---|
| `marker_6dof_sample.launch.py` | 6-DOF interactive markers (cube / sphere / line / mesh) |
| `bounding_box_marker_sample.launch.py` | interactive markers for a `BoundingBoxArray` |
| `transformable_server_sample.launch.py` | transformable-object server; insert a torus/box/cylinder via the rviz menu or the `request_marker_operate` service |
| `sample_transformable_markers_client.launch.py` | transformable markers driven from a YAML config through the client |
| `urdf_model_markers.launch.py` | interactive markers built from URDF models |
| `sample_camera_info_publisher.launch.py` | `CameraInfo` publisher with an interactive camera marker (no rviz) |

Examples:

```bash
ros2 launch jsk_interactive_marker marker_6dof_sample.launch.py

ros2 launch jsk_interactive_marker urdf_model_markers.launch.py \
    models:=$(ros2 pkg prefix --share jsk_interactive_marker)/launch/models/sample_models.yaml
```

## Nodes

Main interactive-marker nodes:

- `interactive_marker_interface` — control a hand pose with a 6-DOF interactive marker
- `urdf_model_marker` — build interactive markers from a URDF
- `transformable_server_sample` — server for transformable objects (torus/box/cylinder)
- `marker_6dof`, `bounding_box_marker`, `polygon_marker`, `footstep_marker`, `camera_info_publisher`, ...

The full node list and ROS 2 migration status is documented in
[`doc/jsk_interactive_marker/ros2_migration_status.md`](../doc/jsk_interactive_marker/ros2_migration_status.md).

## ROS 1 -> ROS 2 の主な変更点

- launch は `.launch.py` に移植。ロボット固有 launch（atlas / hrp2 / pr2 / staro / samplerobot）は、対応する robot description が ROS 2 未提供のため未移植。
- 構造化 rosparam / dynamic_reconfigure はノードパラメータ（YAML ファイルパラメータ + `on_set_parameters_callback`）に置換。例: `model_config` -> `models_config_file`。
- `~server` などの private-name remap はパラメータ（例: `server`）に変更。
- EusLisp ベースのロボット関節/ハンド teleop（`jsk_interactive`）と、それを用いる PR2/Atlas 等の teleop launch は未移植。
