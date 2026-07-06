# Usage (ROS 2)

## urdf_model_markers.launch.py
This launch file generates interactive markers from URDF models.
```bash
$ ros2 launch jsk_interactive_marker urdf_model_markers.launch.py models:=/path/to/model.yaml
```

The model list is read from the YAML file given by `models` (the ROS 1
structured `model_config` rosparam is replaced by the `models_config_file`
parameter; see `src/urdf_model_marker_main.cpp` for the format).

### example
```bash
$ ros2 launch jsk_interactive_marker urdf_model_markers.launch.py \
    models:=$(ros2 pkg prefix --share jsk_interactive_marker)/launch/models/sample_models.yaml
```

## Other samples
```bash
$ ros2 launch jsk_interactive_marker marker_6dof_sample.launch.py
$ ros2 launch jsk_interactive_marker bounding_box_marker_sample.launch.py
$ ros2 launch jsk_interactive_marker transformable_server_sample.launch.py
$ ros2 launch jsk_interactive_marker sample_transformable_markers_client.launch.py
$ ros2 launch jsk_interactive_marker sample_camera_info_publisher.launch.py
```

## Not ported to ROS 2
- `room2yaml.l` — EusLisp script that generated YAML for `urdf_model_marker`.
  EusLisp is not available on ROS 2, so the YAML files must be prepared
  manually (see `launch/models/*.yaml` for examples).
- `robot_actions_sample.launch` and other robot-specific launches
  (atlas / hrp2 / pr2 / staro / samplerobot) — the corresponding robot
  description packages have no ROS 2 release.
