# jsk_rviz_plugins ROS 2 スモークテスト表

全49プラグインクラスの動作確認状況。
「ロード」列は `test_plugin_load` gtest(pluginlibでの生成+破棄、ViewControllerはライブラリ解決のみ)により**CIで毎回自動確認**される。
「描画」列は実データを流した rviz2 での目視/無エラー確認(最終実施: 2026-07-04、DISPLAY=:1)。

CIでは上記gtestに加え、launch_testingで
「サンプルパブリッシャ13本が実際にトピックへ配信すること」
(`test/test_samples_launch.py`)と全XMLの妥当性(`ament_xmllint`)を検証する。

確認コマンド例:

```bash
source install/setup.bash
ros2 launch jsk_rviz_plugins <サンプル>.launch.py
```

## Displays

| クラス | ロード | 描画 | 確認方法 |
|---|---|---|---|
| OverlayText | ✅ | ✅ | `overlay_sample.launch.py` |
| String | ✅ | ✅ | `overlay_sample.launch.py` |
| OverlayMenu | ✅ | ✅ | `overlay_sample.launch.py` |
| PieChart | ✅ | ✅ | `piechart_sample.launch.py` |
| Plotter2D | ✅ | ✅ | `overlay_sample.launch.py` |
| LinearGauge | ✅ | ✅ | `linear_gauge_sample.launch.py` |
| OverlayDiagnostic | ✅ | ⬜ 手動 | `ros2 run jsk_rviz_plugins diagnostics_sample.py` + rviz2 で追加 |
| OverlayImage | ✅ | ✅ | OverlayCamera検証と同じ画像パブリッシャで確認可 |
| TargetVisualizer | ✅ | ⬜ 手動 | `ros2 topic pub /target geometry_msgs/msg/PoseStamped ...` |
| Pictogram | ✅ | ⬜ 手動 | `ros2 run jsk_rviz_plugins pictogram.py` |
| PictogramArray | ✅ | ✅ | `pictogram_sample.launch.py` |
| Diagnostics | ✅ | ⬜ 手動 | `ros2 run jsk_rviz_plugins diagnostics_sample.py` |
| TFTrajectory | ✅ | ⬜ 手動 | 動くTFフレームが必要(例: robot_state_publisher) |
| TwistStamped | ✅ | ⬜ 手動 | `ros2 topic pub ... geometry_msgs/msg/TwistStamped` |
| PoseArray | ✅ | ⬜ 手動 | 非推奨(rviz標準のPoseArray推奨) |
| CameraInfo | ✅ | ✅ | `overlay_camera_sample.launch.py` |
| NormalDisplay | ✅ | ✅ | `normal_sample.launch.py` |
| PeoplePositionMeasurementArray | ✅ | ⬜ 手動 | people_msgs/PositionMeasurementArray を配信 |
| BoundingBox | ✅ | ⬜ 手動 | bbox サンプルの1要素版 |
| BoundingBoxArray | ✅ | ✅ | `boundingbox_sample.launch.py` |
| TorusArray | ✅ | ✅ | `torus_array_sample.launch.py` |
| SegmentArray | ✅ | ✅ | `segment_array_sample.launch.py` |
| HumanSkeletonArray | ✅ | ⬜ 手動 | HumanSkeletonArray を配信 |
| SimpleOccupancyGridArray | ✅ | ✅ | `occupancy_grid_sample.launch.py` |
| PolygonArray | ✅ | ✅ | `polygon_array_sample.launch.py` |
| Footstep | ✅ | ✅ | `footstep_sample.launch.py` |
| AmbientSound | ✅ | ⬜ 手動 | jsk_hark_msgs/HarkPower を配信 |
| RvizScenePublisher | ✅ | ⬜ 手動 | X11必須(`ros2 topic hz /rviz/image` で確認) |
| VideoCapture | ✅ | ⬜ 手動 | X11必須(start captureでaviが生成されるか) |
| QuietInteractiveMarker | ✅ | ⬜ 手動 | interactive_markers サーバが必要 |
| OverlayCamera | ✅ | ✅ | `overlay_camera_sample.launch.py` |

## Panels

| クラス | ロード | 動作 | 確認方法 |
|---|---|---|---|
| PublishTopic | ✅ | ⬜ 手動 | パネル追加→topic名入力→`ros2 topic echo` |
| CancelAction | ✅ | ⬜ 手動 | 実行中のROS 2アクションに対してcancel |
| RecordAction | ✅ | ⬜ 手動 | `/record_command` を echo |
| SelectPointCloudPublishAction | ✅ | ⬜ 手動 | SelectionManager API変更のため要重点確認 |
| ObjectFitOperatorAction | ✅ | ⬜ 手動 | `/object_fit_command` を echo |
| RobotCommandInterfaceAction | ✅ | ✅起動 | `robot_command_interface_sample.launch.py`(ボタン表示まで確認) |
| EmptyServiceCallInterfaceAction | ✅ | ✅起動 | `service_call_panel.launch.py` |
| YesNoButton | ✅ | ⬜ 手動 | `ros2 service call /rviz/yes_no_button jsk_gui_msgs/srv/YesNo "{message: test}"` |
| TabletControllerPanel | ✅ | ⬜ 手動 | `/spots_marker_array` 等の配信が必要 |

## Tools / ViewController

| クラス | ロード | 動作 | 確認方法 |
|---|---|---|---|
| OverlayPicker | ✅ | ⬜ 手動 | overlay_sample起動中にツール選択→オーバーレイをドラッグ |
| CloseAll / OpenAll | ✅ | ⬜ 手動 | ツール実行でDisplayツリーが開閉 |
| ScreenshotListener | ✅ | ⬜ 手動 | `ros2 service call /rviz/screenshot jsk_rviz_plugins_msgs/srv/Screenshot "{file_name: /tmp/s.png}"`(X11) |
| TabletViewController | ✅(解決のみ) | ⬜ 手動 | Views→TabletViewController切替、`/rviz/camera_placement` 配信 |

## 既知の制約

- 画面キャプチャ系(ScreenshotListener / VideoCapture / RvizScenePublisher)はX11必須。Waylandでは `QT_QPA_PLATFORM=xcb` で起動する。
- ViewControllerは rviz_common の設計上、rviz外でのインスタンス化テストができない(基底デストラクタが `context_` を無条件参照)。
- `contact_state_marker` 系スクリプトは hrpsys_ros_bridge (ROS 1専用) 待ちで未移植。
