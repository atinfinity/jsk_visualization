# jsk_rviz_plugins ROS 2 移植状況

`plugin_description.xml` に登録された RViz プラグイン全 **45 クラス**（Display 31 / Panel 9 / Tool 4 / ViewController 1）はすべて ROS 2 (Jazzy) へ移植済みです。以下に種別ごとの一覧を示します。

- 移植状況: ✅ = 移植完了
- 備考: ROS 2 固有の差異・依存・制約がある場合のみ記載（無印は標準的な移植）

## Display（31）

| プラグイン名 | 移植状況 | 備考 |
|---|:---:|---|
| OverlayText | ✅ | |
| String | ✅ | std_msgs/String をオーバーレイ表示 |
| OverlayMenu | ✅ | |
| PieChart | ✅ | |
| Plotter2D | ✅ | |
| LinearGauge | ✅ | |
| OverlayDiagnostic | ✅ | diagnostic_msgs/DiagnosticArray を購読（ROS 1 と同一） |
| OverlayImage | ✅ | |
| TargetVisualizer | ✅ | |
| Pictogram | ✅ | |
| PictogramArray | ✅ | |
| Diagnostics | ✅ | |
| TFTrajectory | ✅ | |
| TwistStamped | ✅ | |
| PoseArray | ✅ | |
| CameraInfo | ✅ | |
| NormalDisplay | ✅ | |
| PeoplePositionMeasurementArray | ✅ | **people_msgs（stopgap）が見つかる場合のみ条件ビルド** |
| RvizScenePublisher | ✅ | 画面キャプチャ系（X11セッションが必要） |
| VideoCapture | ✅ | 画面キャプチャ系（X11セッションが必要） |
| QuietInteractiveMarker | ✅ | |
| OverlayCamera | ✅ | ROS 2 rviz2 のカメラ構造に合わせて移植 |
| BoundingBox | ✅ | jsk_recognition_msgs 依存 |
| BoundingBoxArray | ✅ | jsk_recognition_msgs 依存 |
| TorusArray | ✅ | jsk_recognition_msgs 依存 |
| SegmentArray | ✅ | jsk_recognition_msgs 依存 |
| HumanSkeletonArray | ✅ | jsk_recognition_msgs 依存 |
| SimpleOccupancyGridArray | ✅ | jsk_recognition_msgs 依存 |
| PolygonArray | ✅ | jsk_recognition_msgs 依存 |
| Footstep | ✅ | jsk_footstep_msgs（stopgap）依存 |
| AmbientSound | ✅ | jsk_hark_msgs（stopgap）依存 |

## Panel（9）

| プラグイン名 | 移植状況 | 備考 |
|---|:---:|---|
| PublishTopic | ✅ | |
| CancelAction | ✅ | ROS 2 では action_msgs/srv/CancelGoal サービス経由でキャンセル |
| RecordAction | ✅ | |
| SelectPointCloudPublishAction | ✅ | |
| ObjectFitOperatorAction | ✅ | |
| RobotCommandInterfaceAction | ✅ | ROS 2 は構造体配列パラメータ非対応のため YAML ファイル（default_robot_command.yaml）で設定 |
| EmptyServiceCallInterfaceAction | ✅ | ROS 2 は構造体配列パラメータ非対応のため YAML ファイルで設定 |
| YesNoButton | ✅ | |
| TabletControllerPanel | ✅ | タブレット操作パネル（jsk_rviz_plugins_msgs 依存） |

## Tool（4）

| プラグイン名 | 移植状況 | 備考 |
|---|:---:|---|
| OverlayPicker | ✅ | |
| CloseAll | ✅ | |
| OpenAll | ✅ | |
| ScreenshotListener | ✅ | 画面キャプチャ系（X11セッションが必要） |

## ViewController（1）

| プラグイン名 | 移植状況 | 備考 |
|---|:---:|---|
| TabletViewController | ✅ | view_controller_msgs（stopgap）依存 |

## 補足（全体に関わる制約）

- **stopgap メッセージパッケージ**: `jsk_hark_msgs` / `jsk_footstep_msgs` / `people_msgs` / `view_controller_msgs` / `jsk_gui_msgs` は上流に正式な ROS 2 リリースが無いため、本リポジトリ内で暫定ビルドしています。正式リリースが出たら削除すべきです。
- **dynamic_reconfigure 廃止**: ROS 1 の `.cfg` はノードパラメータ + `on_set_parameters_callback` に置換されています（rqt_reconfigure 連携は無し）。
- **画面キャプチャ系**（RvizScenePublisher / VideoCapture / ScreenshotListener）は X11 セッションが前提です。
