# jsk_rviz_plugins ROS 2 スモークテスト表

全49プラグインクラスの動作確認状況。
「ロード」列は `test_plugin_load` gtest(pluginlibでの生成+破棄、ViewControllerはライブラリ解決のみ)により**CIで毎回自動確認**される。
「描画」列は実データを流した rviz2 での目視/無エラー確認。
最終実施: 2026-07-06、DISPLAY=:1 で全サンプルlaunch(15)+パネル2種(robot_command_interface / service_call)+ contact_state_marker / face_detector を RViz2 全画面表示で再確認し、全て想定通りに描画されることをスクリーンショットで確認した(描画の回帰なし)。

2026-07-05 の全画面再確認で3件の不具合を発見・修正した(commit 0193916): PictogramArray(config の Fixed Frame が map になっており base_link の pictogram が未表示 → base_link に修正)、LinkMarker(publisher が cylinder プリミティブで AttributeError クラッシュ → box/cylinder/sphere 対応に一般化)、OverlayCamera(オーバーレイパネルはレンダーターゲットテクスチャをサンプルできず黒 → カメラ画像テクスチャを直接表示、TF非依存化)。

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
| OverlayDiagnostic | ✅ | ✅ | diagnostics_sample.py + SAC円の描画確認済(2026-07-04) |
| OverlayImage | ✅ | ✅ | camera_sample.py の画像描画確認済 |
| TargetVisualizer | ✅ | ✅ | PoseStamped配信でターゲットマーク描画確認済 |
| Pictogram | ✅ | ✅ | ハートピクトグラム描画確認済 |
| PictogramArray | ✅ | ✅ | `pictogram_sample.launch.py` |
| Diagnostics | ✅ | ✅ | diagnostics_sample.py で3D円+テキスト描画確認済 |
| TFTrajectory | ✅ | ✅ | 動的TFで円軌跡の描画確認済 |
| TwistStamped | ✅ | ✅ | 矢印描画確認済 |
| PoseArray | ✅ | ✅ | 矢印群描画確認済(非推奨、rviz標準を推奨) |
| CameraInfo | ✅ | ✅ | `overlay_camera_sample.launch.py` |
| NormalDisplay | ✅ | ✅ | `normal_sample.launch.py` |
| PeoplePositionMeasurementArray | ✅ | ✅ | リングマーカー描画確認済 |
| BoundingBox | ✅ | ✅ | 単体ボックス描画確認済 |
| BoundingBoxArray | ✅ | ✅ | `boundingbox_sample.launch.py` |
| TorusArray | ✅ | ✅ | `torus_array_sample.launch.py` |
| SegmentArray | ✅ | ✅ | `segment_array_sample.launch.py` |
| HumanSkeletonArray | ✅ | ✅ | ボーン描画確認済 |
| SimpleOccupancyGridArray | ✅ | ✅ | `occupancy_grid_sample.launch.py` |
| PolygonArray | ✅ | ✅ | `polygon_array_sample.launch.py` |
| Footstep | ✅ | ✅ | `footstep_sample.launch.py` |
| AmbientSound | ✅ | ✅ | HarkPower極座標プロット描画確認済 |
| RvizScenePublisher | ✅ | ✅ | /rviz/image 31Hz・実描画内容の配信確認済(Ogre直読み化) |
| VideoCapture | ✅ | ⬜ 手動 | X11必須(start captureでaviが生成されるか) |
| QuietInteractiveMarker | ✅ | ✅ | IMサーバのキューブ描画確認済 |
| OverlayCamera | ✅ | ✅ | `overlay_camera_sample.launch.py` |

## Panels

| クラス | ロード | 動作 | 確認方法 |
|---|---|---|---|
| PublishTopic | ✅ | ⬜ 手動 | パネル追加→topic名入力→`ros2 topic echo` |
| CancelAction | ✅ | ⬜ 手動 | 実行中のROS 2アクションに対してcancel |
| RecordAction | ✅ | ✅ | Recordクリック→/record_command受信確認済 |
| SelectPointCloudPublishAction | ✅ | ⬜ 手動 | SelectionManager API変更のため要重点確認 |
| ObjectFitOperatorAction | ✅ | ✅ | Ontoクリック→/object_fit_command受信確認済 |
| RobotCommandInterfaceAction | ✅ | ✅起動 | `robot_command_interface_sample.launch.py`(ボタン表示まで確認) |
| EmptyServiceCallInterfaceAction | ✅ | ✅起動 | `service_call_panel.launch.py` |
| YesNoButton | ✅ | ✅ | サービス呼び出し→Yesクリック→yes=true応答確認済 |
| TabletControllerPanel | ✅ | ✅ | UI描画+/Tablet/*トピック生成確認済 |

## Tools / ViewController

| クラス | ロード | 動作 | 確認方法 |
|---|---|---|---|
| OverlayPicker | ✅ | ⬜ 手動 | overlay_sample起動中にツール選択→オーバーレイをドラッグ |
| CloseAll / OpenAll | ✅ | ✅ | クリックでツリー全開閉確認済 |
| ScreenshotListener | ✅ | ✅ | サービス→実描画PNG保存確認済(Ogre直読み化、X11不問) |
| TabletViewController | ✅(解決のみ) | ✅ | CameraPlacement配信でカメラ移動+current_camera_placement配信確認済 |

## 残りの手動確認項目(2026-07-04時点)

- **VideoCapture**: 「start capture」チェック→数秒→解除で `filename` のaviが生成されるか(キャプチャ経路はRvizScenePublisherと同一実装で検証済み)
- **OverlayPicker**: ツール選択→オーバーレイをドラッグ(移動には対象displayの「Overtake Position Properties」有効化が必要)
- **PublishTopic**: トピック名入力→ボタン→`ros2 topic echo`
- **CancelAction**: 実行中のROS 2アクションを追加→cancelボタン→goalがキャンセルされるか
- **SelectPointCloudPublishAction**: Selectツールで点群選択→publish(SelectionManager API変更のため要重点確認)

## 既知の制約

- 画面キャプチャ系(ScreenshotListener / VideoCapture / RvizScenePublisher)はOgreレンダーターゲット直読みのためX11/Wayland不問になった。
- ViewControllerは rviz_common の設計上、rviz外でのインスタンス化テストができない(基底デストラクタが `context_` を無条件参照)。
- `contact_state_marker` 系スクリプトは hrpsys_ros_bridge (ROS 1専用) 待ちで未移植。
