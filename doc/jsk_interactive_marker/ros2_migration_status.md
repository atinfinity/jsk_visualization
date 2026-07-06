# jsk_interactive_marker ROS 2 移植状況

`jsk_interactive_marker` パッケージのノード（C++ 実行ファイル 15、Python スクリプト 6）はすべて ROS 2 (Jazzy) へ移植済みです。

- 移植状況: ✅ = 移植完了
- 備考: ROS 2 固有の差異・依存・制約がある場合のみ記載（無印は標準的な移植）

## C++ ノード（15）

| ノード名 | 移植状況 | 備考 |
|---|:---:|---|
| bounding_box_marker | ✅ | BoundingBoxArray を購読しインタラクティブマーカー化 |
| camera_info_publisher | ✅ | CameraInfo を配信。dynamic_reconfigure → ノードパラメータ。`yaml_filename` 対応 |
| door_foot | ✅ | 足領域マーカー。構造化 rosparam `foot_list` を YAML でパラメータ化 |
| footstep_marker | ✅ | フットステップのインタラクティブマーカー。dynamic_reconfigure → ノードパラメータ |
| interactive_marker_interface | ✅ | `mesh_config`（構造化 rosparam）→ `mesh_config_file`（YAML ファイルパラメータ） |
| interactive_point_cloud | ✅ | 点群のインタラクティブ選択 |
| marker_6dof | ✅ | 6DOF インタラクティブマーカー（cube/sphere/line/mesh） |
| point_cloud_config_marker | ✅ | 点群設定マーカー |
| pointcloud_cropper | ✅ | 点群クロップ。dynamic_reconfigure → ノードパラメータ |
| polygon_marker | ✅ | ポリゴンのインタラクティブマーカー |
| transformable_server_sample | ✅ | Transformable サーバのサンプル。初期状態は空（ROS 1 と同様、サービス/メニューでオブジェクト挿入） |
| triangle_foot | ✅ | 三角形の足領域マーカー |
| urdf_control_marker | ✅ | URDF 制御マーカー |
| urdf_model_marker | ✅ | URDF モデルマーカー。`model_config`（構造化 rosparam）→ `models_config_file`（YAML ファイルパラメータ） |
| world2yaml | ✅ | マーカー配置を YAML に出力 |

## Python スクリプト（6）

| ノード名 | 移植状況 | 備考 |
|---|:---:|---|
| transformable_markers_client.py | ✅ | Transformable マーカークライアント。`~server` remap → `server` パラメータに変更 |
| transformable_joy_configure.py | ✅ | Joy によるマーカー操作設定（テレオペ）。sample launch には未統合 |
| transformable_spacenav_configure.py | ✅ | SpaceNav によるマーカー操作設定（テレオペ）。sample launch には未統合 |
| dummy_camera.py | ✅ | ダミー CameraInfo パブリッシャ |
| semantic_robot_state_generator.py | ✅ | セマンティックなロボット状態生成 |
| joint_state_publisher.py | ✅ | 関節状態パブリッシャ（ヘルパ） |

## 補足（全体に関わる制約）

- **設定方式の変更**: ROS 1 の構造化 rosparam / dynamic_reconfigure は、ROS 2 ではノードパラメータ（`models_config_file` / `mesh_config_file` などの YAML ファイルパラメータ、`on_set_parameters_callback`）に置換されています。
- **`~private` remap の廃止**: 例として transformable_markers_client の `~server` remap は `server` パラメータに変更されています。
- **未移植の周辺要素（ノード本体には影響なし）**:
  - メタパッケージの `jsk_interactive` サブパッケージ（EusLisp ベースのロボット関節/ハンド操作）は `COLCON_IGNORE` でビルド対象外。
  - ロボット固有 launch（atlas / hrp2 / pr2 / staro / samplerobot）は、対応する robot description パッケージが ROS 2 未提供のため未移植。
  - テレオペ統合 launch（SpaceNav / KORG nanoKONTROL 等）は未移植（設定スクリプトは移植済みだが sample launch には未統合）。
