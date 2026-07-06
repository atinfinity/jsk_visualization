# jsk_interactive_marker ROS 2 スモークテスト表

移植済みの sample launch を実機で起動し、**描画内容が想定通りか**を確認した結果。

- 確認方法: `DISPLAY=:1` で各 launch を起動し、RViz2 をフルスクリーン表示してスクリーンショットを目視確認（最終実施: 2026-07-06）。
- 「結果」列: ✅ = 想定通りに描画

| launch | 想定される描画内容 | 結果 |
|---|---|:---:|
| `marker_6dof_sample.launch.py` | 4 つの 6-DOF インタラクティブマーカー（cube / sphere / line / mesh）が map 原点に重畳表示。並進矢印（赤=X / 緑=Y / 青=Z）＋回転リング＋中央にメッシュ | ✅ |
| `bounding_box_marker_sample.launch.py` | `BoundingBoxArray` のサンプル 3 ボックス（寸法・位置が異なる青／橙／灰）がインタラクティブマーカーとして表示 | ✅ |
| `sample_transformable_markers_client.launch.py` | クライアントが YAML から登録した 3 つの transformable ボックス（ラベル dim_a/b/c、選択中は黄枠） | ✅ |
| `transformable_server_sample.launch.py` | 初期状態は空（設計通り）。rviz メニューまたは `request_marker_operate` サービスでオブジェクトを挿入すると描画される（torus 挿入で確認：map 原点に滑らかなトーラス、udiv/vdiv=100） | ✅ |
| `urdf_model_markers.launch.py` | `launch/models/sample_models.yaml` の 2 つの URDF モデルマーカー（sample.urdf：黄ベース＋紫アーム＋青球）＋オーバーレイのテキストラベル | ✅ |
| `sample_camera_info_publisher.launch.py` | RViz なし（ヘッドレス）。`CameraInfo` を配信（yaml 版: 1504×1504 / camera / plumb_bob、default 版: 640×480）＋カメラフレームのインタラクティブマーカー | ✅（トピックで確認） |

## 補足

- `transformable_server_sample` はデフォルトでマーカーを挿入しない設計（ROS 1 と同一：ノードはサーバ生成のみ）。空表示は正常で、サービス/メニュー挿入後にオブジェクトが描画される。
- `sample_camera_info_publisher` は launch に RViz を含まないため全画面表示の対象外。CameraInfo 配信をトピックで確認。
- 起動時に `urdf_model_markers` で一過性の TF 警告（`"map" ... does not exist`）が出ることがあるが、モデルは正常に描画・配置される（起動時の TF 公開レース）。
