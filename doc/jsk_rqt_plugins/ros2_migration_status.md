# jsk_rqt_plugins ROS 2 移植状況

`plugin.xml` に登録された rqt プラグイン全 **11 種**はすべて ROS 2 (Jazzy) へ移植済みです。

- 移植状況: ✅ = 移植完了
- 備考: ROS 2 固有の差異・依存・制約がある場合のみ記載（無印は標準的な移植）

| プラグイン名 | 移植状況 | 備考 |
|---|:---:|---|
| StatusLight | ✅ | UInt8 の値でライト色を表示（トピック選択後に購読開始） |
| StringLabel | ✅ | **ROS 2 はグローバルパラメータサーバが無いため、ROS パラメータ値の表示は不可。トピックの string フィールドのみ表示可** |
| ImageView2Plugin | ✅ | 移植した image_view2 ノードが必要。トピック選択（歯車→ダイアログ）後に marked 画像を表示（往復を実機確認済み） |
| ServiceButtons | ✅ | レイアウトは `layout_yaml_file` パラメータ（YAML）で指定。サービス呼び出しは非同期化 |
| ServiceRadioButtons | ✅ | 同上（ラジオボタン版） |
| ServiceTabbedButtons | ✅ | ROS 1 のネスト rosparam `~tabbed_layout` → `tabbed_layout_yaml_file`（YAML ファイル）に変更 |
| DRCEnvironmentViewer | ✅ | mini_maxwell。`std_msgs/Time` → `builtin_interfaces/Time` に変更 |
| Plot3D | ✅ | |
| HistogramPlot | ✅ | |
| Plot2D | ✅ | RANSAC 直線フィット（`--fit-line-ransac`）は sklearn（オプション依存, rosdep: `python3-sklearn`）が必要。未導入時は RANSAC のみスキップ |
| YesNoButton | ✅ | `/rqt_yn_btn` リクエスト到着までボタンは無効 |

## 補足

- **実行ファイル・サンプルも移植済み**: `rqt_2d_plot` / `rqt_3d_plot` / `rqt_histogram_plot` / `rqt_image_view2` / `rqt_service_buttons` / `rqt_tabbed_buttons` / `rqt_status_light` / `rqt_string_label` / `rqt_yn_btn` / `rqt_drc_mini_maxwell`（計 10）と、`sample_*_plot.py` / `sample_hist_pub.py` / `sample_service_buttons.py` などのサンプルスクリプト。
- **設定方式の変更**: ROS 1 のネスト rosparam / dynamic_reconfigure は、ROS 2 ではノードパラメータ（`layout_yaml_file` / `tabbed_layout_yaml_file` 等の YAML ファイルパラメータ）に置換。
- **サービス呼び出しの非同期化**: ボタン系プラグインのサービス呼び出しは `call_async` 化され、GUI 更新は Qt シグナル経由でスレッド安全に行う。
- **未移植の launch（3 件）**: `sample_drc_mini_maxwell.launch` / `sample_service_radio_buttons.launch` / `sample_service_buttons_wo_perspective.launch` は ROS 1 XML のまま（`.launch.py` 未変換）。プラグイン本体の機能には影響なし。
