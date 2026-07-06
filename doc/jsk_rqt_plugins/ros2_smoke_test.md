# jsk_rqt_plugins ROS 2 スモークテスト表

全 11 プラグインを実機で起動し、**描画内容が想定通りか**を確認した結果。

- 確認方法: `DISPLAY=:1` で各プラグインを起動し、rqt ウィンドウをフルスクリーン表示してスクリーンショットを目視確認（最終実施: 2026-07-06）。
- 「結果」列: ✅ = 想定通りに描画

| プラグイン | 想定される描画内容 | 結果 |
|---|---|:---:|
| Plot2D | `sample_2d_plot.py` の正弦波データ（青線）＋フィット直線（緑=最小二乗、赤=RANSAC）を凡例付きで描画 | ✅ |
| Plot3D | `sample_3d_plot.py` の 3 系列を 3D 塗りつぶしポリゴンで描画＋凡例 | ✅ |
| HistogramPlot | `sample_hist_pub.py` の配列データを正規分布ヒストグラム（赤）で描画 | ✅ |
| ServiceButtons | `service_button_layout.yaml` の 2 列 6 ボタン（A=画像＋サービス名、C=押下状態、D/E/F） | ✅ |
| ServiceRadioButtons | 6 個のラジオボタン（A=画像、B/C、D/E/F）。tabbed の radio タブで確認 | ✅ |
| ServiceTabbedButtons | 「push button」「radio button」の 2 タブ、各タブに 6 ボタンレイアウト | ✅ |
| ImageView2Plugin | トピック選択（歯車→ダイアログ）後、image_view2 の marked 画像を表示 | ✅ |
| StatusLight | ライトウィジェット（トピック選択前は灰色「Unknown」の円） | ✅ |
| StringLabel | ラベルウィジェット（選択前は空。トピック選択後に string フィールドを表示） | ✅ |
| YesNoButton | プロンプト「Select yes or no.」＋下部の Yes/No ボタン（リクエスト到着前は無効） | ✅ |
| DRCEnvironmentViewer | `sample_drc_mini_maxwell.py` のネットワーク状態フェイス（例: シアン背景に「:)」）を描画 | ✅ |

## 補足

- `StatusLight` / `StringLabel` / `YesNoButton` の初期状態はデータ待ちのアイドル表示（トピック選択やサービス要求の到着で更新される）。空表示は仕様。
- `ImageView2Plugin` はトピック選択が GUI 操作（歯車→ダイアログ）で必要。選択後に `<topic>/marked`（image_view2 の出力）を表示。
- `StringLabel` は ROS 2 にグローバルパラメータサーバが無いため、ROS パラメータ値の表示は非対応（トピックの string フィールドのみ。起動時にその旨の警告ログを出す）。
- `Plot2D` の RANSAC フィット（赤破線）は sklearn（rosdep: `python3-sklearn`）に依存。未導入時は RANSAC のみスキップされる。

### テスト環境上の注意
- rqt には SIGTERM シャットダウン時のハングがある（`rqt_plot` で再現）。`timeout -k` を使い、exit 137 は正常として扱う。
- SIGKILL の連発は FastDDS の共有メモリを破壊し discovery を壊すことがある。テスト時は `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` で回避可能。
