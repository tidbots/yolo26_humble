# yolo26_humble

ROS2 Humble用のYOLO26物体検出パッケージ。ByteTrackトラッキングと検出安定化機能付き。

## 機能

- YOLO26物体検出（Ultralytics）
- ByteTrack / BoTSORT オブジェクトトラッキング
- 検出安定化:
  - バウンディングボックス平滑化（移動平均）
  - 信頼度平滑化
  - クラス予測安定化（モードロック）
  - ヒステリシス（出現/消失閾値）
- OpenCVデバッグウィンドウ表示
- Docker コンテナ化（GPU対応）
- Webカメラ入力対応（OpenCV + V4L2）

## クイックスタート

```bash
git clone https://github.com/tidbots/yolo26_humble.git
cd yolo26_humble

# Dockerイメージをビルド
docker compose -f compose.yaml --profile webcam --profile yolo --profile yolo-tracking build

# X11アクセスを許可（OpenCVデバッグウィンドウ用）
xhost +local:docker

# webcam + YOLO起動（トラッキング無効、15Hz）
docker compose -f compose.yaml --profile webcam --profile yolo up

# トラッキング有効で起動（ByteTrack、30Hz）
docker compose -f compose.yaml --profile webcam --profile yolo-tracking up
```

## Dockerプロファイル

| プロファイル | 説明 |
|-------------|------|
| `webcam` | USBカメラノード（OpenCV + V4L2） |
| `yolo` | YOLO検出（トラッキング無効、15Hz） |
| `yolo-tracking` | YOLO検出 + ByteTrack（30Hz） |

## 起動パラメータ

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `model` | 必須 | YOLOの.ptウェイトファイルパス |
| `image` | `/camera/image_raw` | 入力画像トピック |
| `detections` | `/yolo26/detections` | 出力検出トピック |
| `device` | `0` | GPUデバイスID または "cpu" |
| `conf` | `0.25` | 信頼度閾値 |
| `iou` | `0.45` | IoU閾値 |
| `rate` | `15.0` | 処理レート（Hz） |
| `transport` | `raw` | 画像トランスポート: "raw" or "compressed" |
| `publish_debug` | `true` | デバッグ画像をトピックに配信 |
| `cv_debug_window` | `true` | OpenCVデバッグウィンドウ表示（X11必要） |
| `tracking` | `false` | ByteTrackトラッキング有効化 |
| `tracker` | `bytetrack.yaml` | トラッカー: bytetrack.yaml or botsort.yaml |
| `smoothing` | `15` | 移動平均ウィンドウ（フレーム数） |
| `appear_frames` | `3` | 出現確認フレーム数 |
| `disappear_frames` | `5` | 消失確認フレーム数 |

## カスタムパラメータで実行

```bash
# トラッキング有効、30Hzで実行（コンテナに入らずに）
docker exec yolo26_ros2 bash -lc "source /ros2_ws/install/setup.bash && ros2 launch yolo26_ros2 yolo26.launch.py model:=/models/best.pt tracking:=true rate:=30.0"

# バックグラウンドで実行
docker exec -d yolo26_ros2 bash -lc "source /ros2_ws/install/setup.bash && ros2 launch yolo26_ros2 yolo26.launch.py model:=/models/best.pt tracking:=true rate:=30.0"

# 信頼度閾値を変更
docker exec yolo26_ros2 bash -lc "source /ros2_ws/install/setup.bash && ros2 launch yolo26_ros2 yolo26.launch.py model:=/models/best.pt conf:=0.5"
```

## モニタリング

### 検出結果（Detection2DArray）

ホスト側（ROS2インストール済みの場合）:
```bash
ros2 topic echo /yolo26/detections
```

コンテナから:
```bash
docker exec -it yolo26_ros2 bash -lc "ros2 topic echo /yolo26/detections"
```

### デバッグ画像

`cv_debug_window:=true`の場合、OpenCVデバッグウィンドウ「YOLO26 Debug」が自動で開きます。

または rqt_image_view を使用:
```bash
rqt_image_view /yolo26/debug_image
```

## モデルファイル

`/models/`に配置:
- `/models/best.pt` - YOLOウェイト
- `/models/classes.yaml` - クラス名（オプション）

事前学習モデルのダウンロード:
```bash
cd models
wget https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n.pt
wget https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26s.pt
wget https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26m.pt
wget https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26l.pt
wget https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26x.pt
```

## 検出安定化

`tracking:=true`の場合、以下の安定化が適用されます:

1. **バウンディングボックス平滑化**: `smoothing`フレームの移動平均
2. **信頼度平滑化**: 検出スコアの移動平均
3. **クラスロック**: トラック確認後、最頻クラスにロック
4. **ヒステリシス**:
   - 新規トラックは`appear_frames`連続検出で確認
   - 消失トラックは`disappear_frames`後に削除

これにより、照明変化や一時的な検出失敗によるちらつきが軽減されます。

## アーキテクチャ

```
/camera/image_raw → [Subscription] → [Frame Buffer] → [Timer Loop]
                                                           ↓
                                                    [YOLO Inference]
                                                           ↓
                                                    [ByteTrack] (optional)
                                                           ↓
                                                    [Stabilization]
                                                           ↓
                                    /yolo26/detections (Detection2DArray)
                                    /yolo26/debug_image (annotated Image)
```

## ライセンス

MIT
