# yolo26_humble



git clone
cd
docker compose build

起動コマンド
## 1) YOLOノードだけ起動（GPU想定）
```
docker compose -f compose.yaml --profile yolo26_ros2 up
```

## 2) USBカメラも一緒に起動（/dev/video0）
```
docker compose -f compose.yaml --profile webcam up 
```

## 3) バックグラウンド起動（-d）
```
docker compose -f compose.yaml --profile webcam up -d 
```


## 検出結果（Detection2DArray）
ホスト側に ROS2 が入っていればホストから：
```
ros2 topic echo /yolo26/detections
```

コンテナ内で確認するなら：
```
docker exec -it yolo26_ros2 bash -lc "ros2 topic echo /yolo26/detections"
```

## デバッグ画像（rqt）
ホスト側で：
```
rqt_image_view /yolo26/debug_image
```

## モデル・クラス名ファイルの置き場
- /models/best.pt
- /models/classes.yaml
```
cd models
wget https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n.pt
wget https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26s.pt
wget https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26m.pt
wget https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26l.pt
wget https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26x.pt
```

