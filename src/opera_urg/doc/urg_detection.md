# URG による油圧ショベルの位置・方向検出

このプログラムは、油圧ショベルの BaseLink（走行系）の位置と方向を検出します。

油圧ショベルのクローラ側面に、規定の長さのターゲット板を取り付けます。クローラから、車軸と直角の方向へ少し離した位置に、URG を 2 台、適切な間隔をあけて設置します。2 台のスキャン点群を解析し、ターゲット板の位置と方向をリアルタイムに検出します。検出結果の位置は、ターゲット板の中心です。RViz の矢印は、板の長手方向に対して直角です。

URG の設置と校正が終わってから、検出表示プログラムを実行します。校正の手順は [URG 校正手順](urg_calibration.md) です。

## 検出表示

校正で書いた `urg_position.yaml` を使うため、配信側を起動し直します。一方の端末は開いたままにします。

```bash
source /opt/ros/humble/setup.bash
source /home/fukase/opera_urg_ws/install/setup.bash
ros2 launch opera_urg urg_publish.launch.py
```

別の端末で、検出と RViz を起動します。Fixed Frame は `world` です。2 台の点群、検出した線分、直角の矢印が表示されます。

```bash
source /opt/ros/humble/setup.bash
source /home/fukase/opera_urg_ws/install/setup.bash
ros2 launch opera_urg urg_rviz.launch.py
```

## rosbag

点群と検出値を記録します。ファイル名は `年月日時分_タイトル` です。先に上の 2 つの起動をしてから、別の端末で関数を定義します。

```bash
record_urg_bag() {
  source /opt/ros/humble/setup.bash
  source /home/fukase/opera_urg_ws/install/setup.bash
  cd /home/fukase/opera_urg_ws
  ros2 bag record -o "$(date +%Y%m%d_%H%M)_$1" \
    /urg_10/points /urg_11/points \
    /line_center /line_points \
    /tf_static
}
```

タイトルを引数にして実行します。止めるときは Ctrl+C です。

```bash
record_urg_bag 反射番検出実験
```

例として、`/home/fukase/opera_urg_ws/20261028_1503_反射番検出実験` ができます。
