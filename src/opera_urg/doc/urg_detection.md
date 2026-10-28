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

記録した点群と検出結果を見るときは、実機の起動を止めます。`urg_publish.launch.py` と `urg_rviz.launch.py` が動いていると、同じトピックに実機のデータが混ざります。動いていれば、それぞれの端末で Ctrl+C します。

一方の端末で RViz を開きます。Fixed Frame は `world` です。

```bash
source /opt/ros/humble/setup.bash
source /home/fukase/opera_urg_ws/install/setup.bash
ros2 run rviz2 rviz2 \
  -d /home/fukase/opera_urg_ws/src/opera_urg/rviz/urg_world.rviz \
  --ros-args -p use_sim_time:=true
```

RViz が開いてから、別の端末で再生します。`--clock` により、RViz の時刻が記録時の時刻になります。フォルダ名は記録した `年月日時分_タイトル` に置き換えます。

```bash
source /opt/ros/humble/setup.bash
ros2 bag play /home/fukase/opera_urg_ws/20261028_1503_反射番検出実験 --clock
```

再生が終わると点群は消えます。もう一度見るときは、同じ `ros2 bag play` を再実行します。繰り返し再生するときは末尾に `--loop` を付けます。再生中の端末で Space を押すと一時停止し、もう一度押すと再開します。

`/urg_10/points`、`/urg_11/points`、`/line_points`、`/line_center` が記録どおりに出ます。検出プログラムは起動しません。bag 側に検出結果が入っているためです。
