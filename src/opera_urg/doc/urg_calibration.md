# URG 校正手順

URG_10 は 192.168.0.10、URG_11 は 192.168.0.11 です。センサの +X は正面、+Y は左です。角度の正方向は反時計回りです。

校正のあいだは `urg_rviz.launch.py` を起動しないでください。同じ URG に二重に接続します。

## 1. 設置

2台を 5 m 離します。URG_11 から URG_10 を見た方向が、世界座標の +X とほぼ同じになるように置きます。URG_10 が +X 側、URG_11 がそこから約 5 m 戻った位置です。

各 URG の正面（センサの +X）を、世界座標の +Y に向けます。この向きなら、センサの +Y（左）は世界座標の -X 側を向きます。

各 URG の高さは、計測するターゲット板の高さに合わせます。

## 2. 電源とネットワーク

各 URG に 12 V を接続します。各 URG のイーサネットと PC を、同じイーサネット HUB に接続します。PC は 192.168.0.0/24 のアドレスにしてください。

## 3. 接続確認

```bash
ping -c 3 192.168.0.10
```

```bash
ping -c 3 192.168.0.11
```

どちらも応答があれば接続できています。

## 4. 校正ターゲット

ターゲットの長手方向が、各センサの +Y から ±10° 以内になるように置きます。`urg_calib.yaml` の `max_angle_from_plus_y_deg` が 10 のとき、この範囲の外の線分は捨てます。

ターゲットの中心の世界座標 (x, y) と、板の長手方向を測ります。角度は世界の +X から反時計回りです。向きは、センサの +Y に近い側に合わせます。逆に書くと、算出される URG の向きが 180° ずれます。

## 5. line_world.yaml

`src/opera_urg/config/line_world.yaml` に、測った値を書きます。x、y はメートル、yaw はラジアンで、-π 以上 π 未満です。度で測った角度は `測った度 × 3.14159265359 / 180` にして書きます。

```yaml
line_world:
  x: 0.0
  y: 0.0
  yaw: 0.0
```

## 6. 位置を 0 にリセット

表示を確認し、`y` を入力します。

```bash
source /opt/ros/humble/setup.bash
source /home/fukase/opera_urg_ws/install/setup.bash
ros2 run opera_urg urg_position_reset.py
```

## 7. 抽出範囲の確認

`src/opera_urg/config/urg_calib.yaml` の範囲はセンサ座標で、境界は含みません。ターゲットがこの中に入っていることを確認します。

- 192.168.0.10 は `0.05 < x < 3.0`、`0.0 < y < 4.0`（正面から左）
- 192.168.0.11 は `0.05 < x < 3.0`、`-4.0 < y < 0.0`（正面から右）

線分の長さは 0.75 m、許容は ±0.10 m、センサ +Y からの角度は ±10°、点の間隔は 0.05 m です。購読時間は 5 秒です。

## 8. URG を起動

この端末は開いたままにします。

```bash
source /opt/ros/humble/setup.bash
source /home/fukase/opera_urg_ws/install/setup.bash
ros2 launch opera_urg urg_publish.launch.py
```

## 9. 校正を実行

別の端末で実行します。5 秒間スキャンを受け、各 URG の世界座標での中心と X 軸の角度を表示します。

```bash
source /opt/ros/humble/setup.bash
source /home/fukase/opera_urg_ws/install/setup.bash
ros2 run opera_urg urg_calibrate.py
```

## 10. 結果を書き込む

表示が、測った配置と合っていることを確認します。正面を世界の +Y に向けていれば、yaw は約 1.57 rad（90°）です。2台の X は約 5 m 離れ、URG_10 の X が URG_11 より大きい側です。

合っていれば `y` を押します。`src/opera_urg/config/urg_position.yaml` が上書きされます。`y` 以外では書き込みません。

新しい位置を配信に使うときは、手順 8 の起動を一度止めて、同じコマンドで起動し直します。
