# LiDAR–IMU外部パラメータ

`imu/extrinsic_R`と`imu/extrinsic_T`はLiDAR座標からIMU座標への変換です。

```text
p_I = R_IL * p_L + t_IL
```

- `extrinsic_R`: 行優先で並べた3×3回転行列。
- `extrinsic_T`: IMU座標で表したLiDAR原点の位置。単位はm。
- 入力点群はLiDAR座標、加速度・角速度はIMU座標を使用します。
  入力側ですでに座標変換している場合は、その座標系に対応した外部パラメータが必要です。

## 推定と点群

内部状態の姿勢・位置・速度はIMUを基準にします。歪み補正後の点群は
スキャン終了時のLiDAR座標に保持し、世界座標への変換で外部パラメータを適用します。

```text
p_W = R_WI * (R_IL * p_L + t_IL) + t_WI
```

点と平面の残差の姿勢ヤコビアンは、IMU座標の点`R_IL * p_L + t_IL`から計算します。
点の観測共分散もLiDAR座標からIMU座標へ回転します。地図点の世界座標共分散は、
姿勢・位置の相互共分散を含む状態共分散から計算します。

## 出力座標系

- `camera_init`: 初期IMU座標を基準にする世界座標系。
- `/aft_mapped_to_init`、`camera_init -> aft_mapped`のTF、`/path`:
  世界座標系におけるIMUの姿勢・位置。`aft_mapped`はIMU座標系です。
  推定状態の`rot_end`、`pos_end`をそのまま使用し、移動距離・診断用位置ログもIMU基準です。
- `/cloud_registered_surf`、`/cloud_effected`、ボクセル地図:
  同じ世界座標系に変換した点群・地図。
- KITTIログ: LiDAR姿勢を初期LiDAR座標に変換後、既存のKITTI用カメラ変換を適用。
  このカメラ変換はKITTI固有で、VLP-16用の一般的なカメラ校正ではありません。

オドメトリの基準点と点群の入力座標系は別です。オドメトリをIMU基準にしても、
点群の世界座標変換には`R_IL`と`t_IL`が必要です。LiDARの姿勢が必要な場合は、
`R_WL = R_WI * R_IL`、`t_WL = t_WI + R_WI * t_IL`で求めます。

## 回転行列の検証

YAMLの値は変更しません。実行時はSVDで最も近い正規直交回転行列を求め、
全処理で同じ回転を使用します。補正量のFrobeniusノルムが`1e-6`を超えると、
補正量と実際に使用する行列を警告ログに出します。Long Corridorの公開校正値も
この対象になります。

要素数の不一致、非有限値、非正の行列式、または`||RᵀR - I||F > 0.1`は
起動時にエラーとします。

## 検証

`test_lidar_imu_geometry`で既知の座標変換、静止時の不変性、数値微分との
ヤコビアン・共分散の一致、校正値の検証、実際のIMU処理を通した模擬スキャンの
歪み補正を確認します。模擬スキャンには回転・並進運動、時刻0、同時刻の複数点を含めます。

ROS環境を読み込んだcatkinワークスペースで実行できます。

```bash
catkin_make run_tests_voxel_map_plus_gtest_test_lidar_imu_geometry
catkin_test_results
```
