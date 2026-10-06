# Experimental Settings (SubT-MRS Multi Floor / Long Corridor)

| YAML File | Sequence | Preprocess |
|---|---|---|
| `vlp16_longcorridor_aug.yaml` | Long Corridor | Augmentation |
| `vlp16_longcorridor.yaml` | Long Corridor | None |
| `vlp16_multifloor_aug.yaml` | Multi Floor | Augmentation |
| `vlp16_multifloor.yaml` | Multi Floor | None |


| Section | Variable | Type | Value | Reason |
|---|---|---|---|---|
| `common` | `lid_topic` | string | `/velodyne_points` / `/augmented_cloud` | 入力点群トピックに応じて設定 |
| `common` | `imu_topic` | string | `/imu/data` | 入力IMUトピックに応じて設定 |
| `preprocess` | `lidar_type` | int | `2` | Velodyne LiDARを使用 |
| `preprocess` | `scan_line` | int | `16` | VLP-16のスキャンライン数 |
| `preprocess` | `blind` | float | `0.2` | 近距離点群も使用するため |
| `preprocess` | `point_filter_num` | int | `1` | インデックスベース間引きを適用しない |
| `preprocess` | `calib_laser` | bool | `false` | KITTIデータセットではないため |
| `mapping` | `down_sample_size` | float | `0.5` | upstream `config/velodyne.yaml` の既定値 |
| `mapping` | `max_iteration` | int | `3` | upstream `config/velodyne.yaml` の既定値 |
| `mapping` | `voxel_size` | float | `0.5` | upstream `config/velodyne.yaml` の既定値 |
| `mapping` | `plannar_threshold` | float | `0.01` | upstream `config/velodyne.yaml` の既定値 |
| `mapping` | `max_points_size` | int | `100` | upstream `config/velodyne.yaml` の既定値 |
| `mapping` | `update_size_threshold` | int | `5` | upstream `config/velodyne.yaml` の既定値 |
| `mapping` | `sigma_num` | int | `3` | upstream `config/velodyne.yaml` の既定値 |
| `noise_model` | `ranging_cov` | float | `0.02` | upstream `config/velodyne.yaml` の既定値 |
| `noise_model` | `angle_cov` | float | `0.1` | upstream `config/velodyne.yaml` の既定値 |
| `noise_model` | `acc_cov_scale` | float | `1.0` | upstream `config/velodyne.yaml` の既定値 |
| `noise_model` | `gyr_cov_scale` | float | `0.5` | upstream `config/velodyne.yaml` の既定値 |
| `imu` | `imu_en` | bool | `true` | SubT-MRSデータセット参照 |
| `imu` | `extrinsic_T` | float[3] | `[0.08, 0.029, 0.03]` | SubT-MRSデータセット参照 |
| `imu` | `extrinsic_R` | float[9] | `[0.999212900, -0.000519121, 0.004000000, 0.000516111, 0.999218492, -0.000939132, -0.004000000, 0.000802565, 0.999993652]` | SubT-MRSデータセット参照 |
| `visualization` | `pub_voxel_map` | bool | `false` | 評価時の不要な可視化処理を無効化 |
| `visualization` | `pub_voxel_map_period` | int | `50` | upstream `config/velodyne.yaml` の既定値 |
| `visualization` | `pub_point_cloud` | bool | `false` | 評価時の不要な可視化処理を無効化 |
| `visualization` | `dense_map_enable` | bool | `false` | upstream `config/velodyne.yaml` の既定値 |
| `visualization` | `pub_point_cloud_skip` | bool | `false` | upstream `config/velodyne.yaml` の既定値 |
| `Result` | `write_kitti_log` | bool | `false` | KITTI形式ログを出力しない |
| `Result` | `result_path` | string | upstream default | `write_kitti_log=false` のため未使用 |
