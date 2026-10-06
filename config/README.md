# Experimental Settings (SubT-MRS Multi Floor / Long Corridor)

| YAML File | Sequence | Preprocess |
|---|---|---|
| `vlp16_longcorridor_aug.yaml` | Long Corridor | Augmentation |
| `vlp16_longcorridor.yaml` | Long Corridor | None |
| `vlp16_multifloor_aug.yaml` | Multi Floor | Augmentation |
| `vlp16_multifloor.yaml` | Multi Floor | None |


| Section | Variable | Type | Value | Reference |
|---|---|---|---|---|
| `common` | `lid_topic` | string | --- | 入力点群トピックに応じて設定 |
| `common` | `imu_topic` | string | --- | 入力IMUトピックに応じて設定 |
| `preprocess` | `lidar_type` | int | `2` | Velodyne LiDARを使用 |
| `preprocess` | `scan_line` | int | `16` | VLP-16のスキャンライン数 |
| `preprocess` | `blind` | float | `0.2` | 近距離点群も使用するため |
| `preprocess` | `point_filter_num` | int | `1` | インデックスベース間引きを適用しない |
| `preprocess` | `calib_laser` | bool | `false` | KITTIデータセットではないため |
| `mapping` | `down_sample_size` | float | `0.5` | upstream `config/velodyne.yaml` |
| `mapping` | `max_iteration` | int | `3` | upstream `config/velodyne.yaml` |
| `mapping` | `voxel_size` | float | `0.5` | upstream `config/velodyne.yaml` |
| `mapping` | `plannar_threshold` | float | `0.01` | upstream `config/velodyne.yaml` |
| `mapping` | `max_points_size` | int | `100` | upstream `config/velodyne.yaml` |
| `mapping` | `update_size_threshold` | int | `5` | upstream `config/velodyne.yaml` |
| `mapping` | `sigma_num` | int | `3` | upstream `config/velodyne.yaml` |
| `noise_model` | `ranging_cov` | float | `0.02` | upstream `config/velodyne.yaml` |
| `noise_model` | `angle_cov` | float | `0.1` | upstream `config/velodyne.yaml` |
| `noise_model` | `acc_cov_scale` | float | `1.0` | upstream `config/velodyne.yaml` |
| `noise_model` | `gyr_cov_scale` | float | `0.5` | upstream `config/velodyne.yaml` |
| `imu` | `imu_en` | bool | `true` | IMUを使用するため|
| `imu` | `extrinsic_T` | float[3] | --- | [Long Corridor Extrinsics](https://drive.google.com/file/d/1bB3jfEJeTf_XoLUHKOaxCNF_MCkiQTol/view) |
|       |               |          | --- | [Multi Floor Extrinsics](https://drive.google.com/file/d/1BV87D60W35UGzIaHjKD64c_J1G0U70jf/view) |
| `imu` | `extrinsic_R` | float[9] | --- | [Long Corridor Extrinsics](https://drive.google.com/file/d/1bB3jfEJeTf_XoLUHKOaxCNF_MCkiQTol/view) |
|       |               |          | --- | [Multi Floor Extrinsics](https://drive.google.com/file/d/1BV87D60W35UGzIaHjKD64c_J1G0U70jf/view) |
| `visualization` | `pub_voxel_map` | bool | `false` | 評価時の不要な可視化処理を無効化 |
| `visualization` | `pub_voxel_map_period` | int | `50` | upstream `config/velodyne.yaml` |
| `visualization` | `pub_point_cloud` | bool | `false` | 評価時の不要な可視化処理を無効化 |
| `visualization` | `dense_map_enable` | bool | `false` | upstream `config/velodyne.yaml` |
| `visualization` | `pub_point_cloud_skip` | bool | `false` | upstream `config/velodyne.yaml` |
| `Result` | `write_kitti_log` | bool | `false` | KITTI形式ログを出力しない |
| `Result` | `result_path` | string | --- | `write_kitti_log=false` のため未使用 |
