# auto_mode_manager

固定地点への移動、移動対象へのYaw追従、静止対象への射出計算を調整するROS 2ノードです。C++の`auto_mode_manager_node`が要求を受け取り、各計算ノードの結果を`/robot/operation`へまとめます。

## ビルド

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-up-to auto_mode_manager --cmake-args -DBUILD_GPU_NODES=OFF
source install/setup.bash
```

`BUILD_GPU_NODES=OFF`はCPUビルド用です。実機の自己位置推定・従来のLiDAR認識を同時にビルドする場合は、それぞれのビルド設定に合わせてください。旧版から更新するときは、managerの古いbuild/installを取り除いてからビルドし、端末も開き直してください。

## 起動

```bash
ros2 launch auto_mode_manager auto_mode_manager.launch.py
```

モードや承認profileの指定は不要です。起動直後から状態・指令・照準入力を20ms周期で配信します。必要な自己位置推定、navigation、認識、固定対象、射出計算、UDPノードは別途起動してください。

設定を変更する場合は[config/manager.yaml](config/manager.yaml)を参考にします。

```bash
ros2 launch auto_mode_manager auto_mode_manager.launch.py config:=/absolute/path/manager.yaml
```

`map_frame`／`base_frame`はTF、`track_topic`は`MovingBucketTrack`型の認識入力です。Yaw制御は`yaw_gain`、`max_yaw_speed_rad_s`、`yaw_acceleration_rad_s2`、角度変換は`angle_sign`／`angle_offset_deg`で設定します。既定の認識入力は従来の`/moving_bucket_detector/target`です。

## 動作

- Sleep／Standbyを受け取った後の新しいMoveで開始します。起動時のMove保持、同じMoveの継続受信では開始しません。
- 対象0〜6は固定対象の停止姿勢へ移動し、対象7は認識位置へのYaw追従を行います。Yaw追従のXY指令はゼロです。
- Sleep／Standbyで指令をゼロにして終了します。取消処理中のMoveは予約せず、中断後は新しいMoveが必要です。
- 射出解がない場合は射出速度だけをゼロにします。アームの自動シーケンスと射出トリガーは実装していません。

`/auto_mode/status`の状態・対象ID・理由で処理状況を確認できます。managerには安全・品質・鮮度の監視や`reset_fault`はありません。受信値を時間経過では失効させず、認識の明示的なinvalidや計算不能では該当する出力をゼロにします。

## 他パッケージとの接続

既存navigationへの接続に必要な`ControlLease`、`PathVelocity`と、UDP・射出計算のsession／generationは維持しています。待機中は`status=READY`でも、UDPノードが新しいproducerを受け付けられるようOperationは中立の`STANDBY`にします。固定移動前には、前のmanagerが残したFollowPathを一度取り消します。

この変更はmanagerだけが対象です。UDPノードの既定送信無効、`enable_transmission`／`physical_contract_confirmed`、watchdog、navigationや認識・自己位置・射出計算の内部条件は残ります。実機ではrobot-portの接続先・送信設定、固定対象と射出校正を各パッケージの手順で設定してください。
