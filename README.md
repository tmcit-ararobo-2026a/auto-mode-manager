# auto_mode_manager

ROS 2の自動モードを管理するパッケージです。ナビゲーション・通信・認識・射出計算を調停し、操作要求に応じた状態遷移と自動指令の生成を行います。下位パッケージからmanagerへの依存はありません。

通常起動は自動出力が無効で、Standbyの状態だけを公開します。現在はsimulationでの固定対象への移動、移動対象のYaw追従、静止対象の射出解計算に対応しています。実機モード、自動ハンド、移動対象への射出補償は未対応です。

## ビルド

ROS 2 Humbleを使用します。通信・経路計画・自己位置状態・LiDAR認識・固定対象・射出計算の各パッケージを同じワークスペースに配置してください。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to auto_mode_manager --cmake-args -DBUILD_GPU_NODES=OFF
source install/setup.bash
```

上のコマンドはGPUなしの状態監視とsimulation用です。GPU認識ノードもビルドする場合は`BUILD_GPU_NODES`を有効にし、対象GPUのCUDA環境を用意します。型だけをビルドする場合は`-DBUILD_RUNTIME_NODES=OFF`を指定できます。

## 起動

```bash
ros2 launch auto_mode_manager auto_mode_manager.launch.py
```

既定は`mode:=disabled`、`profile_file`は空です。`/auto_mode/status`を20ms周期で公開し、`/robot/operation`を配信しません。主実行ファイル`auto_mode_manager_node`はrclpy runtimeです。

simulationで起動するには`mode:=simulation`と承認済みsimulationプロファイルの`profile_file`を明示し、localhostのDDS・専用ROS domainを使用します。各提供ノードと独立安全監視ノード`navigation_safety_monitor.py`も必要です。以下の結合試験launchは、それらをまとめて起動します。

## 状態と要求処理

`AutoModeStatus`はStandby、Ready、Moving、Tracking、Goal、Failの状態と、プロファイル・能力ごとの可否・取消・指令抑制・実送信の状況を公開します。`producer_session`は起動ごとに変更し、更新番号と世代で古い結果や指令を区別します。

同じMove要求は一度だけ消費します。再要求にはSleepまたはStandbyを150ms以上連続で観測する必要があり、起動時から保持されたMoveでは開始しません。取消は新しいMoveや到着より優先します。通信や品質の復旧だけでは再開しません。

取消時は直ちに制御leaseを失効させ、遅れて受理されたgoalも取り消します。Actionの終了、ゼロ速度、制御履歴の消去を確認し、200msを超えるとFailに移ります。

Readyには承認済み品質プロファイル、入力の鮮度、地図・チームの一致が必要です。品質は1秒以上かつ異なる計算結果10件以上の成立を要求し、固定共分散を精度保証には使いません。

原因を解消し、必要な品質・安全・機構条件と解除要求がそろった後、Failを解除します。

```bash
ros2 service call /auto_mode/reset_fault std_srvs/srv/Trigger '{}'
```

解除しても以前の動作を自動再開しません。

## システム構成

| インターフェース | 用途 |
| :--- | :--- |
| `/robot/command`、`/robot/feedback`、`/robot/communication_status` | 操作要求・基板情報・通信状態の受信 |
| `/localization/status`、`/field/targets`、`/moving_bucket_detector/target` | 自己位置品質と対象情報の受信 |
| `/navigation/compute_path`、`/navigation/follow_path` | 経路計画・追従の依頼 |
| `/navigation/control_lease`、`/navigation/path_velocity` | 制御許可と速度結果の受け渡し |
| `/auto_mode/aim_target`、`/shooter/solution` | 照準対象と射出解の受け渡し |
| `/auto_mode/status` | 調停状態・能力・取消状態の配信 |
| `/auto_mode/safety`、`/auto_mode/execution_path` | 独立安全評価と実行経路の受け渡し |
| `/robot/operation` | simulationでの動作指令の配信 |

安全監視は別プロセスで、costmap・障害物点群・危険領域・車体footprint・現在経路の停止余裕を評価します。評価番号は実際の評価完了ごとに進み、managerは元入力の時刻・期限・sessionを確認します。監視停止、評価停滞、再起動、入力断では許可を失効させます。

ROS時計が停止しても単調時計で期限を評価します。時計が逆行すると動作を取り消し、取消完了後に新sessionのFailへ移ります。Trackingでは並進速度を0に保ち、対象の鮮度とIDを確認してYaw速度だけを生成します。

UDP送信成功、基板反映、物理停止は別の状態です。現在、基板反映・物理停止はunknownです。`shot_calculation_enabled`は計算入力の配信能力を示し、実射出の許可を意味しません。

## 結合試験

結合試験の`test/`ファイルとfixtureはローカル管理です。GitHubからの取得だけでは含まれません。ローカルのファイルを用意し、`BUILD_TESTING=ON`でビルドすると試験とfixtureを利用できます。

```bash
# UDP通信の確認
ros2 launch auto_mode_manager integration_test.launch.py stage:=communication domain_id:=230

# 固定対象への経路計画・追従
ros2 launch auto_mode_manager integration_test.launch.py stage:=fixed_navigation domain_id:=230

# Yaw追従と静止対象の射出解計算
ros2 launch auto_mode_manager integration_test.launch.py stage:=tracking_shot domain_id:=230
```

launchは`ROS_LOCALHOST_ONLY=1`と専用domain（100〜232）を設定し、UDPは127.0.0.1/127.0.0.2で使用します。communicationでは中立指令だけを扱います。ほかのstageでは実際のplanner/follower・品質監視・安全監視・射出計算を使用し、基板・Pose・センサ・対象・機構はfixtureで模擬します。`/auto_mode/simulation_state`は模擬状態であり、実機の計測結果ではありません。

[simulation設定](config/simulation.json)、[品質設定](config/simulation_quality.json)、[射出設定](config/simulation_shot.json)は合成値です。実機の校正・原点・到達・停止・基板適合の承認へ転用しません。`navigation_hil`・`full_robot`は起動を拒否し、hand stageも未対応です。

```bash
colcon test --packages-select auto_mode_manager
colcon test-result --verbose
```

詳細な仕様と検証記録はローカルの`src/docs/`で管理します。仕様書の正本は`src/docs/auto_mode_contract.md`です。
