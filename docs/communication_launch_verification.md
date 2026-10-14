# 通信段階launchの検証（T17aの通信範囲）

実施日: 2026-10-14（Asia/Tokyo）。作業ディレクトリ: `/home/lambda/ros2_ws/src`。
対象: auto-mode-manager feat/1-manager-foundation（ddf14b1からT17a差分）、robot-port feat/10-stm32-new-udp（5714916からライブラリinstall修正）。

Humbleのsetupと `/tmp/stm32-ros2/install/setup.bash` をsourceして、
`colcon build --packages-select robot_port auto_mode_manager --build-base /tmp/stm32-ros2/build --install-base /tmp/stm32-ros2/install`。
ビルド: 成功／終了コード0。`ctest --test-dir /tmp/stm32-ros2/build/auto_mode_manager --output-on-failure`。
試験: 成功／終了コード0、2件。Standby雛形と実際のros2 launch経由の8/32/44byte送受信・鮮度・送信成功を確認。
本番domain／test_modeなしのfixture、未完stageは拒否。127.0.0.1/127.0.0.2・専用DDS domain、ROS_LOCALHOST_ONLY=1。

初回・再試行: 失敗／ctest終了コード8。インストール済みudp_nodeがlibudp_channel.soをロードできずHealth未配信。
robot-portでudp_channelのinstall/exportを追加。従来のbuildディレクトリ上の単体試験では検出できなかった実行時依存を解消した。
修正後のlaunch試験成功。ログ: `/tmp/stm32-ros2/runtime-fix-build.log`、`manager-t17a-launch-retry.log`（失敗）、`manager-t17a-final-tests.log`（成功）。

未実施: manager要求処理・安全監視・固定移動・Tracking・ハンド段階・HIL・実機。
通信以外のstageは起動前に拒否する。全T17a・全体受入の完了ではない。fixtureは本番launchに含めずBUILD_TESTING時だけインストール。
