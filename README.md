# auto_mode_manager

STM32新仕様に対応するROS 2自動モードの独立状態管理パッケージ。
ナビゲーション・通信・認識・射出計算を調停し、各下位パッケージはmanagerに依存しない。

計画に従いタスク単位で実装する。T01aで単独起動できるHumble雛形を実装。Operationやアクチュエータ指令は配信しない。
未確定の要求識別・取消・品質・機構・校正条件は推測で補完しない。

## 起動と契約

`ros2 launch auto_mode_manager auto_mode_manager.launch.py`。
初期状態はStandby。未確定条件は [auto_mode_contract.md](docs/auto_mode_contract.md)。
`-DBUILD_RUNTIME_NODES=OFF` で実行ノードをビルドから分離する。

## 状態・安全型（T02b）

`/auto_mode/status`にAutoModeStatusを20ms周期で公開する。雛形はStandby・自動出力無効・契約未確定。
producer_sessionは起動ごとに変更し、update_sequenceは各スナップショットで進める。
SafetyStateの評価番号は実際の監視処理完了だけで進める契約。雛形は安全評価を配信しない。
この状態公開は基板実モード・Operation受理・物理停止の証拠ではない。

## 早期通信試験launch（T17a）

`ros2 launch auto_mode_manager integration_test.launch.py stage:=communication domain_id:=230`。
robot_portと基板fixtureだけを127.0.0.1/127.0.0.2で起動し、8/32/44byte・鮮度を確認する。
fixtureはBUILD_TESTING時のみインストールする。test_mode=true・ROS_LOCALHOST_ONLY=1・
専用domain [100,232]、stm32_test_接頭辞のノードだけを要求し、他のOperation生成元を拒否する。
物理宛先・本番domain・非testノード・非ゼロ指令を許可しない。実機・HIL・手動入力・機構は模擬しない。

現段階のeligible stageはcommunicationのみ。固定移動・Tracking・ハンドstageは対応する
manager状態管理・仕様契約・安全監視の完了後に追加する。未完stageは起動前に拒否する。
本番launchへfixtureを混入させない。通信試験はT18aの故障注入試験と組み合わせる。
