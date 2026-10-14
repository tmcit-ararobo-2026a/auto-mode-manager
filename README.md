# auto_mode_manager

STM32新仕様に対応するROS 2自動モードの独立状態管理パッケージ。
ナビゲーション・通信・認識・射出計算を調停し、各下位パッケージはmanagerに依存しない。

計画に従いタスク単位で実装する。T01aで単独起動できるHumble雛形を実装。Operationやアクチュエータ指令は配信しない。
要求識別・取消・Ready品質・機構停止・校正のROS契約v1を確定した。
実測profile・基板適合は未取得であり、仕様書の確定を実装・実機有効化と区別する。

## 起動と契約

`ros2 launch auto_mode_manager auto_mode_manager.launch.py`。
初期状態はStandby。規範契約と実機の残条件は [auto_mode_contract.md](../docs/auto_mode_contract.md)。
`-DBUILD_RUNTIME_NODES=OFF` で実行ノードをビルドから分離する。

## ROS契約v1（T00）

現STM32を維持する制限付き要求識別を採用。同じMoveは再送扱いとし、
再要求にはSleep/Stanbyの150ms以上の連続観測が必要。取消が新Move・到着より優先し、
通信復旧・品質回復だけでは再開しない。取消ROS終了確認の期限は200ms。

Readyは承認品質profileと鮮度・地図・チームの一致を必須とし、1秒以上かつ異なる計算結果10件以上で成立。
固定共分散を精度保証に使わない。Fail送信・ROS抑制・物理停止を別判定とする。
未原点・未校正・機構停止未確認の機能は実機では有効化しない。
各時間値はソフトウェア検証契約であり、実機の停止時間・品質精度を承認した値ではない。

契約書C1〜C7は今後の実装仕様。現コードは引き続きStandby公開と通信試験のみ。

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
