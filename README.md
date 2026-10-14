# auto_mode_manager

STM32新仕様に対応するROS 2自動モードの独立状態管理パッケージ。
ナビゲーション・通信・認識・射出計算を調停し、各下位パッケージはmanagerに依存しない。

計画に従いタスク単位で実装する。T01aの雛形から契約v1 runtimeへ移行。通常起動はdisabledでOperationを配信しない。
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

`/auto_mode/status`にAutoModeStatusを20ms周期で公開する。disabled起動はStandby・自動出力無効。simulationはprofileと能力別可否を明示。
producer_sessionは起動ごとに変更し、update_sequenceは各スナップショットで進める。
SafetyStateの評価番号は実際の監視処理完了だけで進める契約。雛形は安全評価を配信しない。
この状態公開は基板実モード・Operation受理・物理停止の証拠ではない。

## 早期通信試験launch（T17a）

`ros2 launch auto_mode_manager integration_test.launch.py stage:=communication domain_id:=230`。
robot_portと基板fixtureだけを127.0.0.1/127.0.0.2で起動し、8/32/44byte・鮮度を確認する。
fixtureはBUILD_TESTING時のみインストールする。test_mode=true・ROS_LOCALHOST_ONLY=1・
専用domain [100,232]、stm32_test_接頭辞のノードだけを要求し、他のOperation生成元を拒否する。
物理宛先・本番domain・非testノード・非ゼロ指令を許可しない。実機・HIL・手動入力・機構は模擬しない。

eligible stageはcommunication／fixed_navigation／tracking_shot。ハンドstageは未実装のため拒否する。
本番launchへfixtureを混入させない。通信試験はT18aの故障注入試験と組み合わせる。

## 契約v1 runtime

主実行ファイルauto_mode_manager_nodeはscripts/auto_mode_manager_runtime.py（rclpy）へ移行した。
通常launchはmode=disabledでStandbyのみ。simulationは明示mode/profile_file、localhost DDS・専用domainを要求する。
150ms解除・Move消費・取消優先・session/世代・reset_fault・計画/Action調停を実装。
取消は即時lease失効後、未決goal受理・終端・zero/履歴消去を確認し、200ms超過でFail。
独立safety monitorはcostmap・危険領域・車体footprint・現在経路を評価する。
評価番号・元入力・期限・sessionをmanagerが検査し、停止・停滞・再起動で失効する。

Statusにprofile・能力・取消・抑制・実送信を公開。基板反映・物理停止はunknown、自動ハンドは無効。
navigation_hil/full_robotはE1〜E5の実測・外部適合がないため拒否する。

## 模擬結合（T16a/b・T17a）

`ros2 launch auto_mode_manager integration_test.launch.py stage:=fixed_navigation domain_id:=230`
またはstage:=tracking_shot。実装済みplanner/follower・品質monitor・安全monitor・射出計算を使い、
基板・Pose/センサー・対象・機構はBUILD_TESTING時だけ入るsimulation_fixture.pyで供給する。
演算／指令はloopbackだけ。simulation.json/quality/shotは合成値で実機承認を意味しない。
障害物cloudも元の時刻と測定TFで検査し、期限・costmap・危険領域・車体／経路の停止余裕を監視する。

単調時計timerでROS時計停止中も失効を評価し、逆行では取消完了後に新sessionのFailへ移る。
復旧・新品質だけで再開せず、解除観測とresetを要求する。Movingの射出補償とハンド・実機は未対応。
shot_calculation_enabledは計算入力の配信能力であり、実射出許可ではない。

検証は[契約v1実装記録](docs/contract_v1_verification.md)を参照。旧src/auto_mode_manager_node.cppは
T01aの参考ソースで、現在の主実行経路はscripts/auto_mode_manager_runtime.py。
