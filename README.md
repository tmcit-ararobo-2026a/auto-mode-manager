# auto_mode_manager

STM32新仕様に対応するROS 2自動モードの独立状態管理パッケージ。
ナビゲーション・通信・認識・射出計算を調停し、各下位パッケージはmanagerに依存しない。

計画に従いタスク単位で実装する。T01aで単独起動できるHumble雛形を実装。Operationやアクチュエータ指令は配信しない。
未確定の要求識別・取消・品質・機構・校正条件は推測で補完しない。

## 起動と契約

`ros2 launch auto_mode_manager auto_mode_manager.launch.py`。
初期状態はStandby。未確定条件は [auto_mode_contract.md](docs/auto_mode_contract.md)。
`-DBUILD_RUNTIME_NODES=OFF` で実行ノードをビルドから分離する。
