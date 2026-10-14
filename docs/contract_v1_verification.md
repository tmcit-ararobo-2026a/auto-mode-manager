# manager契約v1の実装・模擬受入

実施日: 2026-10-14（Asia/Tokyo）。作業ディレクトリ: `/home/lambda/ros2_ws/src`。
対象: feat/3-manager-contract-v1、基準dfab972からT10・T12・T14・T16a/b・T17aの差分。
依存: robot-port feat/12-bridge-contract-v1、navigation feat/12-navigation-contract-v1、localization feat/12-localization-contract-v1、shooter fix/3-installed-runtime。未マージの基盤PRから最新先頭をfetchして分岐した。

## 検証コマンドと結果

Humble setup、CPU/CUDAなしの提供側6パッケージを `/tmp/manager-v1/build`／installへcolcon build。
そのinstallをsourceして、`colcon build --packages-select auto_mode_manager --build-base /tmp/manager-v1/build --install-base /tmp/manager-v1/install`。
ビルド成功／終了コード0。最終compile/installの後、`ctest --test-dir /tmp/manager-v1/build/auto_mode_manager --output-on-failure`。
7試験と追加の安全geometry単体試験1件成功／終了コード0（core内5ケース・非同期イベント内2ケース・Yaw内1ケースを含む）。
証跡: manager-acceptance-build.log／manager-post-build-tests.log、各提供側のquality-clock-final-tests.log・navigation-clock-final-tests.log・bridge-post-build-tests.log・shooter-install-tests.log（全て/tmp/manager-v1）。
Python構文・相対リンク・Git差分検査は共通verificationへ記録する。

## 受入例との対応

- A01/A02: 起動Move拒否、保持100回・Goal後のMove消費、150ms解除境界、品質不足時の消費、取消中の別Moveは予約しない。
- A03: 取消に対する遅延goal受理は取消し、古い成功・計画を採用しない。Action終端だけでは消去完了と扱わない。実followerをSIGSTOPして取消200ms超過のFailとzeroを検証。
- A04: 実Trackの模擬入力からXY zero/Yawを確認。取消観測後の保持Move・短い解除で再開せず、Track断でFail。基板が送らない取消の検出は提供せずE1未受入。
- A05: localization所有の品質AND、1秒・10結果、重複・fallback・閾値・識別不一致・同一計測Pose/TF・session・固定共分散除外は提供側試験を参照。
- A06: safetyプロセスSIGSTOP、map/obstacle入力停止、ROS時計停止／逆行でも許可失効。復旧だけで再開せず、品質再成立・解除・明示resetを確認。
- A07/A10: mock機構不許可でFail、全statusでarm無効、board_applied/physical_stop_confirmedはunknown。高さ0や新feedbackを物理停止・原点validにしない。
- A08: 実shooter計算の固定対象解とloopback belt値、移動対象は無効で旧値不採用。実LUT・trigger遮断・電圧補償の適合は未受入。
- A09: TargetSet revision変更でFail、angle符号・±20外・0.5丸め・非有限をcore試験で確認。実測地図・照準の承認は未受入。

試験はlocalhost DDS／専用domain、127.0.0.1/127.0.0.2のUDPだけ、合成データ。これはIsaac Simでの検証ではない。

## 失敗と解消

- core初回: 失敗。テストの解除helperが「goalなし取消」の終了確認を供給していなかった。helperで明示ack後、5ケース成功。
- 品質ビルド: 失敗（終了コード2）。Humble Timeのto_msg API不存在。ローカルヘッダに従い変換operatorへ修正。品質の初回試験は古い生成型のpose欠落で失敗（CTest8）、ビルド・新install後に成功。
- 実launch初回: 失敗（CTest8）。shooter共有ライブラリがinstallされず起動127。ライブラリexport修正後、再試験成功。終了済みprocessへのkillpgが元失敗を覆うテストcleanupも修正。
- 検証順序: 初期の試験にはcompile/install完了前に開始したものがある。最終証跡は全ビルド完了後に再実行したmanager-post-build-tests.logと提供側ログを使用し、旧成果物の成功を新成果物へ転記しない。

## 未実施・残条件

E1〜E5の基板適合・実測profile・実機Ready・原点／機構停止・自動ハンド・移動対象射出・HIL・GPU実行・bag精度・T19: 未実施／未承認。
navigation_hil/full_robotは起動拒否。実機の制動時間、Nodeスケジューリング、手動入力残存を模擬結果から保証しない。
共通docsの契約正本は維持し、ROS実装と実機受入の状態を分ける。

## CI再実行時の起動待ち修正

初回push CI（run38583157831）は成功。PR CI（run38584094002）は失敗。
品質monitorが先に安定した時点で、manager statusの初受信を待たず空リストを参照したIndexError。
初期waitを品質・manager status・Operationの全受信へ修正し、失敗時に合成launchログを表示する。
動作の条件や期限を緩めず、固定sleepによる試験回避を行わない。再試験とCI結果は共通current-stateへ記録する。

2026-10-15のPR再CI（run38585946956）は時計試験の最終generation==1で失敗。
新sessionは1から始まるが、復帰の解除通知で実行失効番号が進むため、reset後の値を1に固定する期待が契約C1と矛盾していた。
「新sessionの初期1の通知を観測」「reset後に自動出力なし・zero」を確認する試験へ修正。
