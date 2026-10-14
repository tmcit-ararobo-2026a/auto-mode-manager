# 自動モード契約・機能有効化条件（T00）

正本の保存先: [src/docs/auto_mode_contract.md](../../docs/auto_mode_contract.md)。ユーザー指定により共通docsを正本とする。本ファイルは配置変更前の内容を参考保存し、以後の仕様更新は正本だけに行う。

更新日: 2026-10-14（Asia/Tokyo）。契約版: `auto-mode-contract-v1`。
状態: ROS側の要求識別・取消・品質評価・機能有効化・校正手順を確定。実機数値と基板・機構側の適合確認は未取得。
基板参照: gn10-mainboard `feat/new-teleop-api` / `e66ca03305666d2c53ba97a3e5c855201e2f7e53`。
通信契約は [robot-portの契約文書](https://github.com/tmcit-ararobo-2026a/robot-port/blob/feat/10-stm32-new-udp/docs/new_mainboard_contract.md) と合わせて読む。

managerがJetson状態・Operationの唯一の生成元。基板の最終調停・CAN出力とは別責務。
下位パッケージからmanagerへの依存は作らない。試験fixtureと本番構成を分離する。

## 調査時の未確定項目・担当区分・保留条件（履歴）

以下は調査時の分類を保持したもの。現在のROS側の規範は後半の「v1確定契約」を優先する。
担当は相談先の区分であり、担当者の引受・実機承認は未確認。

| 項目 | 担当区分 | 未確定の間の扱い |
| --- | --- | --- |
| U3〜U5: Stanby取消、要求sequence、新規と再送、同一対象再要求、ボタン保持・解除、再起動・周回・順序逆転 | 基板状態管理・コントローラー | 実機要求処理・自動再開を有効化しない。内部世代でwireの識別欠落を補ったと扱わない |
| U6〜U9・U14: Fail調停、operation/teleop断、全機構停止、受理ACK欠落 | 基板通信・調停・機構 | Fail要求・ROS抑制・UDP送信・基板反映・物理停止を別々に判定。実機停止受入を保留 |
| U10・U13: アーム単位・原点・範囲・ロック・展開・把持・手順・危険領域 | 機構・運用 | ハンドを有効化しない。高さ0・arm_enabled=falseだけを機構無効化の証拠にしない |
| U11: ベルト速度変換・校正範囲・無効射出解 | 射出・校正 | 初速とベルト速度を分離。未校正の自動射出を有効化しない |
| U12: 角度符号・丸め・無効値 | 基板・照準 | 99radや無条件clipを流用しない |
| U13: Ready品質、Goal離脱、Fail復帰、地図・チーム・停止位置 | 自己位置・navigation・運用 | 品質・対象・復帰契約不足でReadyや移動を許可しない |
| U15: shutdown・logging・cloth_collect副作用 | 運用・基板 | commandを通知するだけで副作用を実装しない |

## 当初の内部契約（v1にも継承）

- ROS内の実行世代とproducer sessionは遅延計画・Action結果・速度の排除に使う。wireには追加しない。
- 取消は自動速度ゼロ・lease失効を先に行い、Action取消・経路・制御履歴の消去を確認する。取消応答前にReadyのハンドを起動しない。取消期限超過はFail要求。
- 20ms周期、Operation/lease/安全評価100ms、command/feedback150msは初期設計値で、実機負荷・許容停止時間の承認値ではない。
- 安全評価番号は監視処理が実際に完了したときだけ進める。評価・入力の期限を検査し、停止・停滞・古い判定の再送・監視再起動で許可を失効させる。
- 起動はStandby。復旧だけでは中断した経路を再開しない。
- manager雛形はOperationを配信しない。状態型・内部模擬試験を先に整備し、確定資料のない実機機能は有効化しない。

## 段階別受入

通信・固定移動・Tracking・射出解のloopback模擬結合はハンド未完成でも進める。
限定HILは不要機構の物理的な駆動無効化の確認と該当契約の確定が必要。
全体実機は全機構・校正・停止・復帰を含むT19cが必要。現在は未実施。

## v1確定契約: 適用範囲と承認の意味

ユーザーの仕様確定依頼に基づいてROS実装の動作を決める。基板に存在しない要求ID・ACK・停止機能、実測値を追加済みとは扱わない。今回は文書のみ変更し、実装・実機有効化を行わない。

v1は現STM32を維持する`command_policy=observed_transition_v1`を採用する。この制限付き要求契約は2026-10-14にユーザーが明示選択した。完全な同一対象再押下識別は提供しない。別方式を使う場合は契約版と基板適合証跡を更新し、自動推定で切り替えない。

| 区分 | 確定した内容 | 未完了の範囲 |
| --- | --- | --- |
| 要求識別 | 同一Moveの消費、解除を挟んだ再要求、ROS世代、起動・復旧時の再受付条件 | UDP欠落・逆順・基板再起動の完全識別、取消要求を観測できる基板条件 |
| 取消 | 全状態で取消優先、即時の許可失効、Action終了待ち、非自動再開 | コントローラーからの取消保持、物理停止の実証 |
| Ready品質 | 品質profile、全条件のAND、安定区間、品質喪失時の扱い | 真値比較と許容位置・Yaw誤差、実機profileの承認 |
| 機構停止 | ROS抑制と物理停止の分離、機構別の確認条件、未確認時の禁止 | 停止経路・原点・保持・到達・CAN測定鮮度・停止時間の実機証跡 |
| 校正 | 単位・変換境界、profileと版、補間範囲、承認・失効・試験手順 | 実測座標、機構寸法、角度対応、射出LUT・電圧補償 |

`approved=true`は値を承認した証跡への参照を伴うものとし、設定だけで証跡を代替しない。値を得ていない項目は未承認のまま、その能力を拒否する。この禁止動作も確定仕様に含む。模擬データの承認は実機承認とは別。

### C1. 要求識別

1. `RobotCommand.sequence`は不透明なwire値として記録する。受信回数・sequence変化・feedback連番を要求IDとみなさない。8 byteのwireは変更しない。
2. 起動はStandby、受付未許可。最初に届いたMoveは開始しない。新鮮なSleepまたはStanbyを**150ms以上連続観測**した後に次のMoveを受け付ける。50ms周期なら通常4回以上の観測を要し、150ms以上の受信gap、Move混入、異常・再起動でこの区間をリセットする。これはROSの解除確認規則であり、物理ボタンを離した証明ではない。
3. 受付後は`(navigation_command=Move, target_id)`を消費済みとし、周期再送を無視する。Goal・Fail・計画失敗・受付時品質不足でも、保持中のMoveを後から実行しない。品質不足時は拒否理由を通知し、解除からやり直す。
4. 同一対象の再要求は「解除区間 → 新しいMove」だけを許す。固定対象のボタンを離して押し直すだけでは基板のMoveが保持され、識別できないため再要求にならない。運用では左スティック取消または手動への切替を挟む。
5. 走行中に対象IDだけ変わったMoveも開始・予約しない。目標差替えは取消完了・解除区間・新Moveの順とする。これにより取消待ち中の要求をキューに残さない。
6. managerの起動ごとに非ゼロ`producer_session`を生成する。`generation`は1から開始し、受付・実行失効のたびに単調増加する。周期再送では変えない。計画・Action・PathVelocity・射出解にsession/世代を関連付け、対象不一致・失効世代・廃止sessionの結果を捨てる。uint64周回前は停止し、新sessionのStandbyからやり直す。
7. command/feedback断、bridge再起動、ROS時計逆行、観測できた基板再起動では実行を失効する。通信復旧だけで再受付しない。基板のboot IDはwireにないため、無通信を伴わない再起動は完全検知できない。8bit番号の周回処理も本方式では要求判定に使用しない。

**適合限界**: 150msの解除観測ではUDPの古いMove・古いStanbyを完全識別できない。現wireに生成時刻・boot ID・要求IDがないため、任意の遅延・逆順・取消パケット全欠落への保証はない。ROS内の遅延結果排除とwireの完全識別を区別する。実機適合には通信路の遅延・欠落試験と基板側の取消保持が必要。

完全識別を要求する次版では、基板のboot/session、要求単位のID、同一要求の再送規則、取消の持続、受理確認を外部契約として定義する。現8bit sequenceに勝手な意味を付けない。本タスクではSTM32変更を行わない。

### C2. 取消・競合・復帰

managerは同じ制御tickに取り込んだイベントを次の順で評価する。優先度の低いイベントで高いイベントの結果を上書きしない。

| 優先度 | 条件 | 処理 |
| --- | --- | --- |
| 1 | 非常停止・過電流の通知、通信／入力／品質／安全評価の失効、処理異常 | 自動許可を失効してFailをラッチ。復旧だけで解除しない |
| 2 | Sleep | 自動許可を失効し取消。正常な終了確認後にStandby。Sleepは実モードや全停止のACKではない |
| 3 | Stanby、managerの明示取消 | 全状態の実行を取消。Movingだけでなく計画中・Tracking・Goal・ハンドにも適用 |
| 4 | 新Move | Ready、取消なし、解除確認済み、対象機能の全条件成立時だけ受付 |
| 5 | 計画・Action到着・Track更新 | 現session/世代の結果だけ反映 |

commandを「最後の1件だけ」に畳み込んで取消を消さない。bridge以後のROS配送も順序・欠落を検証対象とし、購読処理で観測したSleep/Stanbyをtickまでラッチする。ただし基板から送信されなかった取消は検出できない。

取消処理は次の順に実行する。

1. 同じtickで出力採用を禁止し、managerの自動`vx/vy/vyaw`をゼロ化する。旧実行世代のControlLeaseを`enabled=false`で失効させ、旧速度を採用しない。停止処理の完了まで新世代の許可leaseを発行しない。
2. 未完了ComputePathの結果を無効化し、保有するFollowPath goalを取消する。遅いgoal受理通知が届いた場合も、そのgoalを取消する。既存の経路・積分・平滑化履歴を再利用しない。
3. ROS取消完了は、すべての未決goal受理が解決し、取消対象Actionが終端状態になり、追従側のゼロ出力・履歴消去が確認できた時点とする。goalがなかった場合も未決受理がないことを確認する。lease失効による終端も許可失効後の停止経路として扱う。
4. 取消開始から**200ms**でROS終了確認が得られなければFailにする。取消到着とSUCCESSが競合した場合も、その実行をGoal成功として採用しない。これはROS処理の試験上限で、物理停止時間の承認値ではない。
5. 終了確認前にReadyへ戻らず、Readyに伴うハンドを起動しない。正常終了後は品質等が成立すればReady、SleepならStandby。異常・機構状態不明ならFailを維持する。取消後にReadyとなってもハンドは未armのまま保持する。

Stanbyを1回観測した時点で取消をラッチし、その後の保持Moveを拒否する。解除区間を再観測するまで再開しない。取消中の新Move・到着・見失いの競合は取消またはFailが勝つ。MovingBucketの解除も同じ取消処理を使う。

Fail復帰はオペレーターの明示reset、原因解消、必要品質の再成立、機構の再確認、解除区間の再観測をすべて要求する。標準`std_srvs/Trigger`を使うmanagerの`/auto_mode/reset_fault`を実装時に設け、条件不足なら拒否理由を返す。resetは旧経路再開・自動ハンド開始を行わない。Sleep/Stanby単独でラッチ済みFailを解除しない。

**現基板との不適合**: Tracking取消は押下エッジ1回で、保持ボタンが次のteleopでMoveに戻すため、50ms周期commandに取消が載らない場合がある。コントローラーの「ボタンを離してから取消」を運用条件とするだけではパケット欠落まで保証できない。基板側で取消を解除確認まで保持する条件、または同等の別取消経路が検証されるまでは、物理Trackingの取消適合を未完了とする。

### C3. Ready品質と状態の意味

Readyは「選択した運転profileで次の要求を安全に受け付けられる」という**Jetson側の状態**。基板の実モード・ハンド原点・物理停止を観測できたという意味ではない。`contracts_confirmed`は選択profileに必要な契約の適合を表し、模擬profileのtrueを実機承認として表示しない。実装時にstatusへprofile ID・能力別可否を明示する。

品質評価は自己位置パッケージの独立CPU monitorが所有し、managerが再度鮮度・profile一致を検査する。`ready_quality_valid`は下記の瞬時条件と安定条件が成立したときだけtrueにする。現コードが常にfalseであることは維持すべき未実装状態で、仕様確定だけでtrueへ変更しない。

以下をすべてANDで要求する。

- 点群・IMU・計算結果が新鮮で、センサーTFが測定時刻で得られる。Poseと`map→base_link`も同じ測定に対応し、有限位置・正規化可能なQuaternionである。
- `last_match_success=true`、`lost=false`、`map_loaded=true`、`using_fallback_map=false`。
- `match_metrics_valid`と`inlier_metrics_valid`がtrueで、`best_cost <= max_best_cost`、`inlier_count >= min_inliers`、`inlier_cost <= max_inlier_cost`。
- 地図の内容ID、チーム、センサー外部校正、実装版・設定版が承認済み品質profileと一致する。
- 実行時は承認profileのPose鮮度・運動整合・品質閾値を満たす。その閾値で許容位置／Yaw誤差内であることは独立真値による受入で承認済みとする。実行時に真値を取得できるとは仮定しない。静止時の散らばりだけで絶対位置精度を承認しない。

成立が**1秒以上かつ異なる計算結果10件以上**連続してからReady品質を成立させる。同じresult_sequenceのmonitor再配信は件数を増やさない。どれか1条件の不成立・結果session変更・時計逆行で連続区間をリセットする。走行中は1件の不成立または期限切れで直ちに自動許可を失効する。既存のlost_count=10まで待たない。

実装上の安定区間1秒／10結果はROS仕様として固定する。現matching設定の`0.165 / 60 / 0.05`は計算閾値の参照値であり、実機の許容位置／Yaw誤差を表さない。模擬試験のprofileには使用できるが、実機の品質profileは真値比較の結果に基づいて別途承認する。

| profile必須値 | 単位・制約 | 根拠の取得方法 |
| --- | --- | --- |
| max_best_cost / min_inliers / max_inlier_cost | 現matcherの指標尺度、有限非負／正整数 | 正常・誤位置・遮蔽・類似形状・赤青地図の検証データで選定 |
| max_cloud_age / max_imu_age / max_result_age / max_pose_age / max_tf_age | 秒、有限正値 | 負荷下の最悪鮮度と制動距離を評価。現在のmonitor既定0.5秒を安全承認値として転用しない |
| max_position_error / max_yaw_error | m / rad、有限正値 | 測量した独立真値に対する最大誤差と余裕。navigation許容幅・障害物余裕・照準条件以内 |
| max_pose_jump / max_yaw_jump | m / rad、運動範囲を考慮 | 実速度・測定間隔と姿勢差による不整合検出。正常走行を誤判定しない |
| map_id / team / extrinsic_id / quality_id / evidence_id | 空でない識別子、版と内容hash | 実際に評価した地図・校正・ソフトウェア・検証記録に固定 |

共分散は現在固定値`0.005, 0.005, 0.002`であり、`covariance_valid=false`を維持する。v1では「共分散が小さい」をReady根拠にせず、真値比較で承認した品質profileを利用する。推定共分散を将来導入する場合は整合性検証後に別契約版とする。基板仕様の「信頼度・共分散が安定」との差はこのROS補足契約として明記し、既存実装が要求を満たしたと主張しない。

Boot／通常復旧のReady移行には解除確認も必要。最新commandがSleepの間は品質が成立してもStandbyを維持し、MoveがReady成立前に届いた場合は消費して拒否する。起動中に品質が未成立であるだけならStandbyと理由を公開し、実行済みの状態からの品質喪失と区別する。Moving・Tracking中の品質喪失はFail、Ready／Goal中の品質喪失も自動ハンドを失効してFail。Goal離脱は承認済み停止Poseからの位置・Yaw誤差が離脱閾値を超えた場合に実行世代とハンドを失効し、品質成立時だけReadyへ戻る。離脱閾値は到達閾値より大きくし、実機数値を未承認の既定値で埋めない。

### C4. 機構停止・機能有効化

停止記録は`automatic_inhibited`（ROS抑制）、`fail_sent`（UDP送信）、`board_applied`（基板反映）、`physical_stop_confirmed`（機構実測）を分ける。送信失敗はfalse、確認経路のない後2段階はunknownとする。feedbackをACKへ読み替えない。

Fail・Standby・Sleepでは手動入力が機構を駆動し得る。R1/L1は全状態共通で、ベルト速度0は基板で最小速度へclampされる。したがってROSのゼロ化・Fail・送信停止を全機構停止方法として指定しない。異常時に保持・把持の電源を切るか維持するかも、荷重や落下リスクを含む機構profileで決める。

| 機構 | 停止確認に必要な契約 | 現行feedbackで不足する情報 |
| --- | --- | --- |
| 車輪 | 制御権遮断、全輪の実測速度閾値・連続静止時間、許容停止時間と距離 | CAN計測時刻・通信健全性、基板受理状態。周期feedbackの受信だけでは古いCAN値を区別できない |
| ベルト射出 | トリガー遮断、進行中fireの中断、実測速度・原点、再装填の抑制 | fire中断ACK・状態・原点valid・CAN計測鮮度 |
| 装填 | 装填シーケンス中断、位置保持または電源遮断、到達・静止の確認 | 目標と動作状態、測定鮮度。角度単体ではシーケンス中断を示さない |
| 昇降 | 原点valid、位置制御の単位・限界、制動／保持、安全姿勢 | 原点validと独立の測定時刻。未原点時も高さ0が返る |
| 展開・把持 | 実位置／把持状態、停止と荷物保持、走行許可姿勢 | 展開・把持の到達／保持状態がwireにない |
| 空圧 | トリガー遮断、弁・圧力・シリンダの安全状態、残留エネルギー | 弁・圧力・シリンダ位置・停止ACKがwireにない |

物理停止判定は機構ごとに承認した計測・閾値・連続確認時間を要求する。全停止は全必須機構のAND。新しいUDP feedbackが届いていてもCAN測定が古い場合はunknown。停止時間の上限・速度epsilon・原点・保持位置は実測を取得するまで未設定とする。

Ready/Goalへの遷移を自動ハンド開始トリガーにしない。ハンドは「明示的な処理開始、現世代、機構状態valid、車体静止、対象の承認手順成立」のときだけarmする。取消・Goal離脱・次Move・品質喪失で失効する。Moving/Tracking移行前に収納・ロック完了を確認する。所定時間で確認できなければFail。移行中にハンドが無効化されたことを`bucket_arm_height=0`で表現しない。

現在の基板はReady/Goalで高さ[m]をDC目標へ直接渡し、展開／把持boolをモーター目標0/1へ渡す。高さ変換関数や原点・範囲が存在しても、当該経路でその変換・安全保持が実装された証拠にはならない。原点未完了の高さ0も有効位置ではない。自動ハンドは基板・機構の適合が得られるまで無効。

| 運転profile | 必須条件と許可 |
| --- | --- |
| simulation | loopback、専用domain、模擬基板・品質・機構。実機宛先を禁止 |
| navigation_hil | 車輪停止の検証、実品質・地図・車輪校正、不要な射出・ハンド・空圧の**物理的な遮断の証跡**が必須。Ready/Moving等のwireが不要機構を駆動しない構成でのみ固定移動を許可 |
| full_robot | 全基板条件・機構原点／状態／停止・全校正・取消経路が必須。Moving/Trackingの自動ベルト威力を未校正のまま代替値で埋めない |

`arm_enabled=false`や試験用の`mechanism_valid=true`だけではHILを許可しない。現時点で実機の物理遮断・停止・原点の証跡はないため、実機のprofileを有効化しない。ハンド未完成でもsimulationの固定移動・Tracking・射出解計算は進められる。

### C5. 校正・座標・単位

ROSは`map`固定世界、`base_link`の+x前方・+y左方・+z上方、Yawは+zまわり反時計回り、角度rad・速度m/s・rad/sで統一する。TFに用いる外部校正は版付きで承認する。車輪指令はbase_link座標でmanagerから送信し、基板の軸変換をROSで二重に行わない。正方向の適合はHILで軸ごとに確認する。

照準bearingは`wrap(atan2(target_y-robot_y,target_x-robot_x)-robot_yaw)`。wire度変換は承認済みの`sign∈{-1,+1}`と`offset_deg`を用い、`sign*bearing*180/pi+offset_deg`を最近接整数へ丸める。ちょうど0.5は0から遠い整数へ丸める。丸め前の値が[-20,20]外、非有限、未観測なら無効とし、clip・99・NaNを使用しない。

ROS内部は角度ごとのvalidと時刻を保持する。wireにvalidityはないため、無効値時の表示抑止／角度消費規則を基板・表示側で承認するまでは、0を「照準中央・有効」とするOperationを本番へ送らない。LEDのradフィールドへ度を渡す問題をROSの単位変換で隠さない。desk1/2とwireのdesk1枠の選択規則も対象IDと合わせて外部適合確認する。

校正profileには識別子、schema、取得日、対象ハードウェア／ソフトウェア版、地図・チーム、単位、測定範囲、不確かさ・許容誤差、証跡、承認状態を持たせる。既存field/shooterのJSONを再利用し、必要メタデータは所有パッケージの次タスクで追加する。profile承認と実装済みの判定を区別する。

| 校正対象 | 必須測定・契約 | 合否・失効条件 |
| --- | --- | --- |
| 固定対象・地図・TF | 全IDの照準点と停止Poseを別々に測量。赤青・StartZone・障害物・対象寸法・射出口のoffsetを同一mapへ登録 | 地図／チーム／profile不一致、停止位置干渉、実測誤差超過なら拒否 |
| 車輪 | 半径、取付角、中心距離、減速比、encoder単位、全輪符号、指令と物理速度、負荷下の制動時間／距離 | forward/left/yawの正方向・速度・停止試験に合格。未承認の既定速度1.5m/s等を実機性能値にしない |
| 自己位置 | 真値付き静止・走行・遮蔽・誤初期値・類似形状・再探索・入力断 | C3の全条件で誤位置をReadyにせず、承認誤差以内。地図・外部校正変更時は失効 |
| 昇降・展開・把持 | 最高点原点、地面基準高さ[m]、プーリー／減速比、到達範囲、保持／収納姿勢、電流・速度、取消と停止 | 原点validと到達を独立確認。boolを物理位置／電流へ直接流用しない |
| 射出 | 実射出口位置・角度、布質量・面積、抗力モデル、測定初速と機器指令の対応、電圧・荷重・機器版 | 独立検証点で着弾誤差・速度誤差以内、未計測範囲は拒否 |

射出計算の`initial_speed_m_s`（布の初速）と`belt_speed_m_s`（機器への校正済み速度）を分離する。既存の`[initial_speed_m_s,belt_speed_m_s]`の少なくとも2点・単調LUTを再利用し、測定範囲内だけ線形補間する。数学的に2点必要であることは実機承認に十分という意味ではない。電圧・負荷・距離ごとの独立検証点も必要。feedbackのrelease速度を布初速と等しいとは仮定せず、布の速度を独立計測する。

現shooterは1〜20m/sのsolver範囲と単一符号・増加するベルト速度絶対値を要求する。一方、基板は速度clampとfire時電圧補償を持つ。したがって正符号・単位・min/max・電圧補償の責務を測定で確認し、**実際に送信した指令から布初速までの全経路**でLUTを承認する。ROSと基板の二重補償を禁止し、飽和がある場合は拒否する。負符号LUTをROSが構文受理しても基板適合済みとは扱わない。

ShotSolutionは同じsession/世代/target、必要鮮度、校正ID、`valid`、`in_calibration_range`がすべて成立したものだけ採用する。無効・到達不能・期限切れなら以前の有効速度を保持しない。未校正のゼロ指令が射出禁止にならないため、実機ではトリガー遮断契約またはFailを含む承認済み異常手順を必要とする。Fail単独を射出禁止と呼ばない。

移動対象の射出は、飛行時間・target velocity・車体移動・処理遅延を含む補償契約がないためv1実機では無効。Yaw追従と射出解の模擬計算を先行し、静止射出の校正を移動対象へ流用しない。

### C6. 時間・監視

| 項目 | v1のソフトウェア検証値 | 意味 |
| --- | --- | --- |
| manager／安全評価周期 | 20ms | 実機の最悪スケジューリング時間は別途測定 |
| Operation／ControlLease／安全評価期限 | 100ms | 元の生成／評価と入力期限に基づく。再配信で延長しない |
| command／feedback受信期限 | 150ms | 受信健全性。CAN測定鮮度・受理ACKにはならない |
| 解除観測区間 | 150ms以上 | C1の再受付条件。遅延パケットの識別保証にはならない |
| 取消ROS終了確認期限 | 200ms | C2の終端確認。物理停止の期限とは別 |
| Ready品質成立 | 1秒以上・異なる結果10件以上 | 品質成立の安定区間。実測精度を代替しない |

数値はROS制御・模擬試験契約。許容停止時間・物理応答・実機センサー鮮度の承認値ではない。単調時計で期限を検査し、ROS時刻は測定対応に使う。停止通知時は次tick待ちにせず同じ処理で出力採用を禁止し、周期出力は遅くとも次tickまでに反映する。

安全monitorは別プロセス。実評価完了でのみ評価番号を進め、元入力の時刻を保持する。managerは番号停滞・未受信・古い評価再配信・session変更を期限切れとして扱う。manager自身の停止はfollowerのleaseとbridgeのOperation期限で抑制する。独立watchdogがあっても全機構の物理停止保証はC4で判定する。

### C7. 実装受入例と外部残件

| ID | 入力・故障 | 必須結果 | 実施段階 |
| --- | --- | --- | --- |
| A01 | 起動直後Move、同じMoveを100回、Goal後保持Move | 起動Moveを拒否、解除後1実行、Goal後再実行なし | T10・T18b |
| A02 | 同じ対象を解除150ms未満／以上で再要求、取消中の別Move | 未満拒否、以上は品質成立時だけ新世代、取消中は予約なし | T10・T18b |
| A03 | Stanbyと到着競合、goal受理遅延、取消応答断 | 取消勝ち、遅いgoalも取消、200ms超でFail、ハンド非起動 | T10・T08・T18b |
| A04 | Tracking保持中取消、解除・押し直し、取消パケット欠落 | 観測取消後の保持再開なし。観測不能のケースは基板不適合として記録 | T12・T18c、基板試験 |
| A05 | 1秒／10結果の境界、重複raw結果、固定共分散、誤位置、fallback地図 | 承認profileと全ANDだけReady。重複で安定区間を進めない | T05追加・T18b |
| A06 | localization／monitor／manager／bridge停止、時計逆行 | 許可失効、旧世代非採用、復旧だけで再開なし | T14・T18b/c |
| A07 | 未原点で高さ0、CAN断で同じ値をfeedback再送、未確認機構 | 原点・物理停止を成功にしない。対象能力を拒否 | T15・T18d・HIL |
| A08 | 無効射出解、LUT境界／範囲外、正負・電圧補償・clamp | 旧速度不採用、範囲外拒否、実機トリガー禁止を別確認 | T13・T18c・HIL |
| A09 | profile変更・赤青不一致、angles非有限／±20外／半端値 | 実行失効、再承認・解除要求、無条件clipなし | T09・T13・T18e |
| A10 | 全機構停止と手動入力残存、各機構電源／通信断 | ROS抑制・Fail送信・機構停止を別記録、全機構の証跡が揃うまで全停止unknown | T19a/c |

| 残件 | 必要な提供物 | 完了までの制限 |
| --- | --- | --- |
| E1 要求・取消の基板適合 | 取消をcommandへ持続反映する仕様・試験、欠落／逆順／起動の適合試験。完全識別が必要なら次版プロトコル | 物理Tracking取消・完全要求識別の受入不可 |
| E2 Readyの実測 | 真値付き評価、許容位置・Yaw誤差、鮮度・品質profileと証跡 | 実機Ready不可。模擬Readyのみ可能 |
| E3 停止・原点・到達 | 機構別停止経路、単位、原点valid、CAN計測鮮度、最大停止時間／距離、保持・収納の証跡 | 自動ハンド・全体実機不可。限定HILも物理遮断・車輪適合が必要 |
| E4 固定対象・角度・射出 | 測量済みprofile、角度対応・表示valid規則、射出LUTと電圧補償の検証 | 実機の該当照準・射出不可 |
| E5 危険領域・副作用 | 危険領域と停止権限、shutdown/logging/cloth_collectの承認された意味 | 未定義の副作用は実装しない。未定義の安全許可を作らない |

T00はROS契約v1の仕様策定まで完了。外部適合・実測・実機受入は未完了として残す。T10・T12・T14・T16a/bのsimulation実装はこの契約で開始できる。T15の実機手順やT19を完成とするにはE1〜E5の該当証跡が必要。

### 実装対象への反映（今回は未変更）

| タスク・所有者 | 変更対象 | 契約を実現する変更 |
| --- | --- | --- |
| T10・manager | `src/auto_mode_manager_node.cpp`、要求・状態管理クラス、`msg/AutoModeStatus.msg`、manifest/CMake | C1の解除・消費、C2のイベント優先・取消完了・reset、profile／能力別公開、std_srvs依存 |
| T05追加・localization | `src/localization_status_monitor_node.cpp`、`msg/LocalizationStatus.msg`、品質profileとtest | C3の品質AND・安定区間、Pose/TF対応・地図／校正版の識別、未承認false。固定共分散は維持 |
| T08照合・navigation | `src/path_follower_node.cpp`、Action／lease試験 | 失効leaseとAction終端、ゼロ出力・履歴消去、遅延goalと200ms期限の結合受入。既存処理を再利用 |
| T12・T14・manager | Yaw制御・独立安全monitor、`msg/SafetyState.msg`、profileとtest | C2のTracking取消、C6の評価進行・入力鮮度、失効・復帰時の非再開 |
| T09・T13・field/shooter | 既存ノード、`config/targets.json`、`config/calibration.json`、各msgとtest | C5の証跡／版、角度valid、LUT・電圧適合・無効解の処理。実測値取得前にapprovedを変更しない |
| T15・T16・manager/bridge | ハンド状態・Operation組立・機構profile、bridge契約と結合試験 | C4の機構別確認・profileゲート、未決のwire反映はunknown。外部条件をROSのダミーboolで補わない |

実機用の新メタデータがwireにない場合は「生成できないので無効」が既定。実装者が新しいパッケージを追加してデータの欠落が解決したと扱わない。

## v1の照合根拠

2026-10-14に実コードを再確認。STM32の指定ブランチ先頭は引き続き`e66ca033`。ROS各提供側の照合HEADは共通current-stateに記録する。

- [要求生成](https://github.com/tmcit-ararobo-2026a/gn10-mainboard/blob/e66ca03305666d2c53ba97a3e5c855201e2f7e53/app/src/robot_state_machine.cpp#L29-L105)、[初期sequence・localization・Sleep](https://github.com/tmcit-ararobo-2026a/gn10-mainboard/blob/e66ca03305666d2c53ba97a3e5c855201e2f7e53/app/include/app/robot_state_machine.hpp#L80-L103)。
- [基板仕様のReady・取消・Fail](https://github.com/tmcit-ararobo-2026a/gn10-mainboard/blob/e66ca03305666d2c53ba97a3e5c855201e2f7e53/docs/auto_mode_specification.md#L109-L125)、[全状態トリガーと調停](https://github.com/tmcit-ararobo-2026a/gn10-mainboard/blob/e66ca03305666d2c53ba97a3e5c855201e2f7e53/app/src/control_arbitrator.cpp#L19-L85)。
- [機構出力・feedback鮮度・未原点高さ0・停止経路](https://github.com/tmcit-ararobo-2026a/gn10-mainboard/blob/e66ca03305666d2c53ba97a3e5c855201e2f7e53/app/src/app.cpp#L332-L422)、[teleop先行のループ](https://github.com/tmcit-ararobo-2026a/gn10-mainboard/blob/e66ca03305666d2c53ba97a3e5c855201e2f7e53/app/src/app.cpp#L524-L550)。
- [アーム高さ・原点・角度変換](https://github.com/tmcit-ararobo-2026a/gn10-mainboard/blob/e66ca03305666d2c53ba97a3e5c855201e2f7e53/app/include/app/bucket_arm_controller.hpp#L85-L122)、[速度clamp](https://github.com/tmcit-ararobo-2026a/gn10-mainboard/blob/e66ca03305666d2c53ba97a3e5c855201e2f7e53/app/include/app/belt_launcher_controller.hpp#L31-L34)、[fire時電圧補償・再装填](https://github.com/tmcit-ararobo-2026a/gn10-mainboard/blob/e66ca03305666d2c53ba97a3e5c855201e2f7e53/app/src/belt_launcher_controller.cpp#L17-L41)。
- [自己位置の現在の監視](https://github.com/tmcit-ararobo-2026a/gn10-pointcloud-localization/blob/7798df1764b52152011b851757e3e268bde244a7/src/localization_status_monitor_node.cpp#L120-L147)、[固定共分散とraw品質](https://github.com/tmcit-ararobo-2026a/gn10-pointcloud-localization/blob/7798df1764b52152011b851757e3e268bde244a7/src/localization_node.cpp#L1030-L1078)。
- [射出profile検査とLUT](https://github.com/tmcit-ararobo-2026a/shooter_calculator/blob/50b9f62d6cd6f6f63af573d8099214e3f5529512/src/shooter_calculator_node.cpp#L60-L125)、[FollowPath](https://github.com/tmcit-ararobo-2026a/gn10-navigation/blob/5c8db721da0a0a84f3395647f53382794fb13888/action/FollowPath.action)。
