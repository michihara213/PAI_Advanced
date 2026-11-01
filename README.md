# 模倣学習によるロボットの手振り動作

ROS 2 HumbleとGazebo上のCRANE+を対象に、手本の操作記録から手振りを学習・自動実行するプロジェクトです。動作の進行度（0〜1）を入力とし、アーム4関節とグリッパ1関節の目標角度をRBF回帰で推定します。

CPUで学習できます。講義のACTとは異なる、進行度に条件付けた開ループの軌道模倣です。

## 実行結果

教示10本のうち`ep01`〜`ep08`を学習、`ep09`・`ep10`を評価に使用しました。

| 評価対象 | 一定角度モデルのRMSE | RBFモデルのRMSE |
|---|---:|---:|
| 関節1 | 13.04度 | 2.11度 |
| 関節3 | 10.26度 | 0.71度 |

一定角度モデルは、学習用データ全体の平均目標角度を常に出力します。各RMSEは評価用2本の全201点ずつをまとめて計算した値です。関節2・4・グリッパの誤差は0または数値丸め程度でした。表はオフラインの目標角度誤差であり、実機精度や動作成功率ではありません。

実行ログでは、開始姿勢への移動を含む約41.4シミュレーション秒の動作で、両コントローラーが成功を返しました。提出動画でも手振りを確認しています。複数試行による成功率の測定は行っていません。

`evidence/reported_metrics.json`と`evidence/episodes_reported.csv`は、実際に得られた端末出力の転記です。元の測定ファイルとは分けています。受領した`results/metrics.json`と転記した評価値の一致も確認しました。

## ファイル

- `wave_ai.py`：データ前処理、学習、Gazeboでの推論・実行
- `test_wave_ai.py`：人工データによる回帰テスト8件
- `requirements.txt`：Pythonの数値計算・描画依存
- `evidence/`：提示された実行ログを基にした評価記録
- `results/`：学習済みモデルや実際の出力ファイルを置く場所
- `ACKNOWLEDGEMENTS.md`：使用環境と開発支援の説明

**実行に使用した学習済みモデル、評価JSON、比較図を`results/`に同梱しています。** 既存のROS・Gazebo環境では再学習せず`python3 wave_ai.py run --sim`で実行できます。元のrosbagは含まないため、再学習には別途教示データが必要です。

## 前提環境

動作確認環境はWindows上のDocker、ROS 2 Humble、Gazebo（Ignition Gazebo 6）、CRANE+です。ベースのDockerイメージは`airobotbook/ros2-desktop-ai-robot-book-humble:latest`を使用しました。`latest`は固定された環境ではないため、完全な環境再現を保証する指定ではありません。

以下のパッケージが必要です。

- Python 3.10以降、NumPy、Matplotlib（図の保存時）
- ROS 2：`rclpy`、`rosidl_runtime_py`、`sensor_msgs`、`trajectory_msgs`、`rosgraph_msgs`、`control_msgs`
- `crane_plus_gazebo`およびCRANE+のアーム・グリッパコントローラー

既存の講義用コンテナ内で、数値ライブラリが不足する場合は次を実行します。

```bash
sudo apt-get update
sudo apt-get install -y python3-numpy python3-matplotlib
```

ROSのPythonパッケージはROS環境から読み込みます。`requirements.txt`だけではROSやGazeboは導入されません。

## 記録データ

データフォルダの直下に`ep01`〜`ep10`を配置し、各フォルダ内にSQLite3形式の`.db3`ファイルを置きます。読み込み対象は以下です。

| トピック | 用途 |
|---|---|
| `/clock` | シミュレーション時刻 |
| `/joint_states` | 関節状態の確認 |
| `/crane_plus_arm_controller/joint_trajectory` | アームの教師指令 |
| `/crane_plus_gripper_controller/joint_trajectory` | グリッパの教師指令 |

教示時はキーボード操作プログラム`physicalai_teleop`を使用しました。1メッセージに1点の指令が入る記録を対象にしています。複数点の軌道メッセージはこの前処理では扱いません。

## 前処理と学習

以降は、このREADMEと同じフォルダを作業ディレクトリとするUbuntu端末のコマンドです。

```bash
source /opt/ros/humble/setup.bash
python3 wave_ai.py prepare --data ~/dataset/rosbags/wave_full
python3 wave_ai.py train
```

`--data`は実際の保存先に変更してください。前処理は元のbagを読み取り専用で開きます。

1. 関節名で指令を並べ替え、メッセージ数・有限値・必要トピックを確認します。
2. `/clock`で受信時刻をシミュレーション時刻に対応させます。過去の最大時刻から10ミリ秒以内の小さな逆戻りのみ補正し、それを超える場合は中断します。
3. 指令をゼロ次ホールドで20 Hzの格子に並べ、0.01 radを超える指令変化を基準に前後の停止区間を除きます。初回変化の約0.5秒前から最終変化の約1秒後を残します。途中の停止区間は残ります。
4. 動作の進行度を0〜1に正規化して各記録を201点に揃えます。
5. 8本を等しい重みで学習し、残り2本に対する誤差を計算します。

前後の停止区間を残す場合は`prepare`に`--keep-idle`を付けます。`results/episodes.png`で各回の動作順序や大きさを確認してください。

モデルは32個のガウス基底と定数項を持ち、L2正則化付き最小二乗法で重みを求めます。中心は0〜1に等間隔、幅は`1.5/31`、正則化係数は`0.001`です。各回の時間正規化した指令の平均を滑らかに近似する方法に相当します。

生成物は`dataset.npz`、`episode_report.csv`、`model.npz`、`metrics.json`、`episodes.png`、`learned.png`です。Matplotlibがない場合は図の生成だけを省略します。

## Gazeboでの自動実行

別端末で従来のシミュレーターを起動します。ワークスペースの場所は環境に合わせて変更してください。

```bash
source /opt/ros/humble/setup.bash
source ~/airobot_ws/install/setup.bash
ros2 launch crane_plus_gazebo crane_plus_with_table.launch.py
```

キーボード操作プログラムをESCで終了し、`results/model.npz`があることを確認して実行します。

```bash
source /opt/ros/humble/setup.bash
python3 wave_ai.py run --sim
```

現在の姿勢から開始姿勢へ移動した後、学習した動作を1回実行します。推定角度を学習データの最小・最大値の範囲に収め、隣接する指令点間の速度が0.3 rad/s以下になるよう動作時間を調整します。アームとグリッパへ`FollowJointTrajectory`アクションを送信します。

他の指令トピック発行者がいる場合や、時刻・状態の更新が止まった場合は中断します。Ctrl+Cは動作キャンセルを要求します。実際の速度・即時停止・衝突回避を保証する機能ではなく、実機向けの検証は行っていません。

## テスト

```bash
python3 -m unittest -v test_wave_ai.py
```

関節名順序、非有限値・欠損の拒否、因果的な指令保持、シミュレーション時刻変換、時計の小さな逆戻り補正と大きな逆戻り検出、待ち時間除去、往復動作の学習、指令点間の速度制限を人工データで確認します。ROSとの結合テストや実機安全性を検証するテストではありません。

## 制限と改善案

- 10本の同種動作による小規模な検証です。未知の環境や別の作業への汎化は未評価です。
- 入力は進行度のみで、カメラや人の位置を認識しません。
- 動作全体の時間正規化だけなので、動作途中の速度差は残ります。動作区間ごとの整列が改善案です。
- 指令に対するモデルの誤差を評価しており、ロボットの実際の追従誤差は別途評価が必要です。
- より多くの評価データと複数回の実行で、再現性を確認する余地があります。

## GitHubへの配置

このフォルダの中身をリポジトリのルートに配置します。提出用リポジトリの公開設定は課題指定の`public`です。実行済みの`results/model.npz`、`results/metrics.json`、`results/learned.png`も同梱した状態で掲載してください。

大きなrosbagやDockerイメージは同梱していません。動画は別途YouTubeにアップロードし、そのURLを提出します。
