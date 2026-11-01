# 使用環境と開発支援

- CRANE+、ROS 2、Gazeboの環境とキーボード操作は、Physical AI応用1講座の演習環境を基にしています。
- 教示時の操作には`physicalai_teleop`を使用しました。参照先：https://github.com/matsuolab/physicalai_teleop
- ベースイメージ：`airobotbook/ros2-desktop-ai-robot-book-humble:latest`
- 本リポジトリで追加した部分は、教示データの検査、進行度による整列、RBF回帰、評価、学習結果のGazebo実行をまとめた処理です。
- コード作成・不具合修正・資料作成にはChatGPTの支援を利用しました。教示データの収集、実環境での学習・実行、動作確認、動画撮影は手元のシミュレーション環境で行いました。
- 上流プロジェクトのコードやDockerイメージはこのパッケージに再配布していません。使用時は各配布元のライセンスに従ってください。
