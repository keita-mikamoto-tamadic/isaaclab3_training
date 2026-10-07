# Isaac Lab チュートリアル 学習カリキュラム

[scripts/tutorials/](../../IsaacLab/scripts/tutorials/) のスクリプトを順番に**自分で書き直しながら**学ぶための教材です。

## 進め方

1. 各章の md を読み、Lesson ごとの「学ぶこと」を確認する
2. 元スクリプトはなるべく見ずに、「自分で書くステップ」に沿って `my_tutorials/<章>/` に書く
3. 実行して「確認ポイント」をチェックする
4. 詰まったら元スクリプトと「参照コード」を読む
5. 余力があれば「発展課題」に挑戦する

コマンドはすべて `isaaclab_study/` のルートで実行します（このプロジェクトの `.venv` が使われます）。

```bash
cd ~/isaaclab_study
mkdir -p my_tutorials/00_sim
uv run python my_tutorials/00_sim/create_empty.py
```

元のチュートリアルは `~/IsaacLab/scripts/tutorials/` にあります。教材中の「参照コード」リンクは `../../IsaacLab/...` を指します。
03_envs 以降で自作タスクを登録するときは、`src/isaaclab_study/tasks/` 以下に置くと自動で検出されます。

## 章一覧

| 章 | ファイル | 内容 |
|----|----------|------|
| 00 | [00_sim.md](00_sim.md) | シミュレーション起動、プリムの配置、CLI 引数 |
| 01 | [01_assets.md](01_assets.md) | 剛体・関節・変形体・グリッパー・ロボット追加 |
| 02 | [02_scene.md](02_scene.md) | `InteractiveScene` でシーンをまとめて管理 |
| 03 | [03_envs.md](03_envs.md) | Manager-based 環境、RL 環境、ポリシー推論 |
| 04 | [04_sensors.md](04_sensors.md) | カメラ・レイキャスタ・接触センサなど |
| 05 | [05_controllers.md](05_controllers.md) | 微分 IK、オペレーショナルスペース制御 |
| 06 | [06_deploy.md](06_deploy.md) | 学習済みポリシーのデプロイ |
| 07 | [07_visualizers.md](07_visualizers.md) | タイルカメラ可視化、動画録画 |

## 進捗

- [ ] 00_sim
- [ ] 01_assets
- [ ] 02_scene
- [ ] 03_envs
- [ ] 04_sensors
- [ ] 05_controllers
- [ ] 06_deploy
- [ ] 07_visualizers
