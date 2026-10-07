# 00_sim — シミュレーションの起動とプリムの配置

対応スクリプト: [scripts/tutorials/00_sim/](../../IsaacLab/scripts/tutorials/00_sim/)

## この章のゴール

- Isaac Lab スクリプトの「お決まりの骨格」（引数パース → ランタイム起動 → `SimulationContext` → ループ）を何も見ずに書ける
- `XxxCfg` を作って `cfg.func(prim_path, cfg, ...)` で USD ステージにプリムを置く、という spawner の使い方を理解する
- CLI 引数でシミュレーションの挙動を変える方法を知る

## 進める順番

| # | スクリプト | テーマ | 目安 |
|---|-----------|--------|------|
| 1 | `create_empty.py` | 空のステージを作って物理を回す | 30分 |
| 2 | `spawn_prims.py` | 地面・ライト・図形・剛体・変形体・USD を配置 | 1〜1.5時間 |
| 3 | `launch_app.py` | 独自 CLI 引数を足してシーンに反映 | 30分 |
| 4 | `log_time.py` | ループ中にシミュレーション時刻をファイルへ記録 | 20分 |

自作ファイルは `user_docs/` ではなく、例えば `my_tutorials/00_sim/` のような別フォルダに置くと、元スクリプトと見比べやすくなります。

---

## Lesson 1: `create_empty.py` — 空のステージ

### 学ぶこと

- **スクリプトの骨格**: Isaac Lab ではシミュレータ関連の import を、引数パースと起動の**後**に書く慣習がある（起動前に import すると失敗するモジュールがあるため）。
- `add_launcher_args(parser)`: `--device`, `--viz`, `--physics` などの共通引数を argparse に追加する。
- `launch_simulation(cfg, args_cli)`: cfg を見て、必要なランタイム（Isaac Sim Kit など）を起動するコンテキストマネージャ。`with` を抜けると閉じる。
- `SimulationCfg`（`dt`, `device`）と `SimulationContext`（シミュレーション本体）。
- `sim.reset()` → `while sim.is_running(): sim.step()` という基本ループ。

### 自分で書くステップ

1. ファイル冒頭に `argparse` で parser を作り、`add_launcher_args` を呼んで `args_cli = parser.parse_args()`。

```python
import argparse
from isaaclab.app import add_launcher_args, launch_simulation

# create argparser
# description="~~" は--help引数時に表示される文
parser = argparse.ArgumentParser(description="Tutorial on creating an empty stage.")
# parserを渡してisaaclab引数をもらう、--deviceや--vizなど共通のもの
add_launcher_args(parser)
# パース
args_cli = parser.parse_args()

```

2. `from isaaclab.sim import SimulationCfg, SimulationContext` を**パース後に** import。
```python
# パース後にインポートすることで、引数ミスによる時間ロスを最小にする。
from isaaclab.sim import SimulationCfg, SimulationContext
```
3. `main()` で `SimulationCfg(dt=0.01, device=args_cli.device)` を作る。
4. `with launch_simulation(sim_cfg, args_cli):` の中で `SimulationContext(sim_cfg)` を生成。
5. `sim.set_camera_view(eye, target)` でカメラ位置を決める。
6. `sim.reset()` してから `is_running()` ループで `sim.step()`。
``` python
def main():
  # Configure the simulation
  sim_cfg = SimulationCfg(dt=0.01, device=args_cli.device)
  # sim_cfgとCLI引数から必要なランタイムを判断し起動。withを抜けると終了
  with launch_simulation(sim_cfg, args_cli):
    # 起動したランタイム上で、指定コンフィグのシミュレーション本体を作成
    sim = SimulationContext(sim_cfg)

    # カメラ設定
    sim.set_camera_viewe([2.5,2.5,2.5], [0.0, 0.0, 0.0])
    
    # simulatorスタート
    sim.reset()
    print("[INFO]: Setup complete...")
    
    # Simulate physics
    while sim.is_runnning():
      sim.step()
```

### 実行

```bash
uv run python my_tutorials/00_sim/create_empty.py              # ビジュアライザなし（ヘッドレス）
uv run python my_tutorials/00_sim/create_empty.py --viz kit    # Isaac Sim のビューポートを開く
```

### --viz kitやphysics指定なしの実行ができない場合
isaacsim環境一式をextra optionで入れる
```bash
uv sync --extra isaacsim
```


> Isaac Lab 3.0 では `--viz` を付けないとヘッドレスで動きます（`--headless` フラグはありません）。
> 見た目を確認したいときは `--viz kit` を付けてください。

### 確認ポイント

- [ ] `[INFO]: Setup complete...` が表示される
- [ ] `--viz kit` 付きならビューポートが開き、何もないステージが見える
- [ ] `--help` で `add_launcher_args` が追加した引数一覧を見られる

### 発展課題

- `dt` を 0.001 / 0.1 に変えると何が変わるか（この時点では見た目の差はほぼ無いことも確認）
- ループ内でステップ数を数え、500 ステップで `break` して終了させる
- `sim.get_physics_dt()` を print してみる

### 参照コード

- [sim_launcher.py: add_launcher_args](../../IsaacLab/source/isaaclab/isaaclab/app/sim_launcher.py#L72)
- [sim_launcher.py: launch_simulation](../../IsaacLab/source/isaaclab/isaaclab/app/sim_launcher.py#L558)
- [simulation_cfg.py: SimulationCfg](../../IsaacLab/source/isaaclab/isaaclab/sim/simulation_cfg.py#L36)
- [simulation_context.py: SimulationContext](../../IsaacLab/source/isaaclab/isaaclab/sim/simulation_context.py#L58)

---

## Lesson 2: `spawn_prims.py` — プリムを置く

### 学ぶこと

- **USD のプリムパス**: `/World/...` のような階層。`sim_utils.create_prim("/World/Objects", "Xform")` で親ノードを作り、その下に子を置ける。
- **spawner パターン**: `cfg = sim_utils.XxxCfg(...)` → `cfg.func("/World/Path", cfg, translation=..., orientation=...)`。
- 見た目だけのプリム vs 物理を持つプリム:
  - 見た目のみ: `ConeCfg(radius, height, visual_material=...)`
  - 剛体: `rigid_props`, `mass_props`, `collision_props` を追加
  - 変形体: `MeshCuboidCfg` + `PhysxDeformableBodyPropertiesCfg` / `PhysxDeformableBodyMaterialCfg`（PhysX 専用）
- `UsdFileCfg(usd_path=...)` で既存 USD（テーブル等）を読み込む。`ISAAC_NUCLEUS_DIR` がアセット置き場。
- **姿勢のクォータニオン**: Isaac Lab 3.0 での順序に注意（`orientation=(0.5, 0.0, 0.5, 0.0)` が何軸回転か、ソースのドキュメントで確認する）。

### 自分で書くステップ

1. Lesson 1 のファイルをコピーして出発点にする。
2. `design_scene()` 関数を作り、`sim.reset()` の**前**に呼ぶ。
3. `GroundPlaneCfg` → `DistantLightCfg` の順に追加し、実行して確認。
4. `/World/Objects` Xform を作り、赤いコーン（見た目のみ）を 2 つ置く → 実行。
5. 緑のコーンに剛体・質量・衝突を付けて空中に置く → 落ちることを確認。
6. 青い変形キューブを追加 → 落ちて潰れることを確認。
7. テーブルの USD を読み込む。

一つ追加するたびに実行して、何が増えたかを目で確認するのがおすすめです。

### 確認ポイント

- [ ] 赤いコーンは空中に浮いたまま、緑のコーンは落下する（違いを説明できる）
- [ ] `--viz kit` で開いた GUI の Stage ツリーで `/World/Objects/...` の階層が見える
- [ ] 青いキューブが変形する

### 発展課題

- 赤いコーンに `rigid_props` だけ付けて `collision_props` を外すとどうなるか
- `SphereCfg`, `CylinderCfg`, `CapsuleCfg` も置いてみる
- `for` ループで 3x3 のグリッドにキューブを並べる（プリムパスを重複させない）

### 参照コード

- [shapes_cfg.py](../../IsaacLab/source/isaaclab/isaaclab/sim/spawners/shapes/shapes_cfg.py)
- [from_files_cfg.py](../../IsaacLab/source/isaaclab/isaaclab/sim/spawners/from_files/from_files_cfg.py)
- [sim/utils/prims.py](../../IsaacLab/source/isaaclab/isaaclab/sim/utils/prims.py)
- [sim/spawners/](../../IsaacLab/source/isaaclab/isaaclab/sim/spawners/) 全体を眺めて、どんな Cfg があるか把握する

---

## Lesson 3: `launch_app.py` — 独自の CLI 引数

### 学ぶこと

- `add_launcher_args` の**前に**自分の引数（`--size`, `--width`, `--height`）を `parser.add_argument` で足せる。
- 引数の値（`args_cli.size`）をシーン設計に使う。
- `--width` / `--height` のように、ランタイム側が読む引数もある（`launch_simulation` に `args_cli` を渡しているので反映される）。

### 自分で書くステップ

1. Lesson 2 のコードから地面・ライトだけ残す。
2. `--size`（float, default 1.0）を追加。
3. `CuboidCfg(size=[args_cli.size] * 3, ...)` を作り、`translation=(0, 0, size / 2)` で地面にぴったり置く。
4. `--width`, `--height` を追加して、ビューポートサイズが変わるか確認。

### 実行

```bash
uv run python my_tutorials/00_sim/launch_app.py --size 0.5
uv run python my_tutorials/00_sim/launch_app.py --size 2.0 --width 1920 --height 1080
```

### 確認ポイント

- [ ] `--size` に応じて立方体の大きさが変わり、常に地面の上に乗っている
- [ ] `--help` に自作引数と共通引数の両方が出る

### 発展課題

- `--color r g b`（`nargs=3`）を追加して立方体の色を変える
- `--num` で立方体の個数を指定して横に並べる

---

## Lesson 4: `log_time.py` — ループ中のログ出力

### 学ぶこと

- `sim.get_physics_dt()` で 1 ステップの時間を取得し、シミュレーション時刻を自分で積算する。
- ループ内でファイルに書き出す（元は Docker 内からログを取り出すチュートリアル用）。

### 自分で書くステップ

1. Lesson 1 をベースに、`logs/docker_tutorial/` ディレクトリを作る処理を追加（`os.makedirs(..., exist_ok=True)` で書いても良い）。
2. `sim_dt = sim.get_physics_dt()`, `sim_time = 0.0` を用意。
3. `with open(...) as log_file:` の中でループし、毎ステップ `sim_time` を書く。

### 確認ポイント

- [ ] `logs/docker_tutorial/log.txt` に 0.0, 0.01, 0.02, ... と増えていく値が並ぶ
- [ ] `--viz` なし（ヘッドレス）で実行し、Ctrl+C で止めてもファイルが残る

### 発展課題

- 「ステップ数・実時間（`time.time()`）・sim 時間」を並べて記録し、リアルタイム比（sim 時間 / 実時間）を計算する
- 次章（01_assets）で物体の位置を取得できるようになったら、このログに位置を足してみる

---

## 章末チェック

- [ ] 白紙から `create_empty.py` 相当を 5 分以内に書ける
- [ ] 「import を引数パースの後に書く理由」を説明できる
- [ ] spawner の `cfg.func(path, cfg, translation=...)` パターンを説明できる
- [ ] 見た目だけのプリムと剛体プリムの違いを説明できる

次へ → [01_assets.md](01_assets.md)
