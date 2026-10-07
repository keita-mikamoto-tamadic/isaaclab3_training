# 02_scene — InteractiveScene でシーンをまとめて複製する

対応スクリプト: [scripts/tutorials/02_scene/](../../IsaacLab/scripts/tutorials/02_scene/)

## この章のゴール

- 01_assets の `design_scene()` 手書き方式を、`InteractiveSceneCfg` のサブクラス（宣言的な設定クラス）に置き換えられる
- `{ENV_REGEX_NS}` を使ったプリムパスで、エンティティを `num_envs` 個の環境に自動複製（クローン）する仕組みを理解する
- ループ内の `reset` / `write_data_to_sim` / `update` を、アセット個別ではなく `scene` 1 つに任せられる
- 次章（03_envs）の `ManagerBasedEnv` / `DirectRLEnv` が内部で使う「シーン」の正体を知る

## 進める順番

| # | スクリプト | テーマ | 目安 |
|---|-----------|--------|------|
| 1 | `create_scene.py` | `run_articulation.py` を InteractiveScene で書き直す | 1〜1.5時間 |
| 2 | （自作）`create_scene.py` の拡張 | 剛体の追加・`get_state` / `reset_to`・環境数のスケール | 1時間 |

この章の元スクリプトは 1 本だけです。Lesson 2 は対応スクリプトの無い練習で、Lesson 1 の自作ファイルを拡張します。

---

## Lesson 1: `create_scene.py` — InteractiveScene への置き換え

### 学ぶこと

- **01_assets との違い**: `run_articulation.py` では次をすべて手で書いていた。
  - 地面・ライトを `cfg.func(path, cfg)` でスポーン
  - `/World/Origin1`, `/World/Origin2` の Xform を作り、`"/World/Origin.*/Robot"` で 2 台だけ配置
  - 原点リスト `origins` を自分で持ち回り、リセット時に足す
  - `robot.reset()` / `robot.write_data_to_sim()` / `robot.update(dt)` をアセットごとに呼ぶ

  InteractiveScene はこれらを肩代わりする: **スポーン・環境の複製・原点の管理・全エンティティへの一括操作**。
- **`InteractiveSceneCfg` を継承した `@configclass`**: クラス属性として各エンティティの Cfg を並べる。属性名がそのまま `scene["名前"]` のキーになる。
  - `num_envs`, `env_spacing` は必須（`MISSING`）。インスタンス化時に `CartpoleSceneCfg(num_envs=..., env_spacing=2.0)` で渡す。
- **非インタラクティブ vs インタラクティブ**:
  - 地面・ライト → `AssetBaseCfg(prim_path=..., spawn=...)`。スポーンされるだけで、`scene.extras` に入り reset/update の対象外。
  - カートポール → `ArticulationCfg`。`scene.articulations` に入り、一括操作の対象になる。
- **`{ENV_REGEX_NS}` マクロ**: `prim_path="{ENV_REGEX_NS}/Robot"` と書くと環境ごとに複製される。デフォルトの環境テンプレートは `/World/envs/env_{}` なので、実体は `/World/envs/env_0/Robot`, `/World/envs/env_1/Robot`, ...。マクロ自体は正規表現 `/World/envs/env_[^/]+` に展開される（01 の `Origin.*` と同じ発想を自動化したもの）。
  - `/World/...` の絶対パスで書いたもの（地面・ライト）は複製されず 1 つだけ。
- **`replace(CARTPOLE_CFG, prim_path=...)`**: 定義済み Cfg をコピーしつつ一部フィールドだけ変える（元の `CARTPOLE_CFG` は変更されない）。01 の `clone()` してから代入、を 1 行にしたもの。
- **`instantiate(scene_cfg)`**: `cfg.class_type(cfg)` を呼ぶだけのヘルパ。`InteractiveSceneCfg.class_type` は文字列で遅延解決されるので、Cfg はファイル冒頭で定義しても、シーン本体の import は `launch_simulation` の中で起きる。
  - そのためスクリプトは `InteractiveScene` を `if TYPE_CHECKING:` の中でだけ import している（型ヒント用）。
- **`scene.env_origins`**: 形状 `(num_envs, 3)` の各環境原点（グリッド配置、間隔 `env_spacing`）。`default_root_pose` は環境ローカルなので、書き込み前に足す必要がある点は 01 と同じ。
- **一括操作**: `scene.reset()` / `scene.write_data_to_sim()` / `scene.update(dt)` は、内部で全アーティキュレーション・剛体・センサー等の同名メソッドを呼ぶだけ。

### 自分で書くステップ

1. 01_assets の自作 `run_articulation.py` をコピーして出発点にする。
2. 引数に `--num_envs`（int, default 2）を追加（`add_launcher_args` の前）。
3. パース後の import に `AssetBaseCfg`, `ArticulationCfg`, `InteractiveSceneCfg`, `configclass`, `instantiate`, `replace` を追加。`InteractiveScene` は `TYPE_CHECKING` ブロックで import。
4. `design_scene()` を削除し、代わりに `@configclass class CartpoleSceneCfg(InteractiveSceneCfg):` を書く。
   - `ground`: `AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())`
   - `dome_light`: `AssetBaseCfg(prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(...))`
   - `cartpole: ArticulationCfg = replace(CARTPOLE_CFG, prim_path="{ENV_REGEX_NS}/Robot")`
5. `main()` で `scene_cfg = CartpoleSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)` → `scene = instantiate(scene_cfg)`。**`sim.reset()` より前**に作ること。
6. `run_simulator(sim, scene)` にシグネチャを変え、`robot = scene["cartpole"]` で取り出す。
7. ループ内を書き換える:
   - `origins` → `scene.env_origins`
   - `robot.reset()` → `scene.reset()`
   - `robot.write_data_to_sim()` → `scene.write_data_to_sim()`
   - `robot.update(sim_dt)` → `scene.update(sim_dt)`
8. 力の指令は 01 と同じく `robot.actuators.target_command.set_effort_index(value=efforts)` で書いてよい（下の注意参照）。

まず `--num_envs 2` で 01 と同じ見た目になることを確認してから、数を増やすのがおすすめです。

> **注意（Isaac Lab 3.0）**: 元の `create_scene.py` は `robot.set_joint_effort_target_index(target=efforts)` を使っていますが、このメソッドは**非推奨**（deprecated）で、ActuatorCollection 経由の `set_effort_index` が推奨されています。01 の書き方に揃えておくと後で困りません。

### 実行

```bash
uv run python my_tutorials/02_scene/create_scene.py
uv run python my_tutorials/02_scene/create_scene.py --num_envs 32
uv run python my_tutorials/02_scene/create_scene.py --num_envs 32
```

### 確認ポイント

- [ ] `--num_envs 32` でカートポールがグリッド状に 32 台並び、500 ステップごとにリセットされる
- [ ] `--viz kit` で開いた GUI の Stage ツリーで `/World/envs/env_0/Robot`, `env_1/Robot`, ... が見え、地面・ライトは `/World` 直下に 1 つだけある
- [ ] `print(scene)` で環境数・間隔・ソースプリム名が表示される
- [ ] `print(scene.keys())` に `cartpole`, `ground`, `dome_light` が含まれる
- [ ] `root_pose[:, :3] += scene.env_origins` を消すと、全台が原点に重なることを説明できる

### 発展課題

- `env_spacing` を 0.5 / 5.0 に変えて見た目を比べる（0.5 だと隣とぶつかりそうだが、`filter_collisions=True`（デフォルト）なので環境間の衝突は起きない）
- `scene.articulations`, `scene.extras` を print し、各エンティティがどの辞書に振り分けられたか確認する
- `scene.env_origins` の形状と値を print し、グリッド配置の規則を読み取る
- `scene["dome_light"]` にアクセスしてみる。`scene["xxx"]`（存在しないキー）で出る `KeyError` のメッセージも見ておく

### 参照コード

- [create_scene.py](../../IsaacLab/scripts/tutorials/02_scene/create_scene.py)
- [interactive_scene_cfg.py: InteractiveSceneCfg](../../IsaacLab/source/isaaclab/isaaclab/scene/interactive_scene_cfg.py#L22)（`num_envs` L86, `env_spacing` L89, `replicate_physics` L103, `filter_collisions` L126）
- [interactive_scene.py: InteractiveScene](../../IsaacLab/source/isaaclab/isaaclab/scene/interactive_scene.py#L58)
- [interactive_scene.py: \_\_init\_\_](../../IsaacLab/source/isaaclab/isaaclab/scene/interactive_scene.py#L120)
- [interactive_scene.py: env_origins](../../IsaacLab/source/isaaclab/isaaclab/scene/interactive_scene.py#L336)
- [interactive_scene.py: reset / write_data_to_sim / update](../../IsaacLab/source/isaaclab/isaaclab/scene/interactive_scene.py#L437)
- [interactive_scene.py: \_\_getitem\_\_](../../IsaacLab/source/isaaclab/isaaclab/scene/interactive_scene.py#L720)
- [interactive_scene.py: \_add_entities_from_cfg](../../IsaacLab/source/isaaclab/isaaclab/scene/interactive_scene.py#L757)（Cfg の型ごとにどの辞書へ入るか）
- [cloner_cfg.py: DEFAULT_ENV_TEMPLATE / expand_env_regex_ns](../../IsaacLab/source/isaaclab/isaaclab/cloner/cloner_cfg.py#L16)
- [configclass.py: instantiate / replace / clone](../../IsaacLab/source/isaaclab/isaaclab/utils/configclass.py#L127)
- [asset_base_cfg.py: AssetBaseCfg](../../IsaacLab/source/isaaclab/isaaclab/assets/asset_base_cfg.py#L19)
- [cartpole.py: CARTPOLE_CFG](../../IsaacLab/source/isaaclab_assets/isaaclab_assets/robots/cartpole.py#L20)
- [base_articulation.py: set_joint_effort_target_index（非推奨）](../../IsaacLab/source/isaaclab/isaaclab/assets/articulation/base_articulation.py#L1837)
- [actuator_collection.py: set_effort_index](../../IsaacLab/source/isaaclab/isaaclab/actuators/actuator_collection.py#L755)
- ドキュメント: [docs/source/how-to/create_scene.rst](../../IsaacLab/docs/source/how-to/create_scene.rst)

---

## Lesson 2: シーンを拡張する（自作）

### 学ぶこと

- シーンへのエンティティ追加は **Cfg に 1 行足すだけ**で、スポーン・複製・一括 reset/update が自動で付いてくる。
- `RigidObjectCfg` を追加すると `scene.rigid_objects` に入り、`scene.reset()` 等の対象になる。
- **シーン状態のスナップショット**:
  - `scene.get_state(is_relative=False)` → `{"articulation": {"cartpole": {"root_pose", "root_velocity", "joint_position", "joint_velocity"}}, "rigid_object": {...}, ...}` という入れ子 dict。
  - `scene.reset_to(state, env_ids=None, is_relative=False)` で、その状態に書き戻す。`is_relative=True` だと環境原点からの相対値として扱う。
- **宣言順について**: Cfg の docstring は「地形 → 物理アセット → センサー → ライト」の順を推奨している。3.0 の実装ではセンサーを最後に、浅いプリムパスを先に作るよう内部で並べ替えるが、推奨順で書く習慣をつけておく。

### 自分で書くステップ

1. Lesson 1 のファイルをコピー。
2. `CartpoleSceneCfg` に剛体キューブを追加:
   - `cube = RigidObjectCfg(prim_path="{ENV_REGEX_NS}/Cube", spawn=sim_utils.CuboidCfg(size=..., rigid_props=..., mass_props=..., collision_props=...), init_state=RigidObjectCfg.InitialStateCfg(pos=(...)))`
   - 00_sim の Lesson 2 で使った剛体の 3 点セットを思い出す。カートの横、少し上空に置く。
3. リセット処理でキューブも初期姿勢に戻す（`default_root_pose` + `scene.env_origins` → `write_root_pose_to_sim_index`）。
4. ステップ 0 の直後に `init_state = scene.get_state(is_relative=True)` を保存し、リセットを「手書きの書き込み」ではなく `scene.reset_to(init_state, is_relative=True)` に置き換えた版も作ってみる。
5. `env_ids` に一部の環境（例: 偶数番だけ）を渡して部分リセットを試す。

### 実行

```bash
uv run python my_tutorials/02_scene/create_scene_ext.py --num_envs 16
uv run python my_tutorials/02_scene/create_scene_ext.py --num_envs 1024
```

### 確認ポイント

- [ ] 各環境にキューブが 1 つずつ現れ、落下して地面に乗る
- [ ] `scene.keys()` に `cube` が増え、`scene.rigid_objects` に入っている
- [ ] `get_state` の dict の階層を print して説明できる
- [ ] `reset_to(..., is_relative=True)` で全環境がそれぞれの原点で初期状態に戻る
- [ ] 部分リセットで指定した環境だけが戻る

### 発展課題

- `--num_envs` を 1, 64, 1024, 4096 と増やし、起動時間とステップ速度（`time.time()` で計測）がどう変わるか記録する
- `is_relative=False` で保存した状態を `is_relative=True` で書き戻すと何が起きるか予想してから試す
- キューブの `prim_path` を `/World/Cube`（マクロ無し）にするとどうなるか確認する
- 環境ごとに異なるアセットを置く方法を [multi_asset_spawning.rst](../../IsaacLab/docs/source/how-to/multi_asset_spawning.rst) で読んでおく

### 参照コード

- [interactive_scene.py: reset_to](../../IsaacLab/source/isaaclab/isaaclab/scene/interactive_scene.py#L519)
- [interactive_scene.py: get_state](../../IsaacLab/source/isaaclab/isaaclab/scene/interactive_scene.py#L592)
- [rigid_object_cfg.py: RigidObjectCfg](../../IsaacLab/source/isaaclab/isaaclab/assets/rigid_object/rigid_object_cfg.py#L16)
- [clone_plan.py: grid_transforms](../../IsaacLab/source/isaaclab/isaaclab/cloner/clone_plan.py#L141)（環境原点のグリッド配置）
- ドキュメント: [docs/source/how-to/cloning.rst](../../IsaacLab/docs/source/how-to/cloning.rst)（クローンの仕組みの詳細）

---

## 章末チェック

- [ ] 白紙から `CartpoleSceneCfg` + `instantiate` + 一括ループの `create_scene.py` を 10 分以内に書ける
- [ ] `{ENV_REGEX_NS}` が何に展開され、絶対パスとどう違うか説明できる
- [ ] `AssetBaseCfg` と `ArticulationCfg` / `RigidObjectCfg` で、シーン内の扱い（extras か、reset/update 対象か）が違うことを説明できる
- [ ] `scene.env_origins` を足す理由を説明できる
- [ ] `run_articulation.py` と比べて、InteractiveScene が何を肩代わりしているか 3 つ挙げられる

次へ → [03_envs.md](03_envs.md)
