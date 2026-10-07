# 03_envs — マネージャベース環境（ManagerBasedEnv / ManagerBasedRLEnv）

対応スクリプト: [scripts/tutorials/03_envs/](../../IsaacLab/scripts/tutorials/03_envs/)

## この章のゴール

- 02_scene までの「シーン + 自前ループ」を、**設定クラス（Cfg）だけで組み立てる環境**に置き換えられる
- 行動・観測・イベント・報酬・終了の各**マネージャ**と、そこに登録する**項（Term）**の書き方を理解する
- `ManagerBasedEnv`（sense-act のみ）と `ManagerBasedRLEnv`（報酬・終了・エピソード管理付き）の違いを説明できる
- 学習済み（JIT）ポリシーを読み込んで環境上で推論を回せる

## 進める順番

| # | スクリプト | テーマ | 目安 |
|---|-----------|--------|------|
| 1 | `create_cartpole_base_env.py` | Cartpole を `ManagerBasedEnv` で作る（行動・観測・イベント） | 1〜1.5時間 |
| 2 | `run_cartpole_rl_env.py` | 登録済みタスク `Isaac-Cartpole`（`ManagerBasedRLEnv`）を読む・回す | 1時間 |
| 3 | `create_cube_base_env.py` | 自作 `ActionTerm`・自作観測関数・prestartup イベント | 1〜1.5時間 |
| 4 | `create_quadruped_base_env.py` | ANYmal-C + 高さスキャン + 学習済みポリシーで歩かせる | 1時間 |
| 5 | `policy_inference_in_usd.py` | 自分で学習した H1 ポリシーを倉庫 USD で推論 | 30分（＋学習時間） |

順番の理由: 1→2 は同じ Cartpole で「base → RL」の差分だけに集中できる。3 は既製の項ではなく**項を自作する**回で、4 で読む大量の既製項（観測ノイズ、センサ観測など）の理解の土台になる。4 で「Nucleus 上の学習済みポリシーを読み込む」流れを体験し、5 で「自分で学習 → エクスポート → 別シーンで推論」に進む。
（公式ドキュメントでは cube → quadruped の順に紹介されている。）

自作ファイルは `my_tutorials/03_envs/` に置く想定です。

関連ドキュメント（先に一読推奨）:

- [create_manager_base_env.rst](../../IsaacLab/docs/source/how-to/create_manager_base_env.rst)（Lesson 1, 3, 4）
- [create_manager_rl_env.rst](../../IsaacLab/docs/source/how-to/create_manager_rl_env.rst)（Lesson 2）
- [register_rl_env_gym.rst](../../IsaacLab/docs/source/how-to/register_rl_env_gym.rst)（Lesson 2 の `gym.register` 部分）
- [policy_inference_in_usd.rst](../../IsaacLab/docs/source/how-to/policy_inference_in_usd.rst)（Lesson 5）

---

## Lesson 1: `create_cartpole_base_env.py` — ManagerBasedEnv を Cfg で組む

### 学ぶこと

- **環境 = Cfg の集合**: `ManagerBasedEnvCfg` に `scene` / `observations` / `actions` / `events` を持たせ、`instantiate(env_cfg)` で `cfg.class_type`（= `ManagerBasedEnv`）が生成される。
- **シーンの再利用**: 02_scene で自作した Cartpole シーンと同等のものが `isaaclab_tasks` に `CartpoleSceneCfg` として用意されており、それを import して使う。
- **行動マネージャ**: `mdp.JointEffortActionCfg(asset_name="robot", joint_names=[...], scale=...)`。前章で `set_effort_index` 等を直接呼んでいた部分を肩代わりする。
- **観測マネージャ**: `ObservationGroupCfg`（`ObsGroup`）を継承したクラスに `ObservationTermCfg`（`ObsTerm`）を並べる。**宣言順が結合順**。グループ名（`policy`）が `obs` 辞書のキーになる。`concatenate_terms=True` で 1 本のテンソルに結合。
- **イベントマネージャ**: `EventTermCfg(func=..., mode=..., params=...)`。`mode` は `"startup"`（開始時1回）/ `"reset"`（リセットごと）/ `"interval"` / `"prestartup"`。
- `SceneEntityCfg("robot", joint_names=[...], body_names=[...])` で「どのアセットのどの関節/ボディか」を項に渡す。
- **decimation**: `sim.dt=0.005`（200Hz）、`decimation=4` → 環境ステップは 50Hz。`env.step()` 1回で物理が4回進む。
- `env.step(action)` の戻り値は `(obs, extras)` の **2 要素**（RL 環境とは違う）。

### 自分で書くステップ

1. 骨格（argparse + `add_launcher_args` + `--num_envs`）を書く。import は引数パースの後。
2. `import isaaclab.envs.mdp as mdp` と、`isaaclab.managers` から `EventTermCfg`, `ObservationGroupCfg`, `ObservationTermCfg`, `SceneEntityCfg` を import。`CartpoleSceneCfg` は `isaaclab_tasks.core.cartpole.cartpole_manager_env_cfg` から。
3. `@configclass class ActionsCfg:` に `JointEffortActionCfg` を 1 つ（`slider_to_cart`、`scale=5.0`）。
4. `ObservationsCfg` の中に入れ子で `PolicyCfg(ObsGroup)` を作り、`mdp.joint_pos_rel` / `mdp.joint_vel_rel` を `ObsTerm` で並べる。`__post_init__` で `enable_corruption=False`, `concatenate_terms=True`。外側に `policy: PolicyCfg = PolicyCfg()`。
5. `EventCfg`: startup で `mdp.randomize_rigid_body_mass`（pole に質量を add）、reset で `mdp.reset_joints_by_offset` を cart と pole それぞれに。
6. `CartpoleEnvCfg(ManagerBasedEnvCfg)` に 4 つを代入し、`__post_init__` で `decimation`, `sim.dt`, `sim.default_visualizer_cfg`（`VisualizerCfg(eye=..., lookat=...)`）を設定。
7. `main()` で cfg を作り `scene.num_envs` と `sim.device` を上書き → `with launch_simulation(env_cfg, args_cli):` の中で `env = instantiate(env_cfg)`。
8. ループは `torch.inference_mode()` 内で、300 ステップごとに `env.reset()`、行動は `torch.randn_like(env.action_manager.action)`、`obs, _ = env.step(...)`。最後に `env.close()`。

### 実行

```bash
uv run python my_tutorials/03_envs/create_cartpole_base_env.py --num_envs 32 --viz kit
```

### 確認ポイント

- [ ] 32 台の Cartpole がランダムな力でガタガタ動き、300 ステップごとにリセットされる
- [ ] `[Env 0]: Pole joint:` が出力される（`obs["policy"][0][1]` が **pole 関節角**である理由を、観測項の順番と関節の順番から説明できる）
- [ ] `env.action_manager.action` の shape が `(num_envs, 1)` である理由を説明できる
- [ ] 起動ログの `Physics step-size` / `Environment step-size` が 0.005 / 0.02 になっている
- [ ] 起動ログにマネージャごとの項一覧表（Observation / Action / Event Manager）が出るので確認する

### 発展課題

- `ObsTerm` の順番を入れ替え、`obs["policy"][0][1]` の意味が変わることを確認する
- `concatenate_terms=False` にすると `obs["policy"]` が何になるか print する
- `decimation` を 1 / 10 に変えて、同じ行動列で挙動がどう変わるか比べる
- 2 つ目の観測グループ（例: `critic`）を追加して `obs` 辞書のキーが増えることを確認する

### 参照コード

- [manager_based_env_cfg.py: ManagerBasedEnvCfg](../../IsaacLab/source/isaaclab/isaaclab/envs/manager_based_env_cfg.py#L43)
- [manager_based_env.py: ManagerBasedEnv](../../IsaacLab/source/isaaclab/isaaclab/envs/manager_based_env.py#L36)（docstring にマネージャ構成の説明あり）
- [manager_based_env.py: load_managers](../../IsaacLab/source/isaaclab/isaaclab/envs/manager_based_env.py#L340) / [step](../../IsaacLab/source/isaaclab/isaaclab/envs/manager_based_env.py#L518) / [reset](../../IsaacLab/source/isaaclab/isaaclab/envs/manager_based_env.py#L403)
- [manager_term_cfg.py: ObservationTermCfg](../../IsaacLab/source/isaaclab/isaaclab/managers/manager_term_cfg.py#L151) / [ObservationGroupCfg](../../IsaacLab/source/isaaclab/isaaclab/managers/manager_term_cfg.py#L239) / [EventTermCfg](../../IsaacLab/source/isaaclab/isaaclab/managers/manager_term_cfg.py#L300)
- [event_manager.py: EventManager（mode の説明）](../../IsaacLab/source/isaaclab/isaaclab/managers/event_manager.py#L27)
- [scene_entity_cfg.py: SceneEntityCfg](../../IsaacLab/source/isaaclab/isaaclab/managers/scene_entity_cfg.py#L23)
- [actions_cfg.py: JointEffortActionCfg](../../IsaacLab/source/isaaclab/isaaclab/envs/mdp/actions/actions_cfg.py#L125)
- [observations.py: joint_pos_rel](../../IsaacLab/source/isaaclab/isaaclab/envs/mdp/observations.py#L214)
- [events.py: randomize_rigid_body_mass](../../IsaacLab/source/isaaclab/isaaclab/envs/mdp/events.py#L233) / [reset_joints_by_offset](../../IsaacLab/source/isaaclab/isaaclab/envs/mdp/events.py#L1488)
- [configclass.py: instantiate](../../IsaacLab/source/isaaclab/isaaclab/utils/configclass.py#L127)
- [cartpole_manager_env_cfg.py: CartpoleSceneCfg](../../IsaacLab/source/isaaclab_tasks/isaaclab_tasks/core/cartpole/cartpole_manager_env_cfg.py#L34)

---

## Lesson 2: `run_cartpole_rl_env.py` — ManagerBasedRLEnv とタスク登録

このスクリプト自体は短いので、**主役はタスク側の設定ファイル** [cartpole_manager_env_cfg.py](../../IsaacLab/source/isaaclab_tasks/isaaclab_tasks/core/cartpole/cartpole_manager_env_cfg.py) です。まずこれを Lesson 1 の自作ファイルと見比べながら読んでください。

### 学ぶこと

- `ManagerBasedRLEnvCfg` は `ManagerBasedEnvCfg` に以下を追加したもの:
  - `rewards`: `RewardTermCfg(func, weight, params)` の集合。各項の値 × `weight` × `step_dt` の総和が報酬。
  - `terminations`: `TerminationTermCfg(func, params, time_out=...)`。`time_out=True` の項は **truncated**、それ以外は **terminated** 扱い。
  - `episode_length_s`: エピソード長（秒）。`mdp.time_out` がこれを見る。
  - `commands`, `curriculum`（Cartpole では未使用、Lesson 4〜5 の locomotion タスクで登場）
- `ManagerBasedRLEnv.step()` は `(obs, rew, terminated, truncated, info)` の **5 要素**（gymnasium 形式）を返し、終了した env は**内部で自動リセット**される。
- **タスク登録**: `isaaclab_tasks/core/cartpole/__init__.py` の `gym.register(id="Isaac-Cartpole", entry_point="isaaclab.envs:ManagerBasedRLEnv", kwargs={"env_cfg_entry_point": ...})`。
- `parse_env_cfg("Isaac-Cartpole", device=..., num_envs=..., overrides=...)` でタスク ID から cfg を取得する。`parser.parse_known_args()` で拾えなかった `key=value`（例: `physics=isaacsim_physx`）は Hydra 風オーバーライドとして渡される。
- `parser.set_defaults(visualizer=["kit"])` で `--viz kit` をデフォルトにしている。

### 自分で書くステップ

1. まず `cartpole_manager_env_cfg.py` を読み、`RewardsCfg`（6 項）と `TerminationsCfg`（2 項）の各項が何を表すか、自分の言葉でメモする。
2. Lesson 1 のファイルをコピー。自作の Cfg クラス群は全部削除する。
3. `args_cli, hydra_overrides = parser.parse_known_args()` に変更。
4. `from isaaclab_tasks.utils import parse_env_cfg` を import し、`parse_env_cfg("Isaac-Cartpole", ...)` で cfg を得る。
5. `instantiate(env_cfg)` で環境を作り（`class_type` が `ManagerBasedRLEnv` になっていることに注目）、`env.step()` の戻り値を 5 つで受ける。

### 実行

```bash
uv run python my_tutorials/03_envs/run_cartpole_rl_env.py --num_envs 32
uv run python my_tutorials/03_envs/run_cartpole_rl_env.py --num_envs 32 physics=isaacsim_physx  # オーバーライド例
```

### 確認ポイント

- [ ] Lesson 1 と違い、300 ステップ待たずに個々の Cartpole がリセットされることがある（カートが ±3m を超える or 5 秒経過）
- [ ] 起動ログに Reward Manager / Termination Manager の項一覧が出る
- [ ] Lesson 1 と比べて `scale`（5.0 vs 100.0）、イベントの範囲、`sim.dt`/`decimation` が違うことに気づける
- [ ] `terminated` と `truncated` の違いを説明できる

### 発展課題

- ループ内で `rew[0]`, `terminated[0]`, `truncated[0]` を print し、カートが端に行ったときの値を観察する
- `env.reward_manager._step_reward[0]` を print して項ごとの報酬内訳を覗く（列の順番は `env.reward_manager.active_terms`、[reward_manager.py](../../IsaacLab/source/isaaclab/isaaclab/managers/reward_manager.py) を参照）
- Lesson 1 の自作 cfg を `ManagerBasedRLEnvCfg` 継承に書き換え、`RewardsCfg` / `TerminationsCfg` を自分で足してみる（`episode_length_s` も忘れずに）
- 余力があれば `uv run isaaclab train --rl_library rsl_rl --task Isaac-Cartpole` で実際に学習させてみる（[register_rl_env_gym.rst](../../IsaacLab/docs/source/how-to/register_rl_env_gym.rst) も参照）

### 参照コード

- [cartpole_manager_env_cfg.py: RewardsCfg](../../IsaacLab/source/isaaclab_tasks/isaaclab_tasks/core/cartpole/cartpole_manager_env_cfg.py#L113) / [TerminationsCfg](../../IsaacLab/source/isaaclab_tasks/isaaclab_tasks/core/cartpole/cartpole_manager_env_cfg.py#L143) / [CartpoleEnvCfg](../../IsaacLab/source/isaaclab_tasks/isaaclab_tasks/core/cartpole/cartpole_manager_env_cfg.py#L161)
- [cartpole/\_\_init\_\_.py: gym.register("Isaac-Cartpole")](../../IsaacLab/source/isaaclab_tasks/isaaclab_tasks/core/cartpole/__init__.py#L54)
- [cartpole/mdp/](../../IsaacLab/source/isaaclab_tasks/isaaclab_tasks/core/cartpole/mdp/)（タスク固有の項）
- [manager_based_rl_env_cfg.py: ManagerBasedRLEnvCfg](../../IsaacLab/source/isaaclab/isaaclab/envs/manager_based_rl_env_cfg.py#L15)
- [manager_based_rl_env.py: ManagerBasedRLEnv](../../IsaacLab/source/isaaclab/isaaclab/envs/manager_based_rl_env.py#L22) / [step](../../IsaacLab/source/isaaclab/isaaclab/envs/manager_based_rl_env.py#L174)（base の step と差分を比べる）
- [manager_term_cfg.py: RewardTermCfg](../../IsaacLab/source/isaaclab/isaaclab/managers/manager_term_cfg.py#L379) / [TerminationTermCfg](../../IsaacLab/source/isaaclab/isaaclab/managers/manager_term_cfg.py#L407)
- [rewards.py: is_alive](../../IsaacLab/source/isaaclab/isaaclab/envs/mdp/rewards.py#L33) / [joint_pos_target_l2](../../IsaacLab/source/isaaclab/isaaclab/envs/mdp/rewards.py#L217)
- [terminations.py: time_out](../../IsaacLab/source/isaaclab/isaaclab/envs/mdp/terminations.py#L32) / [joint_pos_out_of_manual_limit](../../IsaacLab/source/isaaclab/isaaclab/envs/mdp/terminations.py#L106)
- [parse_cfg.py: parse_env_cfg](../../IsaacLab/source/isaaclab_tasks/isaaclab_tasks/utils/parse_cfg.py#L157)

---

## Lesson 3: `create_cube_base_env.py` — 項を自作する

### 学ぶこと

- **自作 ActionTerm**: `ActionTerm` を継承し、`action_dim` / `raw_actions` / `processed_actions` プロパティと、`process_actions(actions)`（**環境ステップごと**に1回）・`apply_actions()`（**物理ステップごと**に1回）を実装する。
- 対応する `ActionTermCfg` サブクラスで `class_type` に自作クラスを指定し、ゲイン等のパラメータも Cfg に持たせる。
- **自作観測関数**: `def f(env, asset_cfg) -> torch.Tensor` を書いて `ObsTerm(func=f, params={...})` に渡すだけ。`env.scene[asset_cfg.name]` でアセットを取る。
- **env 原点**: `asset.data.root_pos_w.torch - env.scene.env_origins` でワールド座標を各 env のローカル座標に直す。
- **prestartup イベント**: `mdp.randomize_rigid_body_scale` / `mdp.randomize_visual_color` は USD ステージを書き換えるので、シーンを `replicate_physics=False` にする必要がある。
- `PhysxRigidBodyCfg(disable_gravity=True)` で重力無効の浮遊キューブ（`isaaclab_physx` からの import = PhysX 専用設定）。

### 自分で書くステップ

1. シーン: `TerrainImporterCfg(terrain_type="plane")`、`RigidObjectCfg`（`CuboidCfg` + 重力無効 + 初期高さ 5m）、`DomeLightCfg`。
2. `CubeActionTerm(ActionTerm)` を書く。`__init__` で `(num_envs, 3)` の raw/processed バッファと `(num_envs, 6)` の速度指令バッファを確保。
3. `process_actions` は raw をコピーするだけ。`apply_actions` で位置誤差・速度誤差から PD で速度指令を作り、`self._asset.write_root_velocity_to_sim_index(root_velocity=...)` で書き込む。
4. `CubeActionTermCfg(ActionTermCfg)` に `class_type`, `p_gain`, `d_gain`。
5. 観測関数 `base_position(env, asset_cfg)` を書き、`PolicyCfg` に登録。
6. イベント: reset で `mdp.reset_root_state_uniform`、prestartup でスケールと色のランダム化。
7. `CubeEnvCfg` でシーンに `replicate_physics=False`、`decimation=2`, `sim.dt=0.01`, `sim.render_interval=2`。
8. `main()` では、目標位置 `target_position`（`(num_envs, 3)`）を作り、それを**行動として** `env.step(target_position)` に渡す。観測と目標の誤差を print。

### 実行

```bash
uv run python my_tutorials/03_envs/create_cube_base_env.py --num_envs 32 --viz kit
```

### 確認ポイント

- [ ] キューブの大きさ・色が env ごとにバラバラ
- [ ] `Mean position error` がステップとともに小さくなる
- [ ] 元スクリプトでは目標から `env_origins` を引いているので、全キューブが**ワールド原点付近**へ集まる。その理由を説明できる
- [ ] `process_actions` と `apply_actions` の呼ばれる頻度の違いを説明できる（`ActionManager` と `ManagerBasedEnv.step` を読んで確認）

### 発展課題

- `target_position -= env.scene.env_origins` を消すと、各キューブが自分の env 内の目標へ向かうことを確認する
- `p_gain` / `d_gain` を Cfg 側で変えて応答の違いを観察する
- `process_actions` で行動を `[-1, 1]` にクリップしてからスケールする処理を入れる
- `replicate_physics=True` に戻すと何が起きるか（エラー/警告メッセージを読む）

### 参照コード

- [action_manager.py: ActionTerm](../../IsaacLab/source/isaaclab/isaaclab/managers/action_manager.py#L29)（[process_actions](../../IsaacLab/source/isaaclab/isaaclab/managers/action_manager.py#L150) / [apply_actions](../../IsaacLab/source/isaaclab/isaaclab/managers/action_manager.py#L162)）
- [manager_term_cfg.py: ActionTermCfg](../../IsaacLab/source/isaaclab/isaaclab/managers/manager_term_cfg.py#L77)
- [envs/mdp/actions/](../../IsaacLab/source/isaaclab/isaaclab/envs/mdp/actions/)（既製の ActionTerm 実装。自作の参考に）
- [events.py: randomize_rigid_body_scale](../../IsaacLab/source/isaaclab/isaaclab/envs/mdp/events.py#L41) / [reset_root_state_uniform](../../IsaacLab/source/isaaclab/isaaclab/envs/mdp/events.py#L1267) / [randomize_visual_color](../../IsaacLab/source/isaaclab/isaaclab/envs/mdp/events.py#L1825)

---

## Lesson 4: `create_quadruped_base_env.py` — 四脚ロボットと学習済みポリシー

### 学ぶこと

- **地形**: `TerrainImporterCfg(terrain_type="generator", terrain_generator=ROUGH_TERRAINS_CFG)` で凸凹地形を生成。
- **センサをシーンに置く**: `RayCasterCfg` + `patterns.GridPatternCfg` で高さスキャン（詳細は次章 04_sensors）。`update_period = decimation * sim.dt` で環境ステップに合わせる。
- **観測ノイズ**: `ObsTerm(..., noise=UniformNoiseCfg(n_min, n_max), clip=(...))`。`enable_corruption=True` のときだけノイズが加わる。
- 既製の観測項（`base_lin_vel`, `projected_gravity`, `last_action`, `height_scan` など）と、コマンドの代わりに定数を返す自作項 `constant_commands`。
- `JointPositionActionCfg(joint_names=[".*"], scale=0.5, use_default_offset=True)`: 正規表現で全関節、デフォルト姿勢からのオフセットとして位置指令。
- **学習済みポリシーの読み込み**: `check_file_path` → `read_file`（Nucleus 上のファイルも BytesIO で取れる）→ `torch.jit.load(...).to(env.device).eval()` → `policy(obs["policy"])`。
- **観測の並びはポリシーの入力仕様そのもの**。項の順番やスケールを変えると学習済みポリシーは動かなくなる。

### 自分で書くステップ

1. `MySceneCfg`: 地形・ロボット（`replace(ANYMAL_C_CFG, prim_path="{ENV_REGEX_NS}/Robot")`）・`height_scanner`・ライト。
2. `ActionsCfg` に `JointPositionActionCfg`。
3. `constant_commands(env)` を書く（`[1, 0, 0]` を `num_envs` 分）。
4. `PolicyCfg` に元スクリプトと**同じ順番**で 8 項を並べる（ノイズ・clip も）。
5. `EventCfg` は `mdp.reset_scene_to_default` 1 つだけ。
6. `QuadrupedEnvCfg.__post_init__` で `decimation`, `sim.dt`, `sim.physics_material`, センサの `update_period`。
7. `main()` で環境を作り、`ISAACLAB_NUCLEUS_DIR + "/Policies/ANYmal-C/HeightScan/policy.pt"` を読み込み、`policy(obs["policy"])` → `env.step(action)` のループ。

### 実行

```bash
uv run python my_tutorials/03_envs/create_quadruped_base_env.py --num_envs 32 --viz kit
```

注意: ポリシーファイルは Nucleus（ネットワーク）から取得されるため、初回はネットワーク接続が必要。

### 確認ポイント

- [ ] ANYmal-C が凸凹地形の上を前進する
- [ ] 高さスキャンのレイ（デバッグ表示）が足元に見える
- [ ] 観測項の順番を 1 つ入れ替えると歩けなくなることを確認し、理由を説明できる

### 発展課題

- `constant_commands` を `[0, 1, 0]`（横移動）や `[0, 0, 1]`（旋回）にしてみる
- `enable_corruption=False` にしてノイズなしでの挙動と比べる
- `obs["policy"].shape` を print し、各項の次元の合計と一致するか計算する（高さスキャンの点数 = グリッドサイズ / 解像度）

### 参照コード

- [actions_cfg.py: JointPositionActionCfg](../../IsaacLab/source/isaaclab/isaaclab/envs/mdp/actions/actions_cfg.py#L74)
- [observations.py: base_lin_vel](../../IsaacLab/source/isaaclab/isaaclab/envs/mdp/observations.py#L67) / [projected_gravity](../../IsaacLab/source/isaaclab/isaaclab/envs/mdp/observations.py#L85) / [height_scan](../../IsaacLab/source/isaaclab/isaaclab/envs/mdp/observations.py#L293) / [last_action](../../IsaacLab/source/isaaclab/isaaclab/envs/mdp/observations.py#L891)
- [events.py: reset_scene_to_default](../../IsaacLab/source/isaaclab/isaaclab/envs/mdp/events.py#L1723)
- [assets.py: check_file_path](../../IsaacLab/source/isaaclab/isaaclab/utils/assets.py#L587) / [read_file](../../IsaacLab/source/isaaclab/isaaclab/utils/assets.py#L744)
- [velocity_env_cfg.py](../../IsaacLab/source/isaaclab_tasks/isaaclab_tasks/core/velocity/velocity_env_cfg.py)（同じ構成を `ManagerBasedRLEnvCfg` で本格的に書いた locomotion タスク。`CommandsCfg` や `CurriculumCfg` も見られる）

---

## Lesson 5: `policy_inference_in_usd.py` — 自分で学習したポリシーを別シーンで推論

### 学ぶこと

- 既存タスク（`Isaac-Velocity-Rough-H1`）の cfg を `parse_env_cfg` で取り、**一部だけ差し替えて**別の用途に使うパターン。
- `env_cfg.play_mode()`: 推論用の上書き（env 数を最大 50 に、観測ノイズ無効化、地形縮小、外乱イベント無効化など）。
- 地形を `TerrainImporterCfg(terrain_type="usd", usd_path=.../warehouse.usd)` に差し替え、`curriculum=None`。屋根のある倉庫なので高さスキャナのオフセットを 2m に下げる。
- `--device cpu` のときだけ `sim.use_fabric=False`。
- **学習・エクスポート・推論で同じ physics preset を使う**こと（docs では `physics=newton_mjwarp`）。

### 前提（重要）

このスクリプトは `--checkpoint`（JIT エクスポート済みポリシー）が**必須**で、同梱のチェックポイントはありません。先に以下を行う:

1. 学習: `uv run isaaclab train --rl_library rsl_rl --task Isaac-Velocity-Rough-H1 physics=newton_mjwarp`（数時間単位で GPU を使う）
2. play でエクスポート: `uv run isaaclab play --rl_library rsl_rl --task Isaac-Velocity-Rough-H1 physics=newton_mjwarp --num_envs 64 --checkpoint logs/rsl_rl/h1_rough/<EXPERIMENT>/<model>.pt --viz newton_gl`
   → `logs/rsl_rl/h1_rough/<EXPERIMENT>/exported/policy.pt` ができる
3. 倉庫 USD は `ISAAC_NUCLEUS_DIR` から取得されるのでネットワーク接続が必要。

### 自分で書くステップ

1. `--checkpoint`（`required=True`）を追加し、`parse_known_args()` で Hydra オーバーライドも受け取る。
2. `read_file` → `torch.jit.load(file, map_location=args_cli.device)` でポリシーを読む。
3. `parse_env_cfg("Isaac-Velocity-Rough-H1", num_envs=1, overrides=...)` → `play_mode()` → `curriculum=None` → 地形差し替え → `scene.height_scanner.offset.pos` 変更。
4. `launch_simulation` 内で `instantiate`、`obs, _ = env.reset()` してから推論ループ（戻り値は 5 要素）。

### 実行

```bash
uv run python my_tutorials/03_envs/policy_inference_in_usd.py \
  --checkpoint logs/rsl_rl/h1_rough/<EXPERIMENT>/exported/policy.pt physics=newton_mjwarp --viz newton_gl
```

### 確認ポイント

- [ ] H1 が倉庫の床の上を歩く
- [ ] `play_mode()` を呼ばないと何が変わるか（env 数、ノイズ）をソースから説明できる
- [ ] 高さスキャナのオフセットを戻す（20m のまま）と屋根に当たって歩行が崩れる理由を説明できる

### 発展課題

- 学習時と違う physics preset で推論するとどうなるか試す（クロスバックエンドは別途検証が必要、と docs にある）
- `env_cfg.commands` を読み、速度コマンドの範囲を固定値に変えて前進のみにする
- Lesson 4 の ANYmal-C ポリシーを、同じ要領で倉庫 USD に置いて動かしてみる

### 参照コード

- [h1/\_\_init\_\_.py: gym.register("Isaac-Velocity-Rough-H1")](../../IsaacLab/source/isaaclab_tasks/isaaclab_tasks/core/velocity/config/h1/__init__.py#L28)
- [h1/rough_env_cfg.py: H1RoughEnvCfg](../../IsaacLab/source/isaaclab_tasks/isaaclab_tasks/core/velocity/config/h1/rough_env_cfg.py#L73)
- [velocity_env_cfg.py: LocomotionVelocityRoughEnvCfg](../../IsaacLab/source/isaaclab_tasks/isaaclab_tasks/core/velocity/velocity_env_cfg.py#L335) / [play_mode](../../IsaacLab/source/isaaclab_tasks/isaaclab_tasks/core/velocity/velocity_env_cfg.py#L377)
- [manager_based_rl_env_cfg.py: play_mode（基底）](../../IsaacLab/source/isaaclab/isaaclab/envs/manager_based_rl_env_cfg.py#L101)
- [policy_inference_in_usd.rst](../../IsaacLab/docs/source/how-to/policy_inference_in_usd.rst)

---

## 章末チェック

- [ ] 白紙から Lesson 1 相当（行動・観測・イベント付きの `ManagerBasedEnvCfg`）を 20 分以内に書ける
- [ ] `ManagerBasedEnv` と `ManagerBasedRLEnv` の違い（追加マネージャ、`step` の戻り値、自動リセット）を説明できる
- [ ] `sim.dt` と `decimation` と `step_dt` の関係を説明できる
- [ ] Term の Cfg（`func` / `params` / `weight` / `mode`）と `SceneEntityCfg` の役割を説明できる
- [ ] 自作 ActionTerm の `process_actions` と `apply_actions` の役割分担を説明できる
- [ ] `gym.register` → `parse_env_cfg` → `instantiate` の流れでタスクが生成されることを説明できる

次へ → [04_sensors.md](04_sensors.md)
