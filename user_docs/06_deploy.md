# 06_deploy — LEAPP でポリシーをエクスポートする

対応スクリプト: [scripts/tutorials/06_deploy/](../../IsaacLab/scripts/tutorials/06_deploy/)

## この章のゴール

- Direct ワークフローの環境（`DirectRLEnv`）で「観測 → ポリシー → 行動後処理」がどこで計算されているかを説明できる
- LEAPP の 4 つのアノテーション（`input_tensors` / `output_tensors` / `state_tensors` / `update_state`）を正しい場所に書ける
- 学習済みチェックポイントを `.onnx` + `.yaml` のデプロイ用パッケージにエクスポートし、中身を確認できる

## この章の注意（先に読む）

- この章のスクリプトは `anymal_c_env.py` **1 本だけ**で、しかも**単体では実行できません**。これは登録済みタスク `IsaacContrib-Velocity-{Flat,Rough}-AnymalC-Direct` の環境クラス [anymal_c_env.py（オリジナル）](../../IsaacLab/source/isaaclab_tasks/isaaclab_tasks/contrib/anymal_c_direct/anymal_c_env.py) に LEAPP アノテーションを足した「差し替え版」です（`from .anymal_c_env_cfg import ...` という相対 import がその証拠）。
- 公式ドキュメントは `cp scripts/tutorials/06_deploy/anymal_c_env.py source/isaaclab_tasks/...` で**ソースを上書き**する手順ですが、学習用には下記の「自分のファイルを gym に登録し直すランナー」方式をおすすめします（ソースを汚さない）。もし上書きした場合は `git checkout source/isaaclab_tasks/isaaclab_tasks/contrib/anymal_c_direct/anymal_c_env.py` で戻せます。
- LEAPP は**オプション extra**（`--extra leapp`、`--extra all` にも含まれない）。`from leapp import annotate` を書いたファイルは leapp 未インストールだと import 時点で落ちます。
- このタスクの cfg は `PhysxCfg` 固定（Newton プリセットなし）なので、Isaac Sim（`--extra isaacsim`）で動かします。
- エクスポートには**学習済みチェックポイントが必要**です。contrib タスクは公開済み pretrained の対象外なので、基本は自分で短く学習します（Lesson 3）。

## 進める順番

| # | スクリプト | テーマ | 目安 |
|---|-----------|--------|------|
| 1 | `anymal_c_env.py`（アノテーションなし版） | Direct 環境を写経し、自作ランナーで動かす | 1.5〜2時間 |
| 2 | `anymal_c_env.py`（完成版） | LEAPP アノテーションを足す | 1時間 |
| 3 | `export_my_env.py`（自作） | 学習 → エクスポート → 成果物の確認 | 1〜2時間（学習時間込み） |
| 4 | （コマンドのみ） | manager-based 環境のエクスポートとデプロイを比較 | 30分〜1時間 |

自作ファイルは `my_tutorials/06_deploy/` に置きます。最終的に `anymal_c_env.py`, `run_my_env.py`, `export_my_env.py` の 3 ファイルになります。

---

## Lesson 1: `anymal_c_env.py` — Direct 環境を写経して動かす

### 学ぶこと

- `DirectRLEnv` のフック: `_pre_physics_step`（ポリシー出力の前処理、1 env step に 1 回）→ `_apply_action`（decimation 回呼ばれる）→ `_get_observations` / `_get_rewards` / `_get_dones` / `_reset_idx`。
- 観測 48 次元の内訳: `root_lin_vel_b`(3) + `root_ang_vel_b`(3) + `projected_gravity_b`(3) + `commands`(3) + `joint_pos - default_joint_pos`(12) + `joint_vel`(12) + `previous_actions`(12)。Rough では `height_data`（高さスキャン）が加わり 235 次元。
- 行動の後処理: `processed = action_scale * actions + default_joint_pos` を関節位置目標にする。
- 3.0 のデータアクセス: `self._robot.data.joint_pos.torch` のように `.torch` で torch テンソルを取る。書き込みは `write_root_pose_to_sim_index(...)` などの `*_index` 系。
- `ContactSensor.find_sensors(".*FOOT")` で足のインデックスを引き、`compute_first_contact` / `last_air_time` で足の滞空時間報酬を作る。
- **自作ファイルを登録済みタスクに差し替えるテクニック**: `gym.spec(id)` を取り出して `gym.registry.pop(id)` → 同じ kwargs で `entry_point` だけ自作クラスに変えて `gym.register`。

### 自分で書くステップ

1. まずオリジナル [anymal_c_env.py](../../IsaacLab/source/isaaclab_tasks/isaaclab_tasks/contrib/anymal_c_direct/anymal_c_env.py) と [anymal_c_env_cfg.py](../../IsaacLab/source/isaaclab_tasks/isaaclab_tasks/contrib/anymal_c_direct/anymal_c_env_cfg.py) を読み、cfg の `action_scale`, `decimation`, `observation_space`, 報酬スケールを把握する。
2. `my_tutorials/06_deploy/anymal_c_env.py` を作り、`class AnymalCEnv(DirectRLEnv)` を**アノテーションなしで**書く。cfg は `from .anymal_c_env_cfg import AnymalCFlatEnvCfg, AnymalCRoughEnvCfg` と相対 import のままで OK（理由はステップ 5）。
   - `__init__`: `self.scene["robot"]` などを取得、`_actions` / `_previous_actions` / `_commands` / `_episode_sums` を確保、`find_sensors` で base / FOOT / THIGH のインデックス。
   - `_pre_physics_step` → `_apply_action`（`set_joint_position_target_index`）。
   - `_get_observations`: 上の順に `torch.cat`。Rough のときだけ `height_data` を入れる（`isinstance(self.cfg, AnymalCRoughEnvCfg)`）。
   - `_get_rewards`: 10 項目を dict にして合計し、`_episode_sums` に積算。
   - `_get_dones`: base の接触力が 1.0 を超えたら `died`、時間切れで `time_out`。
   - `_reset_idx`: `index_fill_` でバッファをゼロ化、コマンドを `uniform_(-1, 1)` で再サンプル、ルート/関節状態を書き戻し、`extras["log"]` に記録。
3. `my_tutorials/06_deploy/run_my_env.py` を作る。骨格は 07 章の `run_tiled_camera_visualizer.py` と同じ（`add_launcher_args` → `setup_preset_cli` → `sys.argv` 書き換え → `resolve_task_config(task, "")` → `launch_simulation` → `gym.make` → ランダム行動ループ）。
4. ランナーで `import isaaclab_tasks.contrib.anymal_c_direct`（登録を走らせる）した後、タスク ID の登録を差し替える。`entry_point` には**関数**（ファクトリ）を渡す。
5. ファクトリの中で `importlib.util.spec_from_file_location("isaaclab_tasks.contrib.anymal_c_direct._my_env", path)` → `module_from_spec` → `exec_module` して `module.AnymalCEnv(**kwargs)` を返す。モジュール名をタスクパッケージ配下にしておくと、ファイル内の相対 import `.anymal_c_env_cfg` が解決できる。ファクトリは `gym.make` 時（＝ランタイム起動後）に呼ばれるので import 順の問題も起きない。

```python
# ファクトリのシグネチャのイメージ
def _make_my_env(**kwargs): ...   # kwargs には cfg, render_mode が入る
```

### 実行

```bash
uv run python my_tutorials/06_deploy/run_my_env.py --num_envs 16
uv run python my_tutorials/06_deploy/run_my_env.py --num_envs 16 --task IsaacContrib-Velocity-Rough-AnymalC-Direct
```

### 確認ポイント

- [ ] 16 体の ANYmal-C がランダム行動でバタバタ動き、倒れた個体がリセットされる
- [ ] `env.observation_space` が Flat で 48、Rough で 235 次元になっている
- [ ] 自作クラスが本当に使われている（`__init__` に `print` を一時的に入れて確認）

### 発展課題

- `_get_observations` に「各項目の次元」を print して合計 48 を自分で確かめる
- `self.cfg.decimation` 回 `_apply_action` が呼ばれることをカウンタで確認する（Lesson 2 の「出力アノテーションを `_apply_action` に置いてはいけない理由」につながる）

### 参照コード

- [direct_rl_env.py: DirectRLEnv](../../IsaacLab/source/isaaclab/isaaclab/envs/direct_rl_env.py#L37) / [step](../../IsaacLab/source/isaaclab/isaaclab/envs/direct_rl_env.py#L358) / [_pre_physics_step 以降のフック](../../IsaacLab/source/isaaclab/isaaclab/envs/direct_rl_env.py#L718)
- [anymal_c_env_cfg.py: AnymalCFlatEnvCfg](../../IsaacLab/source/isaaclab_tasks/isaaclab_tasks/contrib/anymal_c_direct/anymal_c_env_cfg.py#L109) / [AnymalCRoughEnvCfg](../../IsaacLab/source/isaaclab_tasks/isaaclab_tasks/contrib/anymal_c_direct/anymal_c_env_cfg.py#L161)
- [anymal_c_direct/\_\_init\_\_.py: タスク登録](../../IsaacLab/source/isaaclab_tasks/isaaclab_tasks/contrib/anymal_c_direct/__init__.py)
- [base_contact_sensor.py: find_sensors](../../IsaacLab/source/isaaclab/isaaclab/sensors/contact_sensor/base_contact_sensor.py#L135) / [compute_first_contact](../../IsaacLab/source/isaaclab/isaaclab/sensors/contact_sensor/base_contact_sensor.py#L168)
- [utils/array.py: index_fill_](../../IsaacLab/source/isaaclab/isaaclab/utils/array.py#L47)
- 差し替えテクニックの実例: [leapp_direct_env.py（テスト）](../../IsaacLab/source/isaaclab_rl/test/export/leapp_direct_env.py#L18)

---

## Lesson 2: `anymal_c_env.py` — LEAPP アノテーションを足す

### 学ぶこと

- LEAPP はエクスポート時に「アノテーションされたテンソルから出力までの演算」をトレースして ONNX などのグラフにする。通常実行時はほぼ何もしない（dormant）。
- `annotate.input_tensors(node_id, {"name": tensor})`: パイプラインの**入力**。戻り値（トレース用にラップされたテンソル）を**以降の計算で使う**のがポイント。アノテーションしない値（定数など）は定数としてグラフに焼き込まれる。
- `annotate.output_tensors(node_id, {"name": tensor}, export_with="onnx-dynamo")`: パイプラインの**出力**。1 step に 1 回の場所（`_pre_physics_step`）に置く。`_apply_action` は decimation 回呼ばれるので NG。Direct 環境ではエクスポート形式を CLI ではなくここの `export_with` で決める。
- `annotate.state_tensors` / `annotate.update_state`: 前回行動のような**フィードバック状態**。`_get_observations` で `state_tensors` として読み、`_pre_physics_step` で `update_state` して次 step に渡す。**名前（`previous_actions`）を両方で一致させる**。
- `node_id` には `self.spec.id`（gym のタスク ID）を使う。

### 自分で書くステップ

1. Lesson 1 のファイルの先頭に `from leapp import annotate` を追加（isort に並べ替えられないよう元スクリプトは `# isort: skip` を付けている）。
2. `_get_observations` で、7 つの観測ソース（Rough なら `height_data` も）を 1 つずつ `annotate.input_tensors` で包み、戻り値を変数に受ける。
3. `previous_actions` は `annotate.state_tensors(self.spec.id, {"previous_actions": self._actions})` に置き換える。
4. `torch.cat` には**ラップ後の変数**を使うよう書き換える（`self._robot.data...` を直接使うとトレースが切れる）。
5. `_pre_physics_step` の最後に `update_state`（新しい `actions`）と `output_tensors`（`processed_actions`）を追加。
6. オリジナルとの差分を `diff` で取り、元スクリプトと同じ箇所だけが変わっていることを確認する。

### 実行

```bash
uv run --extra isaacsim --extra leapp python my_tutorials/06_deploy/run_my_env.py --num_envs 16
```

### 確認ポイント

- [ ] `--extra leapp` 付きなら Lesson 1 と同じように動く（アノテーションは通常実行に影響しない）
- [ ] `--extra leapp` を外すと `ModuleNotFoundError: leapp` になる理由を説明できる
- [ ] 「output アノテーションを `_apply_action` に置いてはいけない理由」を説明できる
- [ ] `state_tensors` と `update_state` の名前が一致していないと何が困るか説明できる

### 発展課題

- `annotate.input_tensors` を 1 回の呼び出しで複数テンソルまとめて登録する書き方に変えてみる（戻り値の形を確認）
- `default_joint_pos` をアノテーションしなかった場合、エクスポート後のグラフでどう扱われるかを Lesson 3 で比べる（定数として埋め込まれるはず）

### 参照コード

- [06_deploy/anymal_c_env.py: import](../../IsaacLab/scripts/tutorials/06_deploy/anymal_c_env.py#L18) / [出力・状態更新](../../IsaacLab/scripts/tutorials/06_deploy/anymal_c_env.py#L58) / [入力・状態](../../IsaacLab/scripts/tutorials/06_deploy/anymal_c_env.py#L69)
- [exporting_direct_workflow_policies_with_leapp.rst](../../IsaacLab/docs/source/policy_deployment/05_leapp/exporting_direct_workflow_policies_with_leapp.rst)（この章の公式解説。ここを必ず読む）
- LEAPP 本体のドキュメント: https://nvidia-isaac.github.io/leapp/

---

## Lesson 3: `export_my_env.py` — 学習してエクスポートする

### 学ぶこと

- アノテーションは計算を変えないので、**オリジナル環境で学習したチェックポイントをそのまま使える**（観測・行動の並びが同じ）。
- エクスポートの中身: `num_envs=1` で環境を作る → チェックポイントをロード → `leapp.start(...)` してから数 step 回す → `leapp.compile_graph(...)` でグラフ化・検証。
- 成果物: モデル（既定 `.onnx`）、メタデータ `.yaml`、フィードバック状態の初期値 `.safetensors`、グラフの `.png`。
- `--export_method` は Direct 環境では**エラーになる**（`export_with` で指定済みのため）。
- Direct 環境は `isaaclab leapp deploy`（`LeappDeploymentEnv`）には**非対応**。エクスポートまでがこの章の範囲。

### 自分で書くステップ

1. オリジナルのタスクで短く学習してチェックポイントを作る（品質は問わない。100 iter 程度で十分）。
2. `my_tutorials/06_deploy/export_my_env.py` を作る。Lesson 1 と同じ登録差し替えを行い、最後に `from isaaclab_rl.entrypoints.backends.export_rsl_rl import run` して `raise SystemExit(run(sys.argv[1:]))`。引数パースは `run` 側（`--task`, `--checkpoint`, `--export_save_path`, `--validation_steps` など）に任せる。
3. `--checkpoint` を省略すると `logs/rsl_rl/anymal_c_flat_direct/` の最新ランを自動で探す。
4. 成果物の出力先を分かりやすくするため `--export_save_path` を指定する。

### 実行

```bash
# 1) 学習（オリジナル環境。ヘッドレスで短く）
uv run isaaclab train --rl_library rsl_rl --task IsaacContrib-Velocity-Flat-AnymalC-Direct --max_iterations 100
# （任意）挙動を見る
uv run isaaclab play --rl_library rsl_rl --task IsaacContrib-Velocity-Flat-AnymalC-Direct --num_envs 16

# 2) 自作環境でエクスポート
uv run --extra isaacsim --extra leapp python my_tutorials/06_deploy/export_my_env.py \
    --task IsaacContrib-Velocity-Flat-AnymalC-Direct --export_save_path exports/anymal_c_direct
```

### 確認ポイント

- [ ] `logs/rsl_rl/anymal_c_flat_direct/<日時>/model_*.pt` ができている
- [ ] エクスポート時に検証（既定 5 step）がエラーなく終わる
- [ ] `exports/anymal_c_direct/` 以下に `.onnx` / `.yaml` / `.safetensors` / `.png` がある（`ls -R` で確認）
- [ ] `.png` のグラフに 7 つの入力、`processed_actions` 出力、`previous_actions` のフィードバックが見える
- [ ] `.yaml` に入力名・出力名と実行周波数（`1 / (sim.dt * decimation)` = 50 Hz）が書かれている

### 発展課題

- `onnxruntime`（環境に入っている）で `.onnx` を開き、`get_inputs()` / `get_outputs()` の名前と形を表示する
- Lesson 2 で `previous_actions` の `state_tensors` を外して（`self._actions` 直接使用に戻して）エクスポートし、グラフや検証結果がどう変わるか比べる
- Rough 版（`IsaacContrib-Velocity-Rough-AnymalC-Direct`、学習は重い）で `height_data` 入力が増えることを確認する

### 参照コード

- [export_rsl_rl.py: export_rsl_rl_agent](../../IsaacLab/source/isaaclab_rl/isaaclab_rl/entrypoints/backends/export_rsl_rl.py#L130) / [run](../../IsaacLab/source/isaaclab_rl/isaaclab_rl/entrypoints/backends/export_rsl_rl.py#L203)
- [export_common.py: add_common_export_args](../../IsaacLab/source/isaaclab_rl/isaaclab_rl/entrypoints/backends/export_common.py#L55) / [prepare_export_env](../../IsaacLab/source/isaaclab_rl/isaaclab_rl/entrypoints/backends/export_common.py#L139) / [leapp_capture](../../IsaacLab/source/isaaclab_rl/isaaclab_rl/entrypoints/backends/export_common.py#L181)
- [agents/rsl_rl_ppo_cfg.py](../../IsaacLab/source/isaaclab_tasks/isaaclab_tasks/contrib/anymal_c_direct/agents/rsl_rl_ppo_cfg.py)（`experiment_name` = ログフォルダ名）
- [exporting_policies_with_leapp.rst](../../IsaacLab/docs/source/policy_deployment/05_leapp/exporting_policies_with_leapp.rst)（CLI オプション表・検証の手順）

---

## Lesson 4: manager-based 環境のエクスポートとデプロイ（比較）

### 学ぶこと

- manager-based 環境（`ManagerBasedRLEnv`）は Isaac Lab が自動でアノテーション（`patch_env_for_export`）するので、**環境コードを書き換えずに**エクスポートできる。意味情報（semantics）も自動で付く。
- `isaaclab leapp deploy` は `LeappDeploymentEnv` を使い、学習用の manager 群を使わずに「エクスポートしたパイプラインだけ」でロボットを動かす。sim-to-real 前の最終確認に相当。
- 学習・エクスポート・デプロイで**同じタスク・同じ preset（`physics=...`）・同じ Hydra override** を使う必要がある。

### 自分で書くステップ

ここはコードを書かず、コマンドを流して Lesson 3 との違いを観察します。

1. 公開 pretrained がある core タスク（例: `Isaac-Humanoid`）でエクスポートを試す。無ければ短く学習してから `--checkpoint` を省略して実行。
2. 生成された `.yaml` を `--pipeline` に渡してデプロイし、ビューポートで動きを確認。
3. Lesson 3 の `.yaml` と見比べ、manager-based 側にだけある `isaaclab_connection` などのメタデータを探す。

### 実行

```bash
uv run --extra isaacsim --extra leapp isaaclab leapp export --rl_library rsl_rl \
    --task Isaac-Humanoid --checkpoint pretrained physics=isaacsim_physx
uv run --extra isaacsim --extra leapp isaaclab leapp deploy \
    --task Isaac-Humanoid --pipeline <エクスポートされた .yaml> --viz kit physics=isaacsim_physx
```

pretrained が未公開だと「unavailable」と表示されて終了します（その場合は `uv run isaaclab train --rl_library rsl_rl --task Isaac-Humanoid` で学習）。

### 確認ポイント

- [ ] Direct（Lesson 3）と manager-based で「アノテーションを誰が書くか」の違いを説明できる
- [ ] Direct 環境のエクスポート結果を `leapp deploy` に渡せない理由を説明できる
- [ ] デプロイ時に `physics=...` を揃えないと何が起きうるか説明できる

### 参照コード

- [export_annotator.py: patch_env_for_export](../../IsaacLab/source/isaaclab/isaaclab/utils/leapp/export_annotator.py#L737)
- [leapp_deployment_env.py: LeappDeploymentEnv](../../IsaacLab/source/isaaclab/isaaclab/envs/leapp_deployment_env.py#L140)
- [deploying_exported_policies_with_leapp.rst](../../IsaacLab/docs/source/policy_deployment/05_leapp/deploying_exported_policies_with_leapp.rst)
- 読み物として: [policy_deployment/index.rst](../../IsaacLab/docs/source/policy_deployment/index.rst) から辿れる実機デプロイ例（[gear_assembly](../../IsaacLab/docs/source/policy_deployment/02_gear_assembly/gear_assembly_policy.rst), [reach](../../IsaacLab/docs/source/policy_deployment/04_reach/reach_policy.rst), [compass + NuRec](../../IsaacLab/docs/source/policy_deployment/03_compass_with_NuRec/compass_navigation_policy_with_NuRec.rst), [policy_inference_in_usd](../../IsaacLab/docs/source/how-to/policy_inference_in_usd.rst)）

---

## 章末チェック

- [ ] `DirectRLEnv` の各フックが 1 step に何回呼ばれるか説明できる
- [ ] 4 つの LEAPP アノテーションをどのメソッドに置くか、白紙から書ける
- [ ] 登録済みタスクの `entry_point` を自作クラスに差し替えるランナーを書ける
- [ ] 学習 → エクスポート → 成果物（onnx / yaml / safetensors / png）の確認を一人で回せる
- [ ] Direct と manager-based のエクスポート／デプロイ対応の違いを説明できる

次へ → [07_visualizers.md](07_visualizers.md)
