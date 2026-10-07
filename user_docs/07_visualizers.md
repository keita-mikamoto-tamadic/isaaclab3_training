# 07_visualizers — ビジュアライザのストリーミング表示と動画記録

対応スクリプト: [scripts/tutorials/07_visualizers/](../../IsaacLab/scripts/tutorials/07_visualizers/)

## この章のゴール

- 登録済みタスクの cfg を `resolve_task_config` で取り出し、**起動前に**ビジュアライザ設定（`env_cfg.sim.visualizer_cfgs`）を差し込むパターンを書ける
- Kit / Newton GL ビジュアライザの「ストリーミングビュー（環境ごとのカメラをタイル表示）」を設定できる
- `VideoRecorderCfg` で、ビジュアライザ画面やシーンのカメラセンサから mp4 を記録できる

## この章の注意（先に読む）

- 2 本とも「タスクを `gym.make` してランダム行動で回す」だけのスクリプトです。シーンを自分で組むのではなく、**既存タスクの cfg をいじる**のが主題です。
- ストリーミングビューのタイル表示は、このスクリプトでは **Kit と Newton（`newton_gl` / `newton_rtx`）のみ**受け付けます（`rerun` / `viser` はスクリプト側でエラー）。ドキュメント上は Rerun / Viser も対応していますが、このチュートリアルの対象外です。
- `--viz newton` は非推奨の別名で、パース時に `newton_gl` に正規化されます。そのため元スクリプトの `_resolve_task` にある `"newton" in ...` 判定は実質ヒットせず、`--task` 省略時は常に AnymalD（Kit 用）が選ばれます。**`--task` は常に明示**するのが安全です（発展課題で直してみましょう）。
- Galbot（Newton 例）は公式ドキュメントでは `--extra teleop` 付きで実行しています。
- 動画記録の Example 1 / 3 は Kit の RTX 描画を使うので GPU 負荷が高めです。Example 2 はウィンドウなし（センサのオフライン描画）です。

## 進める順番

| # | スクリプト | テーマ | 目安 |
|---|-----------|--------|------|
| 1 | `run_tiled_camera_visualizer.py` | ビジュアライザのストリーミングビュー（Kit / Newton） | 1〜1.5時間 |
| 2 | `run_video_recording.py` | ビジュアライザ／センサから動画を記録（3 例） | 1.5〜2時間 |

自作ファイルは `my_tutorials/07_visualizers/` に置きます。

---

## Lesson 1: `run_tiled_camera_visualizer.py` — ストリーミングビュー

### 学ぶこと

- **タスク駆動スクリプトの骨格**: `add_launcher_args(parser)` → `setup_preset_cli(parser)` が `(args_cli, hydra_args)` を返す → `sys.argv = [sys.argv[0]] + hydra_args`（Hydra が `sys.argv` を読むため）→ `resolve_task_config(task, "")` で env cfg を得る → `with launch_simulation(env_cfg, args_cli):` → `gym.make(task, cfg=env_cfg)`。
- `resolve_task_config` は Kit 起動**前**に呼んでも安全。逆に、ビジュアライザ設定は `launch_simulation` の**前に** cfg に入れておく必要がある（ランチャーが cfg をスキャンし、Kit の `streaming_view=True` を見つけるとカメラ描画を自動で有効化する）。
- `--viz` の値は `args_cli.visualizer`（リスト）に入る。
- `VisualizerCfg` 共通のストリーミング系フィールド:
  - `streaming_view`: タイル表示を有効化
  - `streaming_envs`: 表示する環境数（int でランダム抽出、list で固定）
  - `streaming_sensor_prim_path`: 既存のカメラセンサを使う場合のプリムパス
  - `streaming_cam_eye` + `streaming_cam_target_prim_path`: カメラを自動生成してロボットを追従させる場合
  - `streaming_gt_types`: `("rgb",)` 以外に `depth`, `segmentation`, `normals` など
- cfg のプリムパスは `{ENV_REGEX_NS}` マクロを含むので、`prim_path.format(ENV_REGEX_NS="/World/envs/env_.*")` で正規表現パスにしてから渡す。
- ループ終了条件: `env.unwrapped.sim.visualizers` の全ウィンドウが `is_running()` でない／`is_closed` になったら抜ける。

### 自分で書くステップ

1. parser に `--num_envs`, `--task` を追加 → `add_launcher_args` → `setup_preset_cli` → `sys.argv` 書き換え、までをモジュールレベルに書く。`import isaaclab_tasks` も先頭で（タスク登録のため）。
2. `main()` で `env_cfg, _ = resolve_task_config(args_cli.task, "")`。
3. まず**ストリーミングなし**で動かす: `launch_simulation` 内で `num_envs` / `device` を CLI で上書き → `gym.make` → `env.reset()` → `torch.inference_mode()` 下でランダム行動 `2 * torch.rand(env.action_space.shape, device=...) - 1` を `env.step`。
4. Kit 用の関数を作る: `KitVisualizerCfg()`（`from isaaclab_visualizers.kit import ...`、関数内 import）に `streaming_view=True`, `streaming_envs=36`。`env_cfg.scene` に `ego_cam` があればそのプリムパス、無ければ `streaming_cam_eye=(3,3,3)` と `streaming_cam_target_prim_path="/World/envs/*/Robot/base"`。
5. Newton 用も同様に `NewtonGLVisualizerCfg()`（`from isaaclab_visualizers.newton import ...`）で作る。
6. `args_cli.visualizer` を見て cfg のリストを作り、`env_cfg.sim.visualizer_cfgs = [...]` に代入。**`launch_simulation` の前**に呼ぶ。
7. ループを「ウィンドウが全部閉じたら終了」に変える。`sim.visualizers` が空なら警告して終了。

### 実行

```bash
# Kit: AnymalD を追従する自動生成カメラ
uv run python my_tutorials/07_visualizers/run_tiled_camera_visualizer.py \
    --task Isaac-Velocity-Rough-AnymalD --num_envs 256 --viz kit

# Newton GL: Galbot の頭部カメラ（ego_cam）をストリーミング
uv run --extra teleop python my_tutorials/07_visualizers/run_tiled_camera_visualizer.py \
    --task IsaacContrib-Stack-Cube-Galbot-Left-Arm-Gripper-Visuomotor --num_envs 25 --viz newton_gl
```

### 確認ポイント

- [ ] Kit で Viewport タブに **Streaming View** が増え、36 個のタイルにロボットが映る
- [ ] Newton GL で HUD サイドバーの **Streaming View** から Hide / Open を切り替えられる
- [ ] ウィンドウを閉じるとスクリプトが終了する
- [ ] 「ビジュアライザ cfg を `launch_simulation` の前に入れる理由」を説明できる

### 発展課題

- `streaming_cam_eye=(0.0, 0.0, 5.0)` で真上からのビューにする
- `streaming_gt_types=("rgb", "depth")` にして深度も並べる（深度の色範囲は `streaming_depth_min` / `streaming_depth_max`）
- `streaming_envs=[0, 1, 2, 3]` で表示する環境を固定する
- `_resolve_task` を `any(v.startswith("newton") for v in ...)` のように直し、`--viz newton_gl` だけで Galbot が選ばれるようにする
- Newton 例で `streaming_sensor_prim_path` を左手首カメラ（スクリプト内コメント参照）に切り替える

### 参照コード

- [preset_cli.py: setup_preset_cli](../../IsaacLab/source/isaaclab_tasks/isaaclab_tasks/utils/preset_cli.py#L62)
- [hydra.py: resolve_task_config](../../IsaacLab/source/isaaclab_tasks/isaaclab_tasks/utils/hydra.py#L440)
- [visualizer_cfg.py: VisualizerCfg（streaming_* フィールド）](../../IsaacLab/source/isaaclab/isaaclab/visualizers/visualizer_cfg.py#L41) / [streaming_view](../../IsaacLab/source/isaaclab/isaaclab/visualizers/visualizer_cfg.py#L79)
- [kit_visualizer_cfg.py: KitVisualizerCfg](../../IsaacLab/source/isaaclab_visualizers/isaaclab_visualizers/kit/kit_visualizer_cfg.py#L23)
- [newton_visualizer_cfg.py: NewtonGLVisualizerCfg](../../IsaacLab/source/isaaclab_visualizers/isaaclab_visualizers/newton/newton_visualizer_cfg.py#L123)
- [simulation_cfg.py: visualizer_cfgs](../../IsaacLab/source/isaaclab/isaaclab/sim/simulation_cfg.py#L140) / [simulation_context.py: visualizers](../../IsaacLab/source/isaaclab/isaaclab/sim/simulation_context.py#L644)
- [base_visualizer.py: is_running](../../IsaacLab/source/isaaclab/isaaclab/visualizers/base_visualizer.py#L103) / [is_closed](../../IsaacLab/source/isaaclab/isaaclab/visualizers/base_visualizer.py#L151)
- [sim_launcher.py: streaming_view でカメラを自動有効化](../../IsaacLab/source/isaaclab/isaaclab/app/sim_launcher.py#L277)
- ドキュメント: [visualizer_tiled_camera.rst](../../IsaacLab/docs/source/features/visualizer_tiled_camera.rst)（トラブルシュート、Newton MJWarp + `replicate_physics=True` で自動生成カメラが失敗する件も記載）、[concepts/visualization.rst](../../IsaacLab/docs/source/concepts/visualization.rst)

---

## Lesson 2: `run_video_recording.py` — 動画を記録する

### 学ぶこと

- `VideoRecorderCfg` を `env_cfg.video_recorders`（リスト）に入れるだけで、環境が毎 step フレームを取り込み mp4 を書く。複数入れれば独立した複数ストリームになる。
- `source` 文字列で取り込み元を選ぶ:
  - `"visualizer:kit"` / `"visualizer:newton"`: ビジュアライザの画面
  - `"visualizer:kit:streaming_view"` / `"visualizer:newton:streaming_view"`: ストリーミングパネル（`streaming_view=True` 必須）
  - `"sensor:<name>"`（`:rgb` / `:depth` / `:segmentation` / `:normals`）: `env.scene.sensors[name]` から直接。ビジュアライザ不要
- カメラ位置や解像度は**ビジュアライザ cfg 側**（`eye`, `lookat`, `window_width`, `focal_length`）で決める。レコーダ側ではない。
- クリップ制御: `video_length`（step 数）、`video_interval`（0 = 1 本だけ）、`fps`、`step_offset`（RTX のウォームアップを飛ばす）、`output_filename_prefix`。
- クリップは `env.close()` 時にフラッシュされる。`_NUM_STEPS` を `video_length` より少し多くしているのはこのため。
- `resolve_task_config(..., overrides=(..., "env.scene.tiled_camera=rgb"))` のように、Hydra override をコードから渡して **PresetCfg の選択肢**（ここではカメラの data_types = rgb）を選べる。センサから記録するには `"rgb"` が必要。
- Kit 画面の記録（Example 1 / 3）は `args_cli.enable_cameras = True` を `launch_simulation` 前に立てる必要がある。

### 自分で書くステップ

1. Lesson 1 の骨格をコピーし、`--example`（1/2/3）と `--num_envs` を追加。タスクは `Isaac-Reorient-Cube-Shadow-Camera-Direct` 固定。
2. 共通の `_shadow_env_cfg(num_envs, env_spacing)` を書く: override で `tiled_camera=rgb` を選び、カメラを 256x256 にし、`num_envs` / `env_spacing` を設定。
3. **Example 1**: `KitVisualizerCfg(eye=..., lookat=...)` を `visualizer_cfgs` に、`VideoRecorderCfg(source="visualizer:kit", output_dir=..., video_length=100, fps=30, step_offset=5)` を `video_recorders` に。`enable_cameras` を立てる。→ 実行して mp4 を確認。
4. **Example 2**: `visualizer_cfgs = []`、`source="sensor:tiled_camera"`。→ ウィンドウが開かずに mp4 ができることを確認。
5. **Example 3**: Kit（`streaming_view=True`, `streaming_envs=min(num_envs, 16)`）と `NewtonGLVisualizerCfg(window_width=1280, window_height=720, ...)` を両方入れ、レコーダを 4 本（kit / kit:streaming_view / newton / sensor）並べる。
6. `--example` の値で builder 関数を引く辞書を作り、`main()` で `gym.make` → `_NUM_STEPS` 回ランダム行動 → **`env.close()`** を忘れずに。

### 実行

```bash
uv run python my_tutorials/07_visualizers/run_video_recording.py --example 1 --num_envs 4
uv run python my_tutorials/07_visualizers/run_video_recording.py --example 2 --num_envs 16
uv run python my_tutorials/07_visualizers/run_video_recording.py --example 3 --num_envs 4
```

### 確認ポイント

- [ ] `videos/recording_tutorial/example_1/kit_viewport_0000.mp4` ができ、2x2 の Shadow Hand が映っている
- [ ] Example 2 ではウィンドウが開かず、`example_2/sensor_0000.mp4` にセンサ視点のタイル映像が入っている
- [ ] Example 3 で `kit_viewport` / `tiled_kit_viewport` / `newton_viewport` / `sensor` の 4 本が同時にできる
- [ ] `env.close()` を消すと動画がどうなるか確認し、理由を説明できる

### 発展課題

- `video_interval=50` と `keep_last_n_clips=1` を試し、クリップの増え方と削除を観察する
- `source="sensor:tiled_camera:depth"` で深度を記録する（`tiled_camera=depth` 等、depth を含む preset が必要）
- 学習コマンドの `--video` フラグ（`uv run isaaclab train --rl_library rsl_rl --task Isaac-Cartpole --viz kit --video`）で、学習中の自動記録も試す
- 06 章のエクスポートで使ったタスクの `play` に記録を付け、学習結果を動画で残す

### 参照コード

- [video_recorder_cfg.py: VideoRecorderCfg](../../IsaacLab/source/isaaclab/isaaclab/envs/utils/video_recorder_cfg.py#L14)
- [video_recorder.py: VideoRecorder](../../IsaacLab/source/isaaclab/isaaclab/envs/utils/video_recorder.py#L53)
- [direct_rl_env.py: レコーダ生成](../../IsaacLab/source/isaaclab/isaaclab/envs/direct_rl_env.py#L164) / [毎 step の取り込み](../../IsaacLab/source/isaaclab/isaaclab/envs/direct_rl_env.py#L463) / [close 時のフラッシュ](../../IsaacLab/source/isaaclab/isaaclab/envs/direct_rl_env.py#L564)
- [direct_rl_env_cfg.py: video_recorders](../../IsaacLab/source/isaaclab/isaaclab/envs/direct_rl_env_cfg.py#L272)
- [shadow_hand_direct_camera_env_cfg.py: ShadowHandTiledCameraCfg（PresetCfg）](../../IsaacLab/source/isaaclab_tasks/isaaclab_tasks/core/reorient/config/shadow_hand/shadow_hand_direct_camera_env_cfg.py#L91)
- ドキュメント: [record_video.rst](../../IsaacLab/docs/source/features/record_video.rst)（source 一覧・クリップ制御・制限事項の表）、[capture_sensor_frames.rst](../../IsaacLab/docs/source/how-to/capture_sensor_frames.rst)

---

## 章末チェック

- [ ] `setup_preset_cli` → `resolve_task_config` → `launch_simulation` → `gym.make` の流れを白紙から書ける
- [ ] 「自動生成カメラで追従」と「既存カメラセンサを流用」のストリーミング設定の違いを説明できる
- [ ] `source` 文字列を見て、何が記録されるか・ビジュアライザが必要かを答えられる
- [ ] カメラ位置をレコーダではなくビジュアライザ cfg で決める理由を説明できる

## この後のおすすめ

チュートリアルはここで終わりです。次は「1 つのタスクを最初から最後まで作る」流れに進みましょう。

- **SO-101 エンドツーエンドチュートリアル**: [docs/source/setup/tutorial.rst](../../IsaacLab/docs/source/setup/tutorial.rst)
  SO-101 アームでバイアルを掴んでラックに置くタスクを、外部プロジェクト（IsaacLabTutorial）として構築 → 検証 → 状態ポリシー学習 → 手首カメラポリシーへの蒸留まで通します。これまでの章（アセット・シーン・manager-based 環境・センサ・ビジュアライザ）の総復習になります。
- **強化学習スクリプト**: [scripts/reinforcement_learning/](../../IsaacLab/scripts/reinforcement_learning/)
  - [train.py](../../IsaacLab/scripts/reinforcement_learning/train.py) / [play.py](../../IsaacLab/scripts/reinforcement_learning/play.py): `uv run isaaclab train` / `play` と同じ統合エントリポイント（中身は [dispatch.py: run_train_cli](../../IsaacLab/source/isaaclab_rl/isaaclab_rl/entrypoints/dispatch.py#L46)）。`--rl_library`（rsl_rl / rl_games / skrl / sb3 など）を切り替えて同じタスクを学習してみる
  - [train_multigpu.py](../../IsaacLab/scripts/reinforcement_learning/train_multigpu.py): 複数 GPU 学習
  - [ray/](../../IsaacLab/scripts/reinforcement_learning/ray/): Ray を使ったハイパーパラメータ探索
  - 使い方は [quickstart.rst](../../IsaacLab/docs/source/setup/quickstart.rst) と [concepts/reinforcement_learning.rst](../../IsaacLab/docs/source/concepts/reinforcement_learning.rst) を参照
