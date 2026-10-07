# 05_controllers — タスク空間コントローラ（微分 IK / OSC）

対応スクリプト: [scripts/tutorials/05_controllers/](../../IsaacLab/scripts/tutorials/05_controllers/)

## この章のゴール

- 「手先をここに置きたい」という**タスク空間の指令**を、関節の指令（位置 or トルク）に変換する流れを説明できる
- `DifferentialIKController` / `OperationalSpaceController` が「**計算だけ**するクラス」であり、ヤコビアン・質量行列・手先姿勢などを**自分で正しい座標系で渡す**必要があることを理解する
- フレーム表記 `_w`（ワールド）/ `_b`（ロボットのベース）/ `_task`（タスク座標系）を意識して変換を書ける

## 進める順番

| # | スクリプト | テーマ | 目安 |
|---|-----------|--------|------|
| 0 | （予習） | ヤコビアン・微分 IK・OSC の数式 | 30分〜1時間 |
| 1 | `run_diff_ik.py` | 微分 IK で手先姿勢を追従（関節位置指令） | 1.5〜2時間 |
| 2 | `run_osc.py` | OSC で傾いた壁を一定力で押しつつ姿勢追従（関節トルク指令） | 3〜4時間 |

自作ファイルは `my_tutorials/05_controllers/` に置きます。どちらも Isaac Sim PhysX 前提のチュートリアルです（ヤコビアン・質量行列を物理エンジンから取得するため）。

---

## Lesson 0: 予習 — 必要な数式だけ

コードを書き直すのに必要な範囲に絞ります。詳しくは [motion_generators.rst](../../IsaacLab/docs/source/concepts/motion_generators.rst) を参照。

### ヤコビアン

- 関節速度 `q̇`（n 次元）と手先の速度（並進 3 + 角速度 3 = 6 次元）の線形関係: `ẋ = J(q) q̇`、`J` は 6×n。
- Isaac Lab の `body_link_jacobian_w` は行 `0:3` が並進、`3:6` が回転。**ワールド座標系**で表現されている。
- 形状は `(num_envs, num_jacobi_bodies, 6, num_joints + num_base_dofs)`。インデックスのずらし方が 2 つある（後述の「Isaac Lab 3.0 の注意点」）。

### 微分 IK（Differential IK）

- 姿勢誤差 `Δx = (Δp, Δφ)`: 位置は差、姿勢は `q_des * q_cur⁻¹` を軸角（axis-angle, 3 次元）にしたもの（`compute_pose_error`）。
- `Δq = J⁻ Δx`、`q_des = q + Δq`。この `q_des` を**関節位置目標**として PD アクチュエータに渡す。
- `J⁻`（擬似逆行列）の計算法 = `ik_method`:
  - `"pinv"`: Moore-Penrose `Jᵀ(JJᵀ)⁻¹`。特異姿勢付近で発散しやすい
  - `"dls"`: 減衰最小二乗 `Jᵀ(JJᵀ + λ²I)⁻¹`。特異点で安定（チュートリアルはこれ、`lambda_val` 既定 0.01）
  - `"trans"`: `Jᵀ` をそのまま使う（勾配法的、遅いが安定）
  - `"svd"`: 小さな特異値を 0 に切る擬似逆
  - `"adaptive_dls"`: 特異値が小さいほど λ を大きくする DLS

### オペレーショナルスペース制御（OSC）

ロボットの運動方程式 `M(q) q̈ + C(q, q̇) q̇ + g(q) = τ` を使い、**トルク**を直接出す。

1. **インピーダンス（バネ・ダンパ）**: 手先の目標加速度 `ẍ_des = Kp·e − Kd·ẋ`（目標速度 0）。`Kd = 2√Kp · ζ`（ζ=1 で臨界減衰）。
2. **慣性デカップリング**: タスク空間の慣性 `Λ = (J M⁻¹ Jᵀ)⁻¹` を使い `F = Λ ẍ_des`。これで各軸が独立した質点のように振る舞う。オフなら `F = ẍ_des`（単純なタスク空間インピーダンス）。`partial_...=True` は並進と回転の結合を無視（3×3 ブロック 2 つに分ける）。
3. **関節トルクへ**: `τ = Jᵀ S_m F`。`S_m` は運動制御する軸の選択行列（対角 0/1）。
4. **力制御**: 力制御軸 `S_f` には `F_c = F_des + K_f (F_des − F_meas)`（`contact_wrench_stiffness_task` が `None` なら開ループで `F_des` のみ）。`τ += Jᵀ S_f F_c`。
5. **ヌル空間制御**: 7 自由度の Franka は手先 6 自由度に対して 1 自由度余る。`q̈_null = kp(q_target − q) − kd q̇` を、手先に影響しない空間に射影して足す（動的整合な射影 `(I − Jᵀ Λ J M⁻¹)`）。
6. **重力補償**: `gravity_compensation=True` なら `τ += g(q)`。

### タスク座標系（task frame）

- 「壁に垂直に押す」は、壁に平行な座標系で書けば `z` 軸だけの話になる。目標・ゲイン・選択行列をタスク座標系で書き、コントローラ内部で `R S Rᵀ` のようにベース座標系へ回転させる。

### 確認ポイント

- [ ] `J` の各次元（6×n）の意味と、行 0:3 / 3:6 の違いを説明できる
- [ ] 微分 IK の出力が「関節位置」、OSC の出力が「関節トルク」であることを説明できる
- [ ] `Kd = 2√Kp · ζ` の ζ=1 が何を意味するか説明できる

---

## Lesson 1: `run_diff_ik.py` — 微分 IK

### 学ぶこと

- `DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls")` → `DifferentialIKController(cfg, num_envs, device)`。
- `action_dim`: `"position"`→3、`"pose"`+相対→6、`"pose"`+絶対→7（`x, y, z, qx, qy, qz, qw`）。
- **指令と計算の分離**: `set_command(command)` は目標を保存するだけ、`compute(ee_pos, ee_quat, jacobian, joint_pos)` が毎ステップ `q_des` を返す。
- `SceneEntityCfg("robot", joint_names=[...], body_names=[...])` + `.resolve(scene)` で `joint_ids` / `body_ids` を得る（`find_joints` / `find_bodies` を名前チェック付きで呼ぶ）。
- ワールド → ベースの変換: `subtract_frame_transforms(root_pos, root_quat, ee_pos, ee_quat)`（`T_root⁻¹ · T_ee`）。
- `VisualizationMarkers` + `FRAME_MARKER_CFG` で現在と目標の座標軸を表示。

### 自分で書くステップ

1. **骨格**: 02/03 章と同じく `argparse` + `add_launcher_args` → パース後に import。独自引数 `--robot`（`franka_panda` / `ur10`）と `--num_envs`（既定 128）。
2. **シーン**: `@configclass` で `InteractiveSceneCfg` を継承し、地面（`pos=(0,0,-1.05)`）、`DomeLightCfg`、台（`UsdFileCfg` で `{ISAAC_NUCLEUS_DIR}/Props/Mounts/Stand/stand_instanceable.usd`, `scale=(2,2,2)`）、ロボットを定義。ロボットは `replace(FRANKA_PANDA_HIGH_PD_CFG, prim_path="{ENV_REGEX_NS}/Robot")`（PD ゲインが高く、重力オフの Franka）。
3. `main()` で `SimulationCfg(dt=0.01)` → `launch_simulation` → `instantiate(scene_cfg)` → `sim.reset()` まで書き、まずロボットが並ぶことを確認。
4. **コントローラ生成**: `run_simulator` 内で cfg と controller を作る。
5. **マーカー**: `clone(FRAME_MARKER_CFG)` → `.markers["frame"].scale = (0.1,0.1,0.1)` → `replace(..., prim_path="/Visuals/ee_current")` と `"/Visuals/ee_goal"` の 2 つ。
6. **目標リスト**: `(3, 7)` のテンソルで 3 つのゴール（ベース座標系、**xyzw**）。`ik_commands = torch.zeros(num_envs, controller.action_dim)`。
7. **インデックス解決**: Franka なら `joint_names=["panda_joint.*"]`, `body_names=["panda_hand"]`。UR10 なら `[".*"]`, `["ee_link"]`。`resolve(scene)` 後、
   - `ee_jacobi_idx = body_ids[0] - 1`（固定ベースの場合。`robot.is_fixed_base` で分岐）
   - `jacobi_joint_ids = [j + robot.num_base_dofs for j in joint_ids]`
8. **ループ（リセット側）**: `count % 150 == 0` で関節をデフォルトに戻す（`write_joint_position_to_sim_index` / `write_joint_velocity_to_sim_index` → `robot.reset()`）、`ik_commands[:] = ee_goals[idx]`、`controller.reset()` → `set_command(ik_commands)`、ゴールを次へ。`joint_pos_des` はデフォルト関節角で初期化しておく。
9. **ループ（通常側）**: 
   - `jacobian = robot.data.body_link_jacobian_w.torch[:, ee_jacobi_idx, :, jacobi_joint_ids]`
   - `ee_pose_w = robot.data.body_pose_w.torch[:, body_id]`、`root_pose_w = robot.data.root_pose_w.torch`
   - `subtract_frame_transforms` でベース座標系の `ee_pos_b, ee_quat_b` を得る
   - `joint_pos_des = controller.compute(ee_pos_b, ee_quat_b, jacobian, joint_pos)`
10. **適用**: `robot.set_joint_position_target_index(target=joint_pos_des, joint_ids=joint_ids)` → `scene.write_data_to_sim()` → `sim.step()` → `scene.update(dt)`。
11. **マーカー更新**: 現在は `body_state_w[..., 0:7]`、目標は `ik_commands[:, 0:3] + scene.env_origins`（目標はベース座標系なのでワールドに戻す）。

```python
DifferentialIKController.compute(ee_pos, ee_quat, jacobian, joint_pos) -> joint_pos_des  # (N, num_joints)
```

### 実行

```bash
uv run python my_tutorials/05_controllers/run_diff_ik.py --viz kit
uv run python my_tutorials/05_controllers/run_diff_ik.py --robot ur10 --num_envs 16 --viz kit
```

（`--viz kit` は公式ドキュメントの実行例に合わせた GUI 表示指定）

### 確認ポイント

- [ ] 手先の座標軸マーカーが目標マーカーに収束し、150 ステップごとに次の目標へ切り替わる
- [ ] `--robot ur10` でも動く（関節名・手先リンク名だけが違う）
- [ ] `ee_jacobi_idx` を `-1` しないと手先が明らかにおかしな動きをする（試して理由を説明できる）
- [ ] 目標マーカーに `env_origins` を足さないと、全部原点付近の環境に重なることを確認した

### 発展課題

- `ik_method` を `"pinv"` / `"trans"` / `"svd"` / `"adaptive_dls"` に変えて収束速度・安定性を比較する（`ik_params` で `k_val` や `lambda_val` も変えてみる）
- `command_type="position"` にする（`set_command` に `ee_quat` が必須になる点に注意）
- `use_relative_mode=True` にして、毎ステップ小さな `(dx, dy, dz, droll, dpitch, dyaw)` を与えて円を描かせる
- ゴールを手の届かない位置にすると DLS と pinv でどう違うか観察する

### 参照コード

- [differential_ik.py: DifferentialIKController](../../IsaacLab/source/isaaclab/isaaclab/controllers/differential_ik.py#L18)
- [differential_ik.py: set_command](../../IsaacLab/source/isaaclab/isaaclab/controllers/differential_ik.py#L119)
- [differential_ik.py: compute](../../IsaacLab/source/isaaclab/isaaclab/controllers/differential_ik.py#L192)
- [differential_ik.py: _compute_delta_joint_pos（各 ik_method の式）](../../IsaacLab/source/isaaclab/isaaclab/controllers/differential_ik.py#L223)
- [differential_ik_cfg.py: DifferentialIKControllerCfg](../../IsaacLab/source/isaaclab/isaaclab/controllers/differential_ik_cfg.py#L18)
- [scene_entity_cfg.py: SceneEntityCfg.resolve](../../IsaacLab/source/isaaclab/isaaclab/managers/scene_entity_cfg.py#L126)
- [base_articulation_data.py: body_link_jacobian_w（インデックス規約）](../../IsaacLab/source/isaaclab/isaaclab/assets/articulation/base_articulation_data.py#L883)
- [base_articulation.py: num_base_dofs](../../IsaacLab/source/isaaclab/isaaclab/assets/articulation/base_articulation.py#L448)
- [math.py: subtract_frame_transforms](../../IsaacLab/source/isaaclab/isaaclab/utils/math.py#L857)
- [math.py: compute_pose_error](../../IsaacLab/source/isaaclab/isaaclab/utils/math.py#L891)
- [markers/config: FRAME_MARKER_CFG](../../IsaacLab/source/isaaclab/isaaclab/markers/config/__init__.py#L64)
- [franka.py: FRANKA_PANDA_HIGH_PD_CFG](../../IsaacLab/source/isaaclab_assets/isaaclab_assets/robots/franka.py#L106)
- ドキュメント: [how-to/run_diff_ik.rst](../../IsaacLab/docs/source/how-to/run_diff_ik.rst)

---

## Lesson 2: `run_osc.py` — オペレーショナルスペース制御

傾いた壁に対し、壁の法線方向（タスク座標系 z）には **10 N の力**、それ以外の 5 軸には**姿勢追従**を行う「ハイブリッド力/運動制御」です。長いので A〜F に分けて、途中で動作確認しながら進めます。

### 学ぶこと

- `OperationalSpaceControllerCfg` の主要パラメータ: `target_types`, `impedance_mode`, `inertial_dynamics_decoupling`, `partial_inertial_dynamics_decoupling`, `gravity_compensation`, `motion_damping_ratio_task`, `contact_wrench_stiffness_task`, `motion_control_axes_task`, `contact_wrench_control_axes_task`, `nullspace_control`。
- `action_dim = target_dim (+6 if variable_kp) (+12 if variable)`。チュートリアルは `pose_abs`(7) + `wrench_abs`(6) + kp(6) = **19**。
- `set_command(command, current_ee_pose_b, current_task_frame_pose_b)` と `compute(jacobian_b, current_ee_pose_b, current_ee_vel_b, current_ee_force_b, mass_matrix, gravity, current_joint_pos, current_joint_vel, nullspace_joint_pos_target)`。
- 関節**トルク**制御するには、アクチュエータの PD ゲインを 0 にして `set_joint_effort_target_index` を使う。
- 接触センサ（04 章）の値を力フィードバックに使う。

### 自分で書くステップ

#### A. シーン（30分）

1. 骨格は Lesson 1 と同じ（`--num_envs` のみ）。
2. `SceneCfg` に地面・ドームライトを置く。
3. **傾いた壁**: `CuboidCfg(size=(2.0, 1.5, 0.01))` に `collision_props`、半透明の `visual_material`（`opacity=0.1`）、`rigid_props=UsdPhysicsRigidBodyCfg(kinematic_enabled=True)`（動かない剛体）、`activate_contact_sensors=True`。`init_state` は `pos=(0.685, 0, 0.3)`, `rot=(0.0, -0.3826834324, 0.0, 0.9238795325)`（xyzw、y 軸まわり −45°）。
4. `ContactSensorCfg(prim_path="/World/envs/env_.*/TiltedWall", update_period=0.0, history_length=2)`。
5. ロボット: `replace(FRANKA_PANDA_HIGH_PD_CFG, ...)` の後、`actuators["panda_shoulder"]` と `["panda_forearm"]` の `stiffness` / `damping` を **0.0** に。`spawn.rigid_props.disable_gravity = True`。
6. ここで一度実行し、壁とロボットが並ぶことを確認（腕は PD も重力も無いので、トルクを与えるまでほぼその場に留まる）。

#### B. コントローラとターゲット（30分）

1. インデックス: `ee_frame_idx = robot.find_bodies("panda_leftfinger")[0][0]`、`arm_joint_ids = robot.find_joints(["panda_joint.*"])[0]`（今回は `SceneEntityCfg` を使わず直接）。
2. `OperationalSpaceControllerCfg(...)` を作る。値はチュートリアルどおり: `target_types=["pose_abs", "wrench_abs"]`, `impedance_mode="variable_kp"`, `inertial_dynamics_decoupling=True`, `partial_...=False`, `gravity_compensation=False`, `motion_damping_ratio_task=1.0`, `contact_wrench_stiffness_task=[0,0,0.1,0,0,0]`, `motion_control_axes_task=[1,1,0,1,1,1]`, `contact_wrench_control_axes_task=[0,0,1,0,0,0]`, `nullspace_control="position"`。
3. 運動軸と力軸が**相補的**（足すと全部 1）になっていることを確認。
4. ターゲット 3 種を作って `torch.cat(..., dim=-1)` で `(3, 19)` の `ee_target_set` に:
   - 姿勢（ベース座標系, xyzw）: `(3, 7)`
   - レンチ（**タスク座標系**）: `[0, 0, 10, 0, 0, 0]` × 3
   - kp: 各行 6 要素（360 / 420 / 320）
5. マーカーは Lesson 1 と同じ。

#### C. `update_states()` — ロボット状態をベース座標系で集める（1時間）

戻り値 10 個: `jacobian_b, mass_matrix, gravity, ee_pose_b, ee_vel_b, root_pose_w, ee_pose_w, ee_force_b, joint_pos, joint_vel`。

1. `jacobi_joint_ids = [j + robot.num_base_dofs for j in arm_joint_ids]`、`ee_jacobi_idx = ee_frame_idx - 1`。
2. `jacobian_w = body_link_jacobian_w.torch[:, ee_jacobi_idx, :, jacobi_joint_ids]`。
3. `mass_matrix = data.mass_matrix.torch[:, ids, :][:, :, ids]`（行と列の両方を絞る。一度にファンシーインデックスすると形が変わる点に注意）。`gravity = data.gravity_compensation_forces.torch[:, ids]`。
4. **ヤコビアンをワールド → ベースへ**: `R = matrix_from_quat(quat_inv(root_quat_w))` を作り、並進行 `[:, :3, :]` と回転行 `[:, 3:, :]` それぞれに `torch.bmm(R, ...)`。
5. 手先姿勢: `body_pos_w` / `body_quat_w` → `subtract_frame_transforms` で `ee_pose_b`。`root_pose_w`, `ee_pose_w` も `torch.cat` で 7 次元に。
6. 手先速度: `body_vel_w[:, ee_frame_idx] - root_vel_w` を `quat_apply_inverse(root_quat_w, ...)` で並進・角速度それぞれベースへ。
7. 接触力: `contact_forces.update(dt)` → `net_normal_forces_w_history`（形 `(N, history, num_sensors, 3)`）を履歴方向に `mean`、センサ方向に `max` して `(N, 3)`。チュートリアルは簡略化のため `ee_force_b = ee_force_w`（ベースが回転していないので成立）。
8. `joint_pos` / `joint_vel` を `arm_joint_ids` で切り出す。

#### D. `update_target()` と `convert_to_task_frame()`（45分）

1. `update_target`: `command[:] = ee_target_set[idx]`、`target_types` をループして `pose_abs` なら `command[:, :7]` を `ee_target_pose_b` に。マーカー用に `combine_frame_transforms(root_pos, root_quat, ...)` でワールドへ。次の `idx` も返す。
2. `convert_to_task_frame`: **タスク座標系 = 目標姿勢そのもの**（壁と平行）とする。`task_frame_pose_b = ee_target_pose_b.clone()`。`pose_abs` 部分を `subtract_frame_transforms(task_frame, command_pose)` でタスク座標系に変換（結果は単位姿勢になる）。`wrench_abs` は最初からタスク座標系で定義済みなので変換しない。
3. `cmd_idx` を 7 / 6 ずつ進めるループにして、`target_types` の順に並んでいることを意識する。

#### E. メインループ（45分）

1. ループ前: `robot.update(dt=sim_dt)`（**最初のステップ前にバッファ更新が必要**）、`joint_centers = soft_joint_pos_limits.torch[:, arm_joint_ids, :].mean(dim=-1)`（ヌル空間の目標＝関節可動域の中央）、`update_states` を 1 回呼ぶ。
2. **リセット（500 ステップごと）**: 関節をデフォルトへ書き込み、`set_joint_effort_target_index(target=zeros)` → `robot.write_data_to_sim()` → `robot.reset()` → `contact_forces.reset()` → `robot.update(dt)` → `update_states` で `ee_pose_b` だけ取り直す → `update_target` → `osc.reset()` → `convert_to_task_frame` → `osc.set_command(command=..., current_ee_pose_b=..., current_task_frame_pose_b=...)`。
3. **通常ステップ**: `update_states` → `osc.compute(...)`（9 個のキーワード引数、`nullspace_joint_pos_target=joint_centers`）→ `robot.set_joint_effort_target_index(target=joint_efforts, joint_ids=arm_joint_ids)` → `robot.write_data_to_sim()`。
4. 共通: マーカー更新 → `sim.step(render=True)` → `robot.update(dt)` → `scene.update(dt)` → `count += 1`。

#### F. 段階的に試す（30分〜）

いきなり全部動かさず、cfg を変えて段階的に確認すると理解が深まります。

1. `target_types=["pose_abs"]`、`impedance_mode="fixed"`、`nullspace_control="none"`、`inertial_dynamics_decoupling=False` の最小構成（`action_dim=7`、ターゲットも姿勢だけに）で姿勢追従だけ。
2. `inertial_dynamics_decoupling=True` にして動きの違いを見る。
3. `nullspace_control="position"` を追加。
4. 最後に `wrench_abs` と選択軸、`variable_kp` を追加してチュートリアルと同じ構成へ。

```python
osc.set_command(command, current_ee_pose_b=None, current_task_frame_pose_b=None)
osc.compute(jacobian_b, current_ee_pose_b, current_ee_vel_b, current_ee_force_b,
            mass_matrix, gravity, current_joint_pos, current_joint_vel, nullspace_joint_pos_target) -> joint_efforts
```

### 実行

```bash
uv run python my_tutorials/05_controllers/run_osc.py --viz kit
uv run python my_tutorials/05_controllers/run_osc.py --num_envs 16 --viz kit
```

### 確認ポイント

- [ ] 手先（左指）が壁に接触し、壁に沿った方向は目標マーカーに追従する
- [ ] 500 ステップごとにリセットされ、3 つの目標（kp も異なる）を順に巡る
- [ ] `osc.action_dim` を print して 19 になることを確認した
- [ ] アクチュエータの stiffness/damping を 0 にしないと何が起きるか試し、理由を説明できる
- [ ] `motion_control_axes_task` と `contact_wrench_control_axes_task` が相補的である理由を説明できる

### 発展課題

- 目標レンチの z を 5 N / 20 N に変え、`contact_forces` の値を print して実測と比較する（`contact_wrench_stiffness_task=None` の開ループと閉ループの差も見る）
- `partial_inertial_dynamics_decoupling=True` に変えて差を見る
- `impedance_mode="variable"` にして damping ratio も指令に含める（`action_dim=25`、`motion_damping_ratio_limits_task` に注意）
- `target_types=["pose_rel", "wrench_abs"]` にして相対指令で壁をなぞらせる
- `gravity_compensation=True` かつロボットの `disable_gravity=False` にして、`gravity` を渡す構成を試す

### 参照コード

- [operational_space.py: OperationalSpaceController](../../IsaacLab/source/isaaclab/isaaclab/controllers/operational_space.py#L65)
- [operational_space.py: action_dim](../../IsaacLab/source/isaaclab/isaaclab/controllers/operational_space.py#L198)
- [operational_space.py: set_command（タスク→ベース座標変換）](../../IsaacLab/source/isaaclab/isaaclab/controllers/operational_space.py#L224)
- [operational_space.py: compute](../../IsaacLab/source/isaaclab/isaaclab/controllers/operational_space.py#L387)
- [operational_space.py: 慣性デカップリング部分](../../IsaacLab/source/isaaclab/isaaclab/controllers/operational_space.py#L491)
- [operational_space.py: ヌル空間トルク](../../IsaacLab/source/isaaclab/isaaclab/controllers/operational_space.py#L557)
- [operational_space.py: _critically_damped_gains](../../IsaacLab/source/isaaclab/isaaclab/controllers/operational_space.py#L582)
- [operational_space_cfg.py: OperationalSpaceControllerCfg](../../IsaacLab/source/isaaclab/isaaclab/controllers/operational_space_cfg.py#L19)
- [base_articulation_data.py: mass_matrix](../../IsaacLab/source/isaaclab/isaaclab/assets/articulation/base_articulation_data.py#L929) / [gravity_compensation_forces](../../IsaacLab/source/isaaclab/isaaclab/assets/articulation/base_articulation_data.py#L943) / [soft_joint_pos_limits](../../IsaacLab/source/isaaclab/isaaclab/assets/articulation/base_articulation_data.py#L518)
- [base_articulation.py: set_joint_effort_target_index](../../IsaacLab/source/isaaclab/isaaclab/assets/articulation/base_articulation.py#L1837)
- [base_contact_sensor_data.py: net_normal_forces_w_history](../../IsaacLab/source/isaaclab/isaaclab/sensors/contact_sensor/base_contact_sensor_data.py#L92)
- [math.py: combine_frame_transforms](../../IsaacLab/source/isaaclab/isaaclab/utils/math.py#L786) / [matrix_from_quat](../../IsaacLab/source/isaaclab/isaaclab/utils/math.py#L164) / [quat_apply_inverse](../../IsaacLab/source/isaaclab/isaaclab/utils/math.py#L639)
- ドキュメント: [how-to/run_osc.rst](../../IsaacLab/docs/source/how-to/run_osc.rst)

---

## Isaac Lab 3.0 の注意点（この章で特にハマりやすい所）

- **クォータニオンは `(x, y, z, w)` 順**。単位姿勢は `[0, 0, 0, 1]`。2.x の記事や論文コード（`w` 先頭）を写すと姿勢がおかしくなる。チュートリアルの目標値もすべて xyzw。
- **データアクセスは `.torch`**: `robot.data.joint_pos` などは `ProxyArray` なので、torch 演算には `robot.data.joint_pos.torch` を使う。
- **書き込み API は `_index` 付き**: `write_joint_position_to_sim_index`, `set_joint_position_target_index`, `set_joint_effort_target_index`。
- **ヤコビアンのインデックス 2 種**:
  - ボディ軸: 固定ベースでは `jacobi_body_idx = body_idx - 1`（ルートが含まれない）
  - DoF 軸: 先頭に `num_base_dofs` 列（固定ベース 0、浮遊ベース 6）があるので `j + robot.num_base_dofs`。質量行列・重力ベクトルも同じ規約。
- ドキュメント上どちらも **Isaac Sim PhysX が必要**なチュートリアル。
- `run_osc.py` のコメントの小さな不一致: 接触力は「4 ステップ平均」「3 面の max」と書かれているが、実際は `history_length=2`、センサは壁 1 つ。コードの挙動を正とする。

---

## 章末チェック

- [ ] 微分 IK の 1 ステップ（誤差 → `J⁻` → `q + Δq`）を紙に書いて説明できる
- [ ] DLS が特異点で pinv より安定な理由を説明できる
- [ ] OSC の `Λ = (J M⁻¹ Jᵀ)⁻¹` が何のためにあるか説明できる
- [ ] ワールド座標のヤコビアン・速度をベース座標に変換するコードを白紙から書ける
- [ ] タスク座標系を使うと力制御軸の指定が簡単になる理由を説明できる
- [ ] 位置制御（PD アクチュエータ）とトルク制御（stiffness=0 + effort target）の切り替え方を説明できる

次へ → [06_deploy.md](06_deploy.md)
