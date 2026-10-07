# 01_assets — アセット（剛体・関節ロボット・変形体・グリッパ）を動かす

対応スクリプト: [scripts/tutorials/01_assets/](../../IsaacLab/scripts/tutorials/01_assets/)

## この章のゴール

- `XxxCfg` → `Xxx(cfg=...)` でアセットを作り、ループで「リセット → 書き込み → step → update → 読み出し」を回せる
- `asset.data.xxx` が `ProxyArray` であり、`.torch` で torch テンソルとして読むことを理解する
- 状態は**ワールド座標**で書き込む（環境原点 `origins` を足す必要がある）ことを説明できる
- 関節ロボットに対して、関節状態のリセットとアクチュエータ指令（effort / position / velocity）を出せる
- `ArticulationCfg` を自分で書いて、USD から新しいロボットを持ち込める

## 進める順番

| # | スクリプト | テーマ | 目安 |
|---|-----------|--------|------|
| 1 | `run_rigid_object.py` | 剛体を生成し、位置リセットと状態読み出し | 1時間 |
| 2 | `run_articulation.py` | Cartpole（関節ロボット）に関節リセットと effort 指令 | 1時間 |
| 3 | `add_new_robot.py` | `ArticulationCfg` を自作して Jetbot / Dofbot を動かす | 1.5時間 |
| 4 | `run_surface_gripper.py` | 吸着グリッパの開閉指令と状態取得（PhysX・CPU 専用） | 45分 |
| 5 | `run_deformable_object.py` | 変形体の節点状態リセットとキネマティック拘束 | 1.5時間 |

アルファベット順ではなく、依存関係順にしています。

- 1 → 2: 剛体の「ルート状態」を理解してから、関節状態とアクチュエータが加わる関節ロボットへ進む。
- 3 は 2 の続き。既製の `CARTPOLE_CFG` を使う側から、`ArticulationCfg` を**書く**側に回る。次章の `InteractiveSceneCfg` も少し先取りする。
- 4 は 2 と同じ構造（`design_scene` + 辞書）に `SurfaceGripper` を足すだけだが、PhysX 専用・`--device cpu` 必須という制約があるので後ろに置く。
- 5 は追加の依存（`--extra tetrahedralization`）が必要で、バックエンドごとに Cfg が変わり、`InteractiveSceneCfg` も使うので最後。

自作ファイルは `my_tutorials/01_assets/` に置きます。

### この章共通の Isaac Lab 3.0 ポイント

- **データは `ProxyArray`**: `obj.data.root_pos_w` などは Warp 配列のラッパー。`obj.data.root_pos_w.torch` で torch テンソル（ゼロコピー）を得る。書き換えるなら `.torch.clone()` してから。`.clone()` を `ProxyArray` に直接呼ぶことはできない。
- **クォータニオンは `(x, y, z, w)` 順**: `default_root_pose` は `[pos(3), quat(4)]` の 7 要素、単位姿勢は `(0, 0, 0, 1)`。
- **書き込み API は `_index` / `_mask` 付き**: `write_root_pose_to_sim_index(root_pose=...)` のようにキーワード引数で渡す。`env_ids` を省略すると全インスタンス。
- **バックエンド別 import**: PhysX 固有の Cfg やアセットは `isaaclab_physx`、Newton 固有は `isaaclab_newton` から import する。
- GUI が出ない場合は `--viz kit` を付ける（公式ドキュメントの実行例は `--viz kit` 付き）。

---

## Lesson 1: `run_rigid_object.py` — 剛体を動かす

### 学ぶこと

- **`design_scene()` と `run_simulator()` の分離**: `sim.reset()` 後に物理付きプリムを追加してはいけない。だから「シーン設計 → reset → ループ」の順になる。
- `RigidObjectCfg(prim_path, spawn, init_state)`: 00_sim の spawner Cfg（`ConeCfg` + `rigid_props` / `mass_props` / `collision_props`）を `spawn=` に包むだけ。
- **正規表現プリムパス**: `prim_path="/World/Origin.*/Cone"` とすると、既存の `/World/Origin0`〜`/World/Origin3` それぞれの下にコーンが生成され、1 つの `RigidObject` で 4 インスタンスをまとめて扱える（`num_instances == 4`）。
- **ルート状態はワールド座標**: 親 Xform の位置は無視されるので、`default_root_pose` に `origins` を足してから書き込む。
- ループの 4 拍子: `write_data_to_sim()` → `sim.step()` → `update(dt)` → `data` を読む。

### 自分で書くステップ

1. 00_sim の `create_empty.py` をコピー。import に `torch`, `isaaclab.utils.math as math_utils`, `from isaaclab.assets import RigidObject, RigidObjectCfg` を追加。
2. `design_scene()`: 地面と `DomeLightCfg` を置き、4 つの原点で `sim_utils.create_prim(f"/World/Origin{i}", "Xform", translation=origin)`。
3. `RigidObjectCfg` を作り、`RigidObject(cfg=...)` を生成。`{"cone": cone_object}` と `origins` を返す。
4. `main()` で `design_scene()` を **`sim.reset()` の前**に呼び、`origins` を `torch.tensor(..., device=sim.device)` に変換。
5. `run_simulator()`: まず reset せずに `write_data_to_sim / step / update` だけ回し、50 ステップごとに `cone_object.data.root_pos_w.torch` を print → 実行（原点に積み重なる様子を見る）。
6. 250 ステップごとのリセットを追加:
   - `root_pose = cone_object.data.default_root_pose.torch.clone()`
   - `root_pose[:, :3] += origins`、さらに `math_utils.sample_cylinder(radius, h_range, size, device)` でランダムな高さを足す
   - `write_root_pose_to_sim_index(root_pose=...)`、`default_root_vel` も同様に `write_root_velocity_to_sim_index(root_velocity=...)`
   - 最後に `cone_object.reset()`

### 実行

```bash
uv run python my_tutorials/01_assets/run_rigid_object.py
uv run python my_tutorials/01_assets/run_rigid_object.py --viz kit   # ビューポートを明示的に開く
```

### 確認ポイント

- [ ] 4 つの緑のコーンが少し違う高さから落ち、約 2.5 秒ごと（250 × dt）に再配置される
- [ ] 表示される `Root position` の shape が `(4, 3)`
- [ ] `origins` を足さないと何が起きるか説明できる（全部 (0,0,0) 付近に出る）

### 発展課題

- `RigidObjectCfg.InitialStateCfg(pos=..., rot=...)` を設定し、`rot` を `(x, y, z, w)` 順で 90° 回転させてみる
- リセット時に `default_root_vel` の z 成分に上向き速度を入れて、コーンを打ち上げる
- `env_ids` を使って偶数番目のコーンだけリセットする
- `root_pos_w` を `.torch` を付けずに使うと何が起きるか（DeprecationWarning）を確認する

### 参照コード

- [rigid_object_cfg.py: RigidObjectCfg](../../IsaacLab/source/isaaclab/isaaclab/assets/rigid_object/rigid_object_cfg.py#L16)
- [asset_base_cfg.py: InitialStateCfg（rot は x, y, z, w）](../../IsaacLab/source/isaaclab/isaaclab/assets/asset_base_cfg.py#L27)
- [base_rigid_object.py: reset / write_data_to_sim / update](../../IsaacLab/source/isaaclab/isaaclab/assets/rigid_object/base_rigid_object.py#L119)
- [base_rigid_object.py: write_root_pose_to_sim_index](../../IsaacLab/source/isaaclab/isaaclab/assets/rigid_object/base_rigid_object.py#L185)
- [base_rigid_object_data.py: default_root_pose](../../IsaacLab/source/isaaclab/isaaclab/assets/rigid_object/base_rigid_object_data.py#L110)
- [base_rigid_object_data.py: root_pos_w](../../IsaacLab/source/isaaclab/isaaclab/assets/rigid_object/base_rigid_object_data.py#L685)
- [proxy_array.py: ProxyArray](../../IsaacLab/source/isaaclab/isaaclab/utils/warp/proxy_array.py#L28)
- [math.py: sample_cylinder](../../IsaacLab/source/isaaclab/isaaclab/utils/math.py#L1458)
- 解説: [docs/source/how-to/run_rigid_object.rst](../../IsaacLab/docs/source/how-to/run_rigid_object.rst)

---

## Lesson 2: `run_articulation.py` — 関節ロボット（Cartpole）

### 学ぶこと

- **Articulation = ルート状態 + 関節状態**。リセットはルート（pose, vel）と関節（position, velocity）の両方を書く。
- 既製 Cfg（`from isaaclab_assets import CARTPOLE_CFG`）を `clone()` してから `prim_path` だけ差し替える。元の定数を直接書き換えない。
- **アクチュエータ指令**: 3.0 では `robot.actuators.target_command.set_effort_index(value=...)` で指令バッファに入れ、`write_data_to_sim()` でアクチュエータモデルを通して物理に渡る。
- effort 制御が効くのは、`CARTPOLE_CFG` のアクチュエータが `stiffness=0.0` に設定されているから（PD のバネ成分が無い）。

### 自分で書くステップ

1. Lesson 1 をコピーし、`RigidObject` を `Articulation` に置き換える。`from isaaclab.utils import clone` も import。
2. `design_scene()`: 原点を 2 つ（`[0,0,0]`, `[-1,0,0]`）にし、`cartpole_cfg = clone(CARTPOLE_CFG)` → `cartpole_cfg.prim_path = "/World/Origin.*/Robot"` → `Articulation(cfg=cartpole_cfg)`。
3. カメラを `set_camera_view([2.5, 0, 4], [0, 0, 2])` に（Cartpole は z=2 に立つ）。
4. 500 ステップごとのリセット: ルートは Lesson 1 と同じ。関節は `default_joint_pos.torch.clone()` に `torch.rand_like(...) * 0.1` のノイズを足し、`write_joint_position_to_sim_index(position=...)` / `write_joint_velocity_to_sim_index(velocity=...)`。
5. 毎ステップ `torch.randn_like(robot.data.joint_pos.torch) * 5.0` を effort として `set_effort_index` → `write_data_to_sim()` → `step` → `update`。

### 実行

```bash
uv run python my_tutorials/01_assets/run_articulation.py
```

### 確認ポイント

- [ ] 2 台の Cartpole が並び、カートとポールがランダムに揺れる
- [ ] `robot.joint_names` / `robot.data.joint_pos.torch.shape` を print して `(2, 2)`（2 台 × 2 関節）であることを確認
- [ ] `stiffness=0` でないと effort 指令が効きにくい理由を説明できる

### 発展課題

- `robot.find_joints("cart_to_pole")` で関節 index を取り、ポールだけに effort をかける（`joint_ids=` 引数）
- effort の代わりに `robot.actuators.target_command.set_position_index(value=...)` を試す（何が起きるか、stiffness との関係を考える）
- 各ステップで `robot.data.joint_pos.torch` を Lesson 0 の `log_time.py` 方式でファイルに記録する

### 参照コード

- [cartpole.py: CARTPOLE_CFG](../../IsaacLab/source/isaaclab_assets/isaaclab_assets/robots/cartpole.py#L20)
- [articulation_cfg.py: ArticulationCfg](../../IsaacLab/source/isaaclab/isaaclab/assets/articulation/articulation_cfg.py#L21)
- [base_articulation.py: BaseArticulation（actuators 属性の説明）](../../IsaacLab/source/isaaclab/isaaclab/assets/articulation/base_articulation.py#L50)
- [base_articulation.py: find_joints](../../IsaacLab/source/isaaclab/isaaclab/assets/articulation/base_articulation.py#L553)
- [base_articulation.py: write_joint_position_to_sim_index](../../IsaacLab/source/isaaclab/isaaclab/assets/articulation/base_articulation.py#L995)
- [actuator_collection.py: target_command](../../IsaacLab/source/isaaclab/isaaclab/actuators/actuator_collection.py#L138)
- [actuator_collection.py: set_effort_index](../../IsaacLab/source/isaaclab/isaaclab/actuators/actuator_collection.py#L755)
- [base_articulation_data.py: joint_pos](../../IsaacLab/source/isaaclab/isaaclab/assets/articulation/base_articulation_data.py#L964)
- [configclass.py: clone / replace / instantiate](../../IsaacLab/source/isaaclab/isaaclab/utils/configclass.py#L127)
- 解説: [docs/source/how-to/run_articulation.rst](../../IsaacLab/docs/source/how-to/run_articulation.rst)

---

## Lesson 3: `add_new_robot.py` — 自分で ArticulationCfg を書く

### 学ぶこと

- **`ArticulationCfg` の中身**: `spawn`（`UsdFileCfg(usd_path=...)` + 物理プロパティ）、`init_state`（`pos`, `joint_pos`）、`actuators`（名前 → `ImplicitActuatorCfg`）。
- `ImplicitActuatorCfg(joint_names_expr=[...], stiffness, damping, ...)`: `joint_names_expr` は正規表現（`".*"`, `"joint[1-2]"`）。`stiffness=None, damping=None` にすると USD に書かれたゲインをそのまま使う（両方指定が必要）。
- `init_state.joint_pos` のキーは**USD の関節名**であり、アクチュエータのグループ名ではない。
- **バックエンド別プロパティ**: `articulation_props=[PhysxArticulationCfg(...), NewtonArticulationCfg(...)]` のようにリストで並べる。`PhysxRigidBodyCfg` は `isaaclab_physx.sim.schemas`、`NewtonArticulationCfg` は `isaaclab_newton.sim.schemas` から import。
- **InteractiveSceneCfg の先取り**: クラス属性にアセット Cfg を並べ、`prim_path="{ENV_REGEX_NS}/Jetbot"` とすると `num_envs` 個の環境に複製される。`instantiate(scene_cfg)` でシーンを作り、`scene["Jetbot"]`、`scene.env_origins`、`scene.reset() / write_data_to_sim() / update()` で一括操作（詳細は次章）。
- 速度指令（Jetbot の車輪）と位置指令（Dofbot の腕）の使い分け。

### 自分で書くステップ

1. `JETBOT_CONFIG` を書く: `UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/NVIDIA/Jetbot/jetbot.usd")` と、全関節 `".*"` の `ImplicitActuatorCfg`（stiffness/damping は None）。
2. `NewRobotsSceneCfg(InteractiveSceneCfg)` を作り、地面・ライト（`AssetBaseCfg`）と `replace(JETBOT_CONFIG, prim_path="{ENV_REGEX_NS}/Jetbot")` を並べる。
3. `main()` で `NewRobotsSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)` → `instantiate(...)` → `sim.reset()`。ここで一度実行して Jetbot が出ることを確認。
4. ループ: リセット（`default_root_pose` に `scene.env_origins` を足す）と、`set_joint_velocity_target_index(target=...)` で直進 `[10, 10]` / 旋回 `[5, -5]` を切り替える。指令テンソルは**`sim.device` 上**に作る（CPU テンソルは弾かれる）。
5. `DOFBOT_CONFIG` を追加: `rigid_props`, `articulation_props`, `init_state.joint_pos`, `pos=(0.25, -0.25, 0)`、`joint[1-2]` / `joint3` / `joint4` の 3 グループのアクチュエータ（高 stiffness）。
6. Dofbot に `0.25 * sin(2π·0.5·t)` の位置目標を `set_joint_position_target_index(target=...)` で与えて手を振らせる。

### 実行

```bash
uv run python my_tutorials/01_assets/add_new_robot.py
uv run python my_tutorials/01_assets/add_new_robot.py --num_envs 4
```

### 確認ポイント

- [ ] Jetbot が直進と旋回を繰り返し、Dofbot が腕を振る
- [ ] `--num_envs 4` で 4 セットが `env_spacing` 間隔で並ぶ
- [ ] Dofbot のグリッパ関節は未設定なので「アクチュエータ未設定の関節」警告が出る（想定通り）
- [ ] `stiffness=None` と `stiffness=0.0` の違いを説明できる

### 発展課題

- Dofbot の残りの関節にもアクチュエータを足して警告を消す（`robot.joint_names` で名前を確認）
- 非推奨の `set_joint_*_target_index` を、`scene["Dofbot"].actuators.target_command.set_position_index(value=...)` / `set_velocity_index` に書き換える
- Dofbot の stiffness を 10000 → 100 に下げて、目標追従がどう変わるか観察する

### 参照コード

- [articulation_cfg.py: ArticulationCfg](../../IsaacLab/source/isaaclab/isaaclab/assets/articulation/articulation_cfg.py#L21)
- [actuator_pd_cfg.py: ImplicitActuatorCfg](../../IsaacLab/source/isaaclab/isaaclab/actuators/actuator_pd_cfg.py#L21)
- [base_articulation.py: set_joint_position_target_index（deprecated 注記あり）](../../IsaacLab/source/isaaclab/isaaclab/assets/articulation/base_articulation.py#L1639)
- [base_articulation.py: set_joint_velocity_target_index](../../IsaacLab/source/isaaclab/isaaclab/assets/articulation/base_articulation.py#L1760)
- [actuator_collection.py: ActuatorTargetCommand.set_position_index](../../IsaacLab/source/isaaclab/isaaclab/actuators/actuator_collection.py#L715)
- [interactive_scene_cfg.py: InteractiveSceneCfg](../../IsaacLab/source/isaaclab/isaaclab/scene/interactive_scene_cfg.py#L22)
- [configclass.py: replace](../../IsaacLab/source/isaaclab/isaaclab/utils/configclass.py#L142)
- 解説: [docs/source/how-to/write_articulation_cfg.rst](../../IsaacLab/docs/source/how-to/write_articulation_cfg.rst)（後半の「Example: configure and run two robots」）

---

## Lesson 4: `run_surface_gripper.py` — 吸着グリッパ

### 学ぶこと

- `SurfaceGripper` は **PhysX 専用**で、`from isaaclab_physx.assets import SurfaceGripper, SurfaceGripperCfg` から import する。
- Isaac Lab 3.0 / Isaac Sim 6.x では**CPU シミュレーションのみ対応** → `--device cpu` が必須。
- `SurfaceGripperCfg.prim_path` はロボット USD 内の SurfaceGripper プリム（`.../Robot/picker_head/SurfaceGripper`）を指す。`max_grip_distance`, `shear_force_limit`, `coaxial_force_limit`, `retry_interval` は未設定ならプリムから読まれる。
- 指令値は連続値 [-1, 1]: `< -0.3` で Opening、`-0.3〜0.3` で Idle、`> 0.3` で Closing。
- 状態 `surface_gripper.state` は Warp 配列: `-1` Open / `0` Closing / `1` Closed。読むときは `wp.to_torch(...)`。

### 自分で書くステップ

1. Lesson 2 をコピーし、`CARTPOLE_CFG` を `PICK_AND_PLACE_CFG` に差し替え（原点は `[±2.75, 0, 0]`、カメラは上から）。parser に `parser.set_defaults(visualizer=["kit"])` を入れる。
2. `SurfaceGripperCfg()` を作り、`prim_path` と 4 つのパラメータを設定して `SurfaceGripper(cfg=...)`。辞書に追加。
3. ループのリセット部分で `surface_gripper.reset()` も呼ぶ。
4. 毎ステップ `torch.rand(num_instances) * 2 - 1` を `set_grippers_command(...)` → `write_data_to_sim()` → `step` → `update(dt)`。
5. `state` を `wp.to_torch(...).tolist()` で文字列にマップして print。

### 実行

```bash
uv run python my_tutorials/01_assets/run_surface_gripper.py --device cpu
```

### 確認ポイント

- [ ] 2 台のピック＆プレースロボットが表示される
- [ ] ターミナルに指令（Opening/Idle/Closing）と状態（Open/Closing/Closed）が毎ステップ出る
- [ ] `--device cpu` を付けないとどうなるかを確認し、理由を説明できる

### 発展課題

- 非推奨の `set_grippers_command` / `reset` を `set_grippers_command_index` / `reset_index` に置き換える
- ランダム指令をやめ、「200 ステップ Closing → 200 ステップ Opening」の決まった周期にする
- ロボットの z 関節を下げてから Closing、持ち上げて Opening、という簡単なシーケンスを書く（物体は 02_scene 以降で追加しても良い）

### 参照コード

- [surface_gripper_cfg.py: SurfaceGripperCfg](../../IsaacLab/source/isaaclab_physx/isaaclab_physx/assets/surface_gripper/surface_gripper_cfg.py#L17)
- [surface_gripper.py: SurfaceGripper](../../IsaacLab/source/isaaclab_physx/isaaclab_physx/assets/surface_gripper/surface_gripper.py#L56)
- [surface_gripper.py: state](../../IsaacLab/source/isaaclab_physx/isaaclab_physx/assets/surface_gripper/surface_gripper.py#L120)
- [surface_gripper.py: set_grippers_command_index](../../IsaacLab/source/isaaclab_physx/isaaclab_physx/assets/surface_gripper/surface_gripper.py#L150)
- [pick_and_place.py: PICK_AND_PLACE_CFG](../../IsaacLab/source/isaaclab_assets/isaaclab_assets/robots/pick_and_place.py#L22)
- 解説: [docs/source/how-to/run_surface_gripper.rst](../../IsaacLab/docs/source/how-to/run_surface_gripper.rst)

---

## Lesson 5: `run_deformable_object.py` — 変形体

### 学ぶこと

- 変形体の状態は「ルート」ではなく**節点（メッシュ頂点）ごとの状態** `nodal_state`（位置 3 + 速度 3）。
- `transform_nodal_pos(nodal_pos, pos, quat)` で、節点群を剛体のように平行移動・回転できる（`quat` は x, y, z, w）。
- **キネマティック拘束**: `nodal_kinematic_target` は `(num_instances, 頂点数, 4)`。`[..., :3]` が目標位置、`[..., 3]` がフラグ（`0` = 拘束、`1` = 自由）。一部の頂点だけ動かし、残りはソルバに任せられる。
- **バックエンドで Cfg が変わる**: `--backend` で `physx` / `newton_vbd` / `ovphysx` を選び、PhysX 系は `PhysxDeformableBodyPropertiesCfg` + `PhysxDeformableBodyMaterialCfg`（ヤング率・ポアソン比）、Newton VBD は `NewtonDeformableBodyMaterialCfg`（ラメ定数 `k_mu`, `k_lambda` に換算）を使う。
- `launch_simulation(cfg=PhysicsCfg(), launcher_args=args_cli) as physics_cfg` で、選んだバックエンドの physics cfg を受け取り `SimulationCfg(physics=physics_cfg)` に渡す。
- シーンは `InteractiveSceneCfg` で `num_envs=4` に複製（`{ENV_REGEX_NS}/Cube`）。

### 自分で書くステップ

1. parser に `--backend` を追加し、`args_cli.physics = args_cli.backend` を設定。最初は `physx` 固定の分岐だけ書いて良い。
2. `deformable_props`, `collision_props`, `physics_material` をバックエンドに応じて作る（`isaaclab_physx.sim.schemas` / `isaaclab_physx.sim.spawners.materials` から import）。
3. `DeformableSceneCfg(InteractiveSceneCfg)`: 地面・ライトと、`DeformableObjectCfg(prim_path="{ENV_REGEX_NS}/Cube", spawn=MeshCuboidCfg(...), init_state=..., debug_vis=True)`。
4. `main()` で `launch_simulation(cfg=PhysicsCfg(), ...)` → `SimulationCfg(dt=0.01, physics=physics_cfg)` → `instantiate(scene_cfg)` → `sim.reset()`。ここで一度実行し、4 つのキューブが落ちて潰れるのを確認。
5. 3 秒ごとのリセット: `default_nodal_state_w.torch.clone()` に、ランダム位置（`+ scene.env_origins`）と `math_utils.random_orientation` を `transform_nodal_pos` で適用 → `write_nodal_state_to_sim_index`。同時にキネマティック目標を全頂点「自由」（`[..., 3] = 1.0`）にして書き込む。
6. 毎ステップ: キューブ 1, 2 の頂点 0 の z 目標を `0.2 * dt` ずつ上げ、`index_fill_` でその頂点のフラグを 0（拘束）に → `write_nodal_kinematic_target_to_sim_index`。

### 実行

```bash
uv run --extra isaacsim --extra tetrahedralization python my_tutorials/01_assets/run_deformable_object.py
uv run --extra isaacsim --extra tetrahedralization python my_tutorials/01_assets/run_deformable_object.py --backend newton_vbd
```

### 確認ポイント

- [ ] 4 つの柔らかいキューブがランダムな姿勢で落ち、潰れる
- [ ] キューブ 1, 2 は 1 つの角（頂点 0）だけが吊り上げられていく（マーカーで目標位置が見える）
- [ ] 1 秒ごとに `Root position` が表示される
- [ ] `nodal_kinematic_target[..., 3]` の 0/1 の意味を説明できる

### 発展課題

- `youngs_modulus` を 1e4 / 1e6 に変えて硬さの違いを見る
- 全キューブの頂点 0 を拘束し、その場で固定（吊るす）してみる
- `physx` と `newton_vbd` で同じ設定の見た目を比較する

### 参照コード

- [deformable_object_cfg.py: DeformableObjectCfg](../../IsaacLab/source/isaaclab/isaaclab/assets/deformable_object/deformable_object_cfg.py#L20)
- [base_deformable_object.py: write_nodal_state_to_sim_index](../../IsaacLab/source/isaaclab/isaaclab/assets/deformable_object/base_deformable_object.py#L113)
- [base_deformable_object.py: write_nodal_kinematic_target_to_sim_index](../../IsaacLab/source/isaaclab/isaaclab/assets/deformable_object/base_deformable_object.py#L177)
- [base_deformable_object.py: transform_nodal_pos](../../IsaacLab/source/isaaclab/isaaclab/assets/deformable_object/base_deformable_object.py#L339)
- [base_deformable_object_data.py: default_nodal_state_w / nodal_kinematic_target](../../IsaacLab/source/isaaclab/isaaclab/assets/deformable_object/base_deformable_object_data.py#L48)
- [isaaclab_physx: DeformableObject](../../IsaacLab/source/isaaclab_physx/isaaclab_physx/assets/deformable_object/deformable_object.py#L45)
- [math.py: random_orientation](../../IsaacLab/source/isaaclab/isaaclab/utils/math.py#L1320)
- [array.py: index_fill_](../../IsaacLab/source/isaaclab/isaaclab/utils/array.py#L47)
- 解説: [docs/source/how-to/run_deformable_object.rst](../../IsaacLab/docs/source/how-to/run_deformable_object.rst)

---

## 章末チェック

- [ ] 白紙から `run_rigid_object.py` 相当（生成 → リセット → ループ → 位置表示）を 15 分以内に書ける
- [ ] `data.xxx.torch.clone()` が必要な理由（ProxyArray / 書き換え前のコピー）を説明できる
- [ ] 状態をワールド座標で書くため `origins` / `env_origins` を足すことを説明できる
- [ ] `write_data_to_sim → step → update` の各役割を説明できる
- [ ] `ArticulationCfg` の `spawn` / `init_state` / `actuators` を自分で書ける
- [ ] PhysX 専用・CPU 専用・追加依存が必要なスクリプトを見分けられる

次へ → [02_scene.md](02_scene.md)
