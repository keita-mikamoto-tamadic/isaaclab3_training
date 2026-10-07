# 04_sensors — センサーを付けてデータを読む

対応スクリプト: [scripts/tutorials/04_sensors/](../../IsaacLab/scripts/tutorials/04_sensors/)
関連ドキュメント: [add_sensors_on_robot.rst](../../IsaacLab/docs/source/how-to/add_sensors_on_robot.rst), [save_camera_output.rst](../../IsaacLab/docs/source/how-to/save_camera_output.rst)

## この章のゴール

- `XxxCfg(prim_path=..., update_period=..., debug_vis=...)` → `scene["name"].data.xxx` という**センサー共通の型**を理解する
- カメラ・レイキャスタ（高さスキャナ）・接触センサ・フレーム変換センサの用途と出力 shape を説明できる
- センサーを `InteractiveScene` に入れる方法と、単体で `instantiate(cfg)` して手動で `update()` する方法の両方を書ける
- USD カメラ（レンダリング）とレイキャストカメラ（Warp でメッシュに当てるだけ）の違いを説明できる

## 進める順番

| # | スクリプト | テーマ | 目安 |
|---|-----------|--------|------|
| 1 | `add_sensors_on_robot.py` | ANYmal にカメラ・高さスキャナ・足の接触センサを付ける（全体像） | 1.5時間 |
| 2 | `run_ray_caster.py` | レイキャスタを単体で作り、手動で `update()` する | 45分 |
| 3 | `run_ray_caster_camera.py` | レイキャストで深度画像を作り、点群へ逆投影 | 1時間 |
| 4 | `run_usd_camera.py` | RTX レンダリングのカメラ：RGB/深度/セグメンテーション、保存と点群描画 | 1.5時間 |
| 5 | `run_frame_transformer.py` | ロボットのリンク間の相対姿勢を取る | 45分 |

公式 how-to と同じく、まず 1 で「シーンにセンサーを入れる」全体像をつかみ、2〜4 でレイキャスト系 → レンダリング系カメラと掘り下げ、最後に毛色の違う FrameTransformer を扱います。

### この章の共通の注意

- **GUI は `--viz kit`**。指定しなければビジュアライザなし（GUI なし）で動きます。
- **カメラ（`CameraCfg`）にはレンダリング拡張が必要**。`launch_simulation` は渡された cfg にカメラがあれば自動で有効化しますが、チュートリアルは `SimulationCfg` しか渡さないので、`args_cli.enable_cameras = True` を**パース直後に自分で**設定しています（CLI フラグではなく属性代入）。レイキャストカメラには不要です。
- **アセットのダウンロード**: ANYmal-C（`isaaclab_assets`）や `ISAAC_NUCLEUS_DIR/Environments/Terrains/rough_plane.usd` はネット経由で取得するため、初回は時間がかかります。
- **クォータニオンは `(x, y, z, w)` 順**（Isaac Lab 3.0）。`OffsetCfg.rot` のデフォルトが `(0, 0, 0, 1)` であることをソースで確認しておくこと。
- **データは `ProxyArray`**。テンソルが欲しいときは `.torch`（例: `data.ray_hits_w.torch`）。

---

## Lesson 1: `add_sensors_on_robot.py` — ロボットにセンサーを付ける

### 学ぶこと

- **センサーもシーンの一要素**: 03 章までの `InteractiveSceneCfg` に、アセットと同じようにセンサー cfg をクラス属性として書くだけ。`scene["camera"]` で取り出せる。
- **共通パラメータ**（`SensorBaseCfg`）: `prim_path`（`{ENV_REGEX_NS}` 使用可）、`update_period`（秒。0 なら毎ステップ）、`debug_vis`。
- **カメラ** `CameraCfg`: `spawn=sim_utils.PinholeCameraCfg(...)` でカメラプリムを**新規に作る**。`width/height`, `data_types=["rgb", "distance_to_image_plane"]`, `offset=CameraCfg.OffsetCfg(pos, rot, convention="ros")`。
- **高さスキャナ** `RayCasterCfg`: `pattern_cfg=patterns.GridPatternCfg(resolution, size)`, `ray_alignment="yaw"`（ロール・ピッチを無視して地面へ下向きに撃つ）, `mesh_prim_paths=[地面]`, 高い位置から撃つため `offset.pos=(0, 0, 20)`。
- **接触センサ** `ContactSensorCfg`: `prim_path=".../Robot/.*_FOOT"` のように**既存の剛体**を正規表現で指定。アセット側で `activate_contact_sensors=True` が必要（`ANYMAL_C_CFG` は設定済み）。`history_length` で過去ステップを保持。
- センサーのバッファは `sim.reset()` 後に初期化され、`scene.update(dt)` の中で `update_period` に従って更新される。

### 自分で書くステップ

1. 03 章（または 02 章 `create_scene.py`）の骨格をコピー。`--num_envs`（default 2）を追加し、`args_cli = parser.parse_args()` の直後に `args_cli.enable_cameras = True`。
2. `SensorsSceneCfg(InteractiveSceneCfg)` に地面・ドームライト・`replace(ANYMAL_C_CFG, prim_path="{ENV_REGEX_NS}/Robot")` を置き、まずセンサー無しで動くことを確認。
3. `contact_forces = ContactSensorCfg(...)` を追加 → ループで `scene["contact_forces"].data.net_normal_forces_w` の max を print。
4. `height_scanner = RayCasterCfg(...)`（`debug_vis=True`）を追加 → `data.ray_hits_w.torch[..., -1]`（z 成分）の max を print。
5. `camera = CameraCfg(...)` を `prim_path="{ENV_REGEX_NS}/Robot/base/front_cam"` で追加 → `data.output["rgb"].shape` と深度の shape を print。
6. ループは 03 章と同様: 500 ステップごとに root pose（`env_origins` を足す）/ 関節をリセット → `set_joint_position_target_index` → `scene.write_data_to_sim()` → `sim.step()` → `scene.update(sim_dt)`。
7. `print(scene["camera"])` のようにセンサーを print すると設定の要約が出るので活用する。

### 実行

```bash
uv run python my_tutorials/04_sensors/add_sensors_on_robot.py --viz kit
uv run python my_tutorials/04_sensors/add_sensors_on_robot.py --num_envs 4   # GUI なし
```

### 確認ポイント

- [ ] ロボットの周りに赤い点（レイのヒット位置）が表示される
- [ ] rgb は `(num_envs, 480, 640, 3)`、深度は `(num_envs, 480, 640, 1)` になる
- [ ] 着地後、接触力の max が 0 でない値になる
- [ ] ビューポートをカメラ視点（`front_cam`）に切り替えて画像を見られる
- [ ] `update_period=0.1` のカメラと `0.0` の接触センサで更新頻度が違う理由を説明できる

### 発展課題

- `ContactSensorCfg` に `track_air_time=True` を足し、`data.current_air_time` を print（02 章以降の歩行報酬で使う量）
- `GridPatternCfg` の `resolution`/`size` を変えて `num_rays` と ray_hits の shape がどう変わるか確認
- `ray_alignment` を `"base"` / `"world"` に変え、ロボットが傾いたときの赤点の違いを観察
- カメラの `offset.convention` を `"world"` にし、同じ向きにするには `rot` をどう書けばよいか考える

### 参照コード

- [sensor_base_cfg.py: SensorBaseCfg](../../IsaacLab/source/isaaclab/isaaclab/sensors/sensor_base_cfg.py#L16)
- [sensor_base.py: SensorBase.update](../../IsaacLab/source/isaaclab/isaaclab/sensors/sensor_base.py#L197)
- [camera_cfg.py: CameraCfg](../../IsaacLab/source/isaaclab/isaaclab/sensors/camera/camera_cfg.py#L35)
- [ray_caster_cfg.py: RayCasterCfg](../../IsaacLab/source/isaaclab/isaaclab/sensors/ray_caster/ray_caster_cfg.py#L25)
- [patterns_cfg.py: GridPatternCfg](../../IsaacLab/source/isaaclab/isaaclab/sensors/ray_caster/patterns/patterns_cfg.py#L34)
- [contact_sensor_cfg.py: ContactSensorCfg](../../IsaacLab/source/isaaclab/isaaclab/sensors/contact_sensor/contact_sensor_cfg.py#L19)
- [base_contact_sensor_data.py: net_normal_forces_w](../../IsaacLab/source/isaaclab/isaaclab/sensors/contact_sensor/base_contact_sensor_data.py#L78)
- [anymal.py: ANYMAL_C_CFG（activate_contact_sensors）](../../IsaacLab/source/isaaclab_assets/isaaclab_assets/robots/anymal.py#L100)
- [sim_launcher.py: カメラ検出による enable_cameras 自動有効化](../../IsaacLab/source/isaaclab/isaaclab/app/sim_launcher.py#L611)

---

## Lesson 2: `run_ray_caster.py` — レイキャスタ単体

### 学ぶこと

- `InteractiveScene` を使わず、`instantiate(RayCasterCfg(...))` でセンサーを直接作る方法。この場合 `update()` / `reset()` は**自分で呼ぶ**。
- `prim_path="/World/Origin.*/ball"` で複数の剛体（ボール）それぞれにセンサーを付ける。
- `update(dt, force_recompute=True)` で `update_period` を無視して毎回計算させる。
- `Timer(..., synchronize="both", device=...)` で GPU 処理時間を計測する。
- 凸凹地形 `rough_plane.usd` を `UsdFileCfg` で読み込み、それに対してレイを当てる。

### 自分で書くステップ

1. 骨格を書く（`enable_cameras` は不要）。
2. `design_scene()`: `rough_plane.usd` を `/World/ground` に、`DistantLightCfg`、`create_prim(f"/World/Origin{i}", "Xform", translation=...)` を 4 つ。
3. `RigidObjectCfg(prim_path="/World/Origin.*/ball", spawn=SphereCfg(...剛体・質量・衝突...))` → `RigidObject(cfg)`。
4. `define_sensor()`: `RayCasterCfg(prim_path=ボールと同じ式, mesh_prim_paths=["/World/ground"], pattern_cfg=GridPatternCfg(0.1, (2.0, 2.0)), ray_alignment="yaw", debug_vis=bool(args_cli.visualizer))` → `instantiate`。
5. ループ: 250 ステップごとにボールをランダム位置（`default_root_pose` を clone して xyz を上書き）へ戻し `ray_caster.reset()`。毎ステップ `sim.step()` → `ray_caster.update(...)`。
6. `update` を `with Timer(...)` で囲み、`ray_caster.num_rays` と `data.pos_w.torch` の max を表示。

### 実行

```bash
uv run python my_tutorials/04_sensors/run_ray_caster.py --viz kit
```

### 確認ポイント

- [ ] 4 つの青いボールが落下し、その下の地形にヒット点が描かれる
- [ ] Timer の出力で 1 回の更新にかかる時間が表示される
- [ ] `--viz` 無しで実行すると `debug_vis` が False になる理由をコードから説明できる

### 発展課題

- `patterns.LidarPatternCfg` や `BpearlPatternCfg` に差し替えて点の分布を比べる
- `max_distance` を小さくし、外れたレイが `inf` になることを確認
- `force_recompute=False` にして `update_period=0.1` を設定し、値が更新されるタイミングを確かめる

### 参照コード

- [base_ray_caster.py: BaseRayCaster](../../IsaacLab/source/isaaclab/isaaclab/sensors/ray_caster/base_ray_caster.py#L34)
- [ray_caster_data.py: RayCasterData（pos_w / ray_hits_w）](../../IsaacLab/source/isaaclab/isaaclab/sensors/ray_caster/ray_caster_data.py#L18)
- [patterns_cfg.py: パターン一覧](../../IsaacLab/source/isaaclab/isaaclab/sensors/ray_caster/patterns/patterns_cfg.py)
- [isaaclab_physx: RayCaster 実装](../../IsaacLab/source/isaaclab_physx/isaaclab_physx/sensors/ray_caster/ray_caster.py#L177)
- [timer.py: Timer](../../IsaacLab/source/isaaclab/isaaclab/utils/timer.py#L31)

---

## Lesson 3: `run_ray_caster_camera.py` — レイキャストカメラ

### 学ぶこと

- `RayCasterCameraCfg` + `patterns.PinholeCameraPatternCfg(focal_length, horizontal_aperture, height, width)` で、ピンホールカメラの画素ごとにレイを飛ばして深度・法線を得る。**レンダリングしない**ので RGB は無いが軽い。
- USD カメラとの違い: レイキャストカメラは**既存の Xform を親**にする（自分で `create_prim(".../CameraSensor", "Xform")` してから cfg の `prim_path` に指定）。
- `data_types=["distance_to_image_plane", "normals", "distance_to_camera"]`。
- 姿勢設定: `set_world_poses_from_view(eyes, targets)`（注視点指定）または `set_world_poses(pos, quat, convention="ros")`。
- `unproject_depth(depth, K)` で深度 → カメラ座標の点群、`project_points(points, K)` で逆に (u, v, d) へ。`camera.data.intrinsic_matrices` が K。
- `save_images_to_file(images, path)` で PNG 保存（値は [0, 1] に正規化しておく）。

### 自分で書くステップ

1. 骨格に `--save` を追加（`--num_envs` は元スクリプトでも未使用なので省略可）。
2. 地形・ライトを置き、`/World/Origin_00/CameraSensor`, `/World/Origin_01/CameraSensor` の Xform を作る。
3. `RayCasterCameraCfg(prim_path="/World/Origin_[^/]+/CameraSensor", mesh_prim_paths=[...], update_period=0.1, data_types=[...], pattern_cfg=...)` → `instantiate`。
4. ループ前に `set_world_poses_from_view` で 2 台を原点に向ける。
5. ループ: `sim.step()` → `camera.update(dt)` → `data.output["distance_to_image_plane"].shape` を print。
6. `--save` 時: 0 番カメラの深度を `torch.nan_to_num(..., posinf=0)` → 最大値で割って `save_images_to_file`。ファイル名に `camera.frame[i]` を使う。
7. 余力があれば `unproject_depth` → `project_points` で元の深度に戻ることを `torch.testing.assert_close` で検証。

### 実行

```bash
uv run python my_tutorials/04_sensors/run_ray_caster_camera.py --viz kit
uv run python my_tutorials/04_sensors/run_ray_caster_camera.py --save
```

### 確認ポイント

- [ ] 深度画像の shape が `(2, 480, 640, 1)`
- [ ] `--save` で自作スクリプトと同じフォルダの `output/ray_caster_camera/depth_XXX.png` が生成される
- [ ] `enable_cameras` を設定していないのに動く理由を説明できる
- [ ] `unproject_depth` → `project_points` の往復チェックが通る

### 発展課題

- `transform_points(points, camera.data.pos_w, camera.data.quat_w_ros)` でワールド座標の点群にする
- `normals` を `(n + 1) / 2` で [0, 1] に変換して PNG 保存
- `set_world_poses(..., convention="ros")` 版（スクリプト内のコメント）に書き換えて同じ視点になるか確認

### 参照コード

- [ray_caster_camera_cfg.py: RayCasterCameraCfg](../../IsaacLab/source/isaaclab/isaaclab/sensors/ray_caster/ray_caster_camera_cfg.py#L20)
- [patterns_cfg.py: PinholeCameraPatternCfg](../../IsaacLab/source/isaaclab/isaaclab/sensors/ray_caster/patterns/patterns_cfg.py#L73)
- [base_ray_caster_camera.py: set_world_poses_from_view](../../IsaacLab/source/isaaclab/isaaclab/sensors/ray_caster/base_ray_caster_camera.py#L250)
- [math.py: unproject_depth](../../IsaacLab/source/isaaclab/isaaclab/utils/math.py#L1152) / [project_points](../../IsaacLab/source/isaaclab/isaaclab/utils/math.py#L1239)
- [camera/utils.py: save_images_to_file](../../IsaacLab/source/isaaclab/isaaclab/sensors/camera/utils.py#L261)

---

## Lesson 4: `run_usd_camera.py` — USD（RTX）カメラ

### 学ぶこと

- `CameraCfg` は**レンダラで描画する**カメラ。RGB に加え `normals`, `semantic_segmentation`, `instance_segmentation`, `instance_id_segmentation_fast` などが取れる（対応 data_types は `Camera` クラスの docstring 参照）。
- `renderer_cfg=IsaacRtxRendererCfg(colorize_...=True)`（`isaaclab_physx.renderers`）でセグメンテーションをカラー画像（uint8, 4ch）にする。`CameraCfg` 側の `colorize_*` は非推奨。
- セマンティクスはスポーン時に `semantic_tags=[("class", "Cube")]` で付ける。
- USD カメラは `spawn` で**カメラプリム自体を作る**（親 Xform `/World/Origin_00` は先に作り、`prim_path` は `.../CameraSensor`）。
- 保存: uint8 は `save_images_to_file`（/255 して float に）、float（深度・法線）は `np.save`。
- 点群描画: `create_pointcloud_from_depth(intrinsic_matrix, depth, position, orientation=quat_w_ros)` → `VisualizationMarkers(RAY_CASTER_MARKER_CFG を replace したもの).visualize(translations=...)`。

### 自分で書くステップ

1. 骨格に `--draw`, `--save`, `--camera_id`（choices {0, 1}）を追加し、`args_cli.enable_cameras = True`。
2. 地面・ライト・`/World/Objects` を作り、ランダムな位置・色・形（Cube/Cone/Cylinder）の剛体を 8 個 `RigidObject` で置く。共通プロパティは dict にまとめて `**` で渡すと楽。
3. `define_sensor()`: 親 Xform 2 つ → `CameraCfg(update_period=0, height=480, width=640, data_types=[...], renderer_cfg=..., spawn=PinholeCameraCfg(...))` → `instantiate`。最初は `data_types=["rgb"]` だけで動かし、少しずつ増やす。
4. ループ前に `set_world_poses_from_view` で 2 台を原点に向ける。
5. ループ: `sim.step()` → `camera.update(dt)` → `data.output` の各キーの shape を print。
6. `--save` 時: `camera.data.output.items()` を回し、dtype で PNG / npy を分けて保存。
7. `--draw` 時: `sim.get_setting("/isaaclab/has_gui")` が真のときだけマーカーをループ**外**で作り、点群が空でなければ描画。

### 実行

```bash
uv run python my_tutorials/04_sensors/run_usd_camera.py --viz kit --draw
uv run python my_tutorials/04_sensors/run_usd_camera.py --save --camera_id 1   # GUI なしで保存のみ
```

### 確認ポイント

- [ ] 物体が落下し、各 data_type の shape が表示される（rgb は 3ch、セグメンテーションは colorize で 4ch）
- [ ] `output/camera/` に `rgb_*.png`, `semantic_segmentation_*.png`, `distance_to_image_plane_*.npy` などができる
- [ ] `--draw` で物体表面に点群が重なって見える
- [ ] 最初の数ステップで点群が空になり得る理由（コメント参照）を説明できる
- [ ] レイキャストカメラとの違い（RGB・動的物体が写る / 重い・要レンダリング）を説明できる

### 発展課題

- `semantic_tags` を変えて同じクラスが同じ色になることを確認
- `width/height` を 64x64 に下げて速度の違いを体感（学習用の低解像度カメラ）
- Lesson 3 の `unproject_depth` + `transform_points` で同じ点群を自分で作り、`create_pointcloud_from_depth` と一致するか比較

### 参照コード

- [camera.py: Camera（対応 data_types 一覧）](../../IsaacLab/source/isaaclab/isaaclab/sensors/camera/camera.py#L144)
- [camera.py: set_world_poses_from_view](../../IsaacLab/source/isaaclab/isaaclab/sensors/camera/camera.py#L544)
- [camera_data.py: output / intrinsic_matrices / quat_w_ros](../../IsaacLab/source/isaaclab/isaaclab/sensors/camera/camera_data.py#L95)
- [isaac_rtx_renderer_cfg.py: IsaacRtxRendererCfg](../../IsaacLab/source/isaaclab_physx/isaaclab_physx/renderers/isaac_rtx_renderer_cfg.py#L121)
- [camera/utils.py: create_pointcloud_from_depth](../../IsaacLab/source/isaaclab/isaaclab/sensors/camera/utils.py#L79)
- [visualization_markers.py: VisualizationMarkers.visualize](../../IsaacLab/source/isaaclab/isaaclab/markers/visualization_markers.py#L151)
- [sensors_cfg.py: PinholeCameraCfg](../../IsaacLab/source/isaaclab/isaaclab/sim/spawners/sensors/sensors_cfg.py#L141)

---

## Lesson 5: `run_frame_transformer.py` — フレーム変換センサ

### 学ぶこと

- `FrameTransformerCfg(prim_path=ソース, target_frames=[FrameCfg(...), ...])` で、ソース剛体から見たターゲット剛体の姿勢を毎ステップ取得する（足先位置の観測などに使う）。
- `FrameCfg(prim_path=".../Robot/.*")` のように正規表現で全リンクを一括指定できる。`name=` で独自名、`offset=OffsetCfg(pos, rot)` で剛体からずらした仮想フレーム（例: すね → 足裏）を定義できる。
- データ: `target_frame_names`, `source_pos_w/source_quat_w`, `target_pos_w/target_quat_w`（ワールド）, `target_pos_source`（ソース座標系）。
- `quat_from_euler_xyz`, `quat_apply` でオフセット回転・並進を計算する。
- シーンを使わずに `robot.update()` と `frame_transformer.update()` を個別に呼ぶ書き方もできる。

### 自分で書くステップ

1. Lesson 1 のシーンから地面・ライト・ANYmal だけ残す（`enable_cameras` 不要）。
2. モジュール先頭で `ROT_OFFSET = quat_from_euler_xyz(0, 0, -pi/2)`、`POS_OFFSET = quat_apply(ROT_OFFSET[0], 足先までのベクトル)` を計算。
3. `frame_transformer = FrameTransformerCfg(prim_path=".../Robot/base", target_frames=[全リンク, LF_SHANK + offset で "LF_FOOT_USER"])`。
4. `scene_cfg = ...(num_envs=1, env_spacing=0.0, replicate_physics=False)` で instantiate。
5. ループ: 既定関節位置を目標に `robot.write_data_to_sim()` → `sim.step()` → `robot.update()` / `frame_transformer.update()`。
6. `sim.has_gui` のとき `FRAME_MARKER_CFG` を `replace` してマーカーを作り、50 ステップごとに `frame_index` を進めて名前を print、ソースとターゲットの座標軸を `visualize(pos, quat)` で描く。

### 実行

```bash
uv run python my_tutorials/04_sensors/run_frame_transformer.py --viz kit
```

### 確認ポイント

- [ ] `Displaying Frame ID N: <名前>` が順に切り替わり、対応するリンクに座標軸が表示される
- [ ] `LF_FOOT_USER` が LF の足裏付近に出る
- [ ] `target_frame_names` の並び順と `target_pos_w` の第 2 次元の対応を説明できる

### 発展課題

- `target_pos_source` を print し、base から見た足先位置が関節角に応じて変わることを確認
- 4 本の足すべてに `offset` 付きのフレームを定義する
- cfg の `debug_vis=True` にしてスクリプトのマーカーと比較

### 参照コード

- [frame_transformer_cfg.py: FrameTransformerCfg / OffsetCfg](../../IsaacLab/source/isaaclab/isaaclab/sensors/frame_transformer/frame_transformer_cfg.py#L30)
- [base_frame_transformer_data.py: target_frame_names ほか](../../IsaacLab/source/isaaclab/isaaclab/sensors/frame_transformer/base_frame_transformer_data.py#L34)
- [math.py: quat_from_euler_xyz](../../IsaacLab/source/isaaclab/isaaclab/utils/math.py#L261) / [quat_apply](../../IsaacLab/source/isaaclab/isaaclab/utils/math.py#L621)
- [markers/config: FRAME_MARKER_CFG](../../IsaacLab/source/isaaclab/isaaclab/markers/config/__init__.py#L64)
- [simulation_context.py: has_gui](../../IsaacLab/source/isaaclab/isaaclab/sim/simulation_context.py#L299)

---

## 章末チェック

- [ ] 白紙から「ANYmal + 足の接触センサ + 高さスキャナ」のシーンを 15 分以内に書ける
- [ ] `update_period`, `debug_vis`, `history_length`, `ray_alignment` の意味を説明できる
- [ ] シーン経由（`scene.update`）と単体（`sensor.update(dt)`）の更新方法の違いを説明できる
- [ ] USD カメラとレイキャストカメラの違い、および `enable_cameras` が要る / 要らない理由を説明できる
- [ ] `.data.xxx` が `ProxyArray` であり、`.torch` でテンソルを取り出すことを知っている

次へ → [05_controllers.md](05_controllers.md)
