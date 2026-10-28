# Phase 4 implementation notes

Status: **stopped at Step 2: experiments c and d failed.** The Isaac
camera image is too coarse for the ChArUco markers to be decoded. Per the
plan ("If any fails, stop and record it"), Steps 3 and 4 were not done, and
`tests/e2e/phase4_check.py` was not written.

These parts are done and verified:

- the phase 3 carry-overs
- the phase 3 code-review fixes
- Step 1 (decisions 1–5 and 7)
- experiments a and b

The phase 3 checks pass on the final image (hwm 7/7, scene 5/5, cuj 8/8),
and `pytest tests/` passes (26 tests). No container or Isaac process is
left running.

## Step 2 experiments

| | Condition | Result |
|---|---|---|
| a | `GetObject(resource_handle_name="d455")` on `world` finds the decision 7 object | **pass**: `d455` with frame `sensor` |
| b | `franka`'s object ID is the same in `world` and `init_world` after the clone | **pass**: `franka` `ofid_2996765320`, `flange` `ofid_2996765338` in both worlds |
| c | board attached, arm at the decision 6 start pose: `Initialize` and one `CaptureData` in < 5 s with `NO_ERRORS` | **fail**: `CAPTURE_DATA_STATUS_NO_DETECTIONS` (time is fine: `Initialize` 0.01 s, total 0.22 s) |
| d | 20 sampled poses (decision 8): at least 16 `NO_ERRORS` | **fail**: 0 of 20 (16 `NO_DETECTIONS`, 4 `INSUFFICIENT_MARKER_POINTS`); samples 15 and 19 were skipped because `MoveToPose` returned `NOT_FOUND` (no IK solution) |

How to reproduce: `.venv/bin/python tests/e2e/exp4/exp4_ab.py` and
`tests/e2e/exp4/exp4_cd.py` against a running container and Isaac. The
outputs are in `logs/phase4_exp_ab.log` and `logs/phase4_exp_cd.log`. The
second run of c and d gave the same results as the first.

Experiment d moves through `RobotService.MoveToPose` (tool `FLANGE`),
taking the sampled poses in order until 20 moves have succeeded. It waits
0.5 s after each move, then calls `CaptureData`. It uses the decision 8
sampler (the image's `sample_poses` CLI, run through `docker exec`). It
does not use decision 11.4's IK and plan filter, which is part of Step 3.
A failed move counts as skipped.

### Why c and d fail (facts)

- **Isaac renders the camera at half resolution and upscales it with
  DLSS.** The camera is 848 × 480 with `fx` = 420.08 (from `camera_sim`'s
  `camera_info`). Isaac's log at start-up says: `[rtx.postprocessing.plugin]
  DLSS increasing input dimensions: Render resolution of (424, 240) is below
  minimal input resolution of 300.` In the captured image the markers are
  smeared shapes (`logs/phase4_exp_c_start_rgb.png`).
- **Geometry at the start pose:** the board is about 0.69 m from the
  camera, so a 30 mm marker is about 18 px wide in the output image. A
  5 × 5 marker with its border is 7 cells, so that is about 2.6 px per
  cell, and fewer in DLSS's internal render.
- **Plain OpenCV on the same image** (`DICT_5X5_100`, the same 7 × 5
  board) finds 0 markers (22 rejected candidates) and 0 ChArUco corners.
  So the problem is the image, not `CalibrationService`.
- **Closer probes** (same orientation rule as decision 6, flange at
  z = 0.55 m):

  | Flange x | Board distance to camera | Markers | ChArUco corners |
  |---|---|---|---|
  | 0.70 m | ≈ 0.63 m | 1 | 0 |
  | 0.80 m | ≈ 0.55 m | 4 | 1 |

  The 0.80 m image is `logs/phase4_exp_c_near_0.80.png`.
- **The rest of the pipeline works.** `Initialize` resolves the camera
  through `camera_sim` (`DescribeCamera`). Each `CaptureData` takes
  about 0.2 s, and that time includes the high-consistency KV write
  through the new zenohd storage (decision 4). The `AssetInstances`
  stand-in serves `d455`. The calibration service classifies every
  capture.

### Options for the designer

None of these was applied; each one goes beyond this plan.

- Isaac render settings for the camera: DLSS off or in a quality mode, or
  another anti-aliasing mode.
- A higher camera resolution.
- Larger squares and markers.
- A start pose and sampling box closer to the camera. The probes above
  suggest that distance alone is not enough at the current resolution.

## Phase 3 carry-overs

| Item | What was done |
|---|---|
| Swept volume in `PlanOnly` | `Planner.swept_bounds` uses every 10th discretized point and the last. At each one it takes the TCP (from `ComputeFk`) and the origins of the 8 arm links, then enlarges the min/max box by 0.08 m. The link origins come from the URDF chain (`motion.ArmChain`); see deviation 2. |
| Fault watcher | It clears faults only if `/clock` advanced within the last 1 s (`SimClock.advancing`). Otherwise it logs once per episode: "Isaac's /clock is not advancing; leaving the fault until it does". |
| `PlanOnlyResponse` comment | It says the duration is at the current speed and that the swept bounds are an approximation. |
| `phase3_hwm_check.py` | The `robot_service` pause (`_signal_robot_service` / `_without_robot_service`) is removed. |

Check results: hwm 7/7 and cuj 8/8 (`logs/phase4_carry_hwm_check.log`,
`logs/phase4_carry_cuj_check.log`).

## Phase 3 code-review fixes

Every finding in `code_review_part3.md` was applied with its stated fix:

1. **LINEAR replan.** A replan is accepted only if FK of the reached joints
   is within 1 mm / 0.5° of the target. Otherwise the call fails with
   `ABORTED` and the planner's message.
2. **Stop or ResetScene on a PLANNING operation.**
   `OperationTable.update` returns False once the operation is done, and
   `_run_motion` then raises `CANCELLED`. `start_arm`, `stop`, ResetScene
   and SetPayload check `op.done`.
3. **Jog** fails with `DEADLINE_EXCEEDED` if its wait times out.
4. **`JogRobot`** has a deadline (the duration plus the RPC timeout).
5. **ResetScene** holds `recover_lock` for the whole reset, and a second
   ResetScene gets `FAILED_PRECONDITION` (see deviation 3).
6. **ICON errors** keep their gRPC code.
7. **AttachObject and DetachObject** reject the robot's own objects.
8. **`d455`** is excluded from ListObjects, AttachObject and DetachObject.
9. **PlanOnly's `duration_s`** is divided by the speed factor.
10. **SetSpeed** returns `FAILED_PRECONDITION` while a motion runs.
11. **GetPointCloud** has a stride (below).
12. **`capture`** names the missing sensor (`INTERNAL`).
13. **An unknown `ToolFrame`** gives `INVALID_ARGUMENT`.
14. **The planner's `INVALID_ARGUMENT` and `NOT_FOUND`** are kept; see
    deviation 4.
15. **`start_gripper`** rejects calls during a reset (except the reset's
    own Release).
16. **The `GetStateResponse` comment** says the joints and tool pose are
    unset while faulted.
17. **The `colliding_pairs` comment** says it is the planner's
    description of the collision.
18. **`RobotService.GetOperation` and `WaitOperation`** were added (below).

Plus the plan's added decisions:

- **Stride:** `GetPointCloud(GetPointCloudRequest{stride})`. A negative
  stride gives `INVALID_ARGUMENT`, and 0 means 4, which gives 101 760
  points. `phase3_cuj_check.py` now uses the default 4 MB channel limit.
- **Operation calls:** `RobotService.GetOperation` and `WaitOperation`
  share their code with the `Operations` servicer.
  `phase3_cuj_check.py` now polls through `RobotService`.
- The stubs were regenerated.
- New unit tests: a motion cancelled while planning is done and rejects
  updates; the arm chain FK; the LINEAR replan is accepted at 0.3 mm and
  aborted at 5 cm short.

The cuj check after these fixes failed twice on `obstacle`, then passed in
the final regression. See "Open: obstacle check flakiness".

## Step 1 (decisions 1–5 and 7)

- **Decision 1.** `build_intrinsic.sh` builds `calibration_service_main`
  and `//ext/calib_poses:sample_poses`. `--test` also runs
  `//ext/calib_poses:sample_poses_test` (with `--nocheck_visibility`, like
  the `//ext` build line). `stage_intrinsic.sh` stages both. The build and
  both `//ext` tests pass (`logs/phase4_step1_bazel.log`).
- **Decision 2.** `container/config/calibration.runtime.txtpb` holds the
  plan's text, and `calibration.runtime.pb` is its encoding. The
  entrypoint starts `calibration` after `camera_sim` and before
  `robot_service` (Isaac backend). The service log says "Calibration
  service listening on 0.0.0.0:18006" about 0.5 s after start.
- **Decision 3.** `platform_stubs.py` adds
  `AssetInstances.GetAssetInstance`. For `d455` it returns
  `config.hardware_device.service.service_config` = Any(`CameraConfig`
  with ROS device id `d455`); any other name gets `NOT_FOUND` with a
  message. Each call is logged
  (`<time> INFO platform_stubs GetAssetInstance OK name=d455`). The stubs
  for `calibration_service.proto` and `asset_instances.proto` are
  generated.
- **Decision 4.** `container/config/zenohd.json5`, plus a `COPY` of
  `/libzenoh_plugin_storage_manager.so` into `/opt/zenoh/`. The entrypoint
  starts zenohd with `-c`. The zenohd log says "Successfully started plugin
  storage_manager from /opt/zenoh/libzenoh_plugin_storage_manager.so".
  `CaptureData`'s KV write completes (experiment c).
- **Decision 5.** `make_charuco.py` uses 0.04 / 0.03 m, `sim/scene.py`
  has `BOARD_SIZE_M = (0.28, 0.20, 0.005)`, and `build_catalog_meshes.py`
  makes a 0.28 × 0.20 × 0.005 m box. The PNG (1400 × 1000) is
  regenerated. The catalog was rebuilt with `build_catalog_meshes.py`,
  Isaac's `build_catalog.py` and `sync_isaac.sh --pull-catalog`. Only
  `calib_board`'s `mesh.glb` and `model.usd` changed.
- **Decision 7.** `robot_model/load_camera.py` runs in the entrypoint
  right after `load_franka`.
  - It reads the pose from `init_world`'s `d455` if there is one,
    otherwise it uses the seed.
  - It creates `d455` in `world` with `name_is_global_alias`, the
    `calibrated` BoolValue and a `sensor` frame, then clones `world` into
    `init_world`.
  - The seed is 0.0300 m and 3.000° from Isaac's true
    `Camera_Pseudo_Depth` pose.
  - Each start logs a line such as `[camera_loader] created d455 with
    frame sensor in world source=seed|init_world calibrated=… position=…
    quaternion_xyzw=…`.
- **Decision 8, CLI part.** `ext/calib_poses` implements the CLI. Its unit
  test checks 200 finite poses, the board facing the camera, distinct
  poses, determinism for the same seed, and rejection of bad input.
- **Decision 12 is also implemented**, although it belongs to Step 3; see
  deviation 9.
- **Image:** `logs/phase4_step1_docker_build.log` and
  `logs/phase4_final_docker_build.log`.

## Deviations and choices

1. **`d455` is created from a scene object, not with
   `create_single_entity_object` plus `CreateFrame`.** The world rejects
   `CreateFrame` for a frame named `sensor` ("is reserved"), and the
   calibration service needs a frame with exactly that name. So
   `load_camera` uses `create_from_scene_object` with two entities:
   - the link `d455`; its local name is what the resource-handle lookup
     needs, and experiment a confirms it works
   - the frame entity `sensor`, the child of `d455`, at identity

   The world also rejects an object without geometry ("Cannot create an
   object without any collision and visual geometry"). The link therefore
   carries a visual-only box, 0.124 × 0.029 × 0.026 m in the optical
   frame, so the camera stays out of collision checks.
2. **Swept-volume link origins come from the URDF, not from
   `ComputeFk`.** The planner's FK resolves only objects and frames, not
   link entities. `motion.ArmChain` follows the Lula URDF chain from
   `panda_link0` to `panda_link8`. The TCP still comes from `ComputeFk`.
3. **ResetScene uses a separate non-blocking `reset_lock`** to reject a
   concurrent reset, and holds `recover_lock` for the reset itself. Using
   `recover_lock` as the non-blocking lock would reject a ResetScene
   whenever the fault watcher happened to be recovering.
4. **Unreachable poses now return `NOT_FOUND`** instead of `ABORTED`. This
   follows from review finding 14: the planner reports "IK solver couldn't
   find any solutions" as `NOT_FOUND`. The planner also reports the LINEAR
   "reached joints" error as `NOT_FOUND`
   (`linear_cartesian_motion_path_planner.cc:180`), so the replan check
   runs before the kept-code rule.
5. **The encode command needs the config's proto file.** Decision 2's
   command omits it; `encode_runtime.sh` needs it for the Any type:
   `container/encode_runtime.sh container/config/calibration.runtime.txtpb container/config/calibration.runtime.pb intrinsic/perception/proto/v1/calibration_service.proto`.
6. **`sample_poses.h` holds the parser and sampler** so that the
   `cc_test` can link them; `sample_poses.cc` is only `main`. The test
   runs with `--nocheck_visibility`, like the `//ext` build line.
7. **Decision 6 does not give the sign of x.** `start_pose` uses
   x = normalize(world z × z), which makes the flange y point upward. The
   arm reaches this start pose (experiment c).
8. **The `CharucoPatternDetectionConfig` field is `charuco_pattern`.**
   Decision 11.3 writes `pattern`; the proto field is `charuco_pattern`.
   This only affects `robot/calibration.py`'s `initialize_request`, which
   experiment c uses.
9. **Decision 12 is done although Step 3 is not.** Step 1 now creates
   `d455` at every start. Under the old phase 3 rule ("calibrated if
   `d455` exists") `GetPointCloud` would always report
   `camera_calibrated: true` with the object's pose. It now returns
   `world`'s `d455` `sensor` pose and `user_data["calibrated"]`.
   Verified live: `camera_calibrated` false and the seed pose (1.33, 0,
   0.90). Nothing else from Step 3 is in place:
   - `CalibrateCamera` still returns `UNIMPLEMENTED`.
   - `CalibrationResult` in the proto is unchanged (decision 11.8 not
     applied).
   - `AttachCalibrationBoard` and `DetachCalibrationBoard` are as in
     phase 3 (decision 10 not revisited).
   - There is no `tests/test_calibration_flow.py`.

   `container/services/robot/calibration.py` holds only the decision 6, 8
   and 11.3 helpers that the experiments use. A draft of the flow and its
   wiring into `robot_service` was removed when the experiments failed.
10. **`phase3_scene_check.py`'s `restart` step** now expects `d455` in
    `world` (decision 7); `sim_world` still mirrors only `/tf`. Without
    this change the step fails on the extra object.

## Open: obstacle check flakiness

After the review fixes, `phase3_cuj_check.py` `obstacle` failed in two
runs:

- 1 of 20 executed-path samples was in collision, with `panda_link5`
  against the obstacle.
- The colliding joints were [-0.314, -0.095, -0.064, -2.205, 0.166,
  2.336, 0.681] and [-0.313, -0.097, -0.064, -2.204, 0.156, 2.336,
  0.689].
- The logs are `logs/phase4_review_cuj_check.log` and `…_check2.log`.

The check passed in the carry-over run and in the final regression, with
the same code. The planner is sampling-based: `PlanOnly` from the same
start returned between 1.01 and 1.38 s in a probe. Not fixed. The check
samples the *executed* path, so a planned path that only grazes the
obstacle can fail it.

## Not done (Step 3 and Step 4)

- **Step 3** (blocked by c and d):
  - decision 9 (sync wait)
  - decision 10 refinements
  - decision 11 (the `CalibrateCamera` flow and the 11.8 proto change)
  - `tests/test_calibration_flow.py`
- **Step 4:**
  - `tests/e2e/phase4_check.py`
  - decision 13 (design doc)

## Check output (final image)

`.venv/bin/python tests/e2e/phase3_hwm_check.py`
(`logs/phase4_final_hwm_check.log`, exit 0):

```
PASS move: Isaac joint error 0.00090 rad
PASS trajectory: Isaac final error 0.00090 rad
PASS gripper: GRASP done True width ICON 0.0000 Isaac 0.0000 m; RELEASE done True width ICON 0.0750 Isaac 0.0750 m
PASS wrench: force norm 5.753 N before, 0.000 N after taring
PASS stop: moving at 0.205 rad/s, stopped (< 0.01 rad/s) 0.020 s sim time after intrinsic.stop
PASS isaac_restart: FAULTED ('DEADLINE_EXCEEDED: BeginCycle: robot_hwm: Timeout after 20001.2 ms') 25 s after kill; ClearFaults OK after 1 attempt(s), ICON ENABLED; move: Isaac joint error 0.00090 rad
PASS epoch: /sim/reset 'epoch 1' -> FAULTED ("'robot_hwm': simulation reset"); ClearFaults OK after 1 attempt(s), ICON ENABLED
elapsed 91.5
```

`.venv/bin/python tests/e2e/phase3_scene_check.py`
(`logs/phase4_final_scene_check.log`, exit 0):

```
PASS objects: sim_world: worst 0.00 mm, sources ground_truth; world: worst 0.00 mm, sources ground_truth
PASS spawn: extra_cube in both worlds 0.2 s after Spawn, gone 2.1 s after Delete
PASS joints: world franka joints within 0.00130 rad of ICON's
PASS camera: RGB + depth 848x480, fx 420.08, stamped +0.052 s after the clock read before Capture, depth median 2.048 m; DescribeCamera [(1, 'color', [1], (848, 480)), (4, 'depth', [2], (848, 480))]
PASS restart: objects re-created 13 s after docker restart; exactly one of each in both worlds: ['franka', 'franka_hand', 'hole_block', 'peg', 'plate', 'red_cube', 'table'] (and d455 in world)
elapsed 64.4
```

`.venv/bin/python tests/e2e/phase3_cuj_check.py`
(`logs/phase4_final_cuj_check.log`, exit 0):

```
PASS logging: unreachable MoveToPose -> NOT_FOUND "operations/MoveToPose-1: planning failed: IK solver couldn't find any solutions ..."
PASS cuj2: LINEAR MoveToPose: true TCP 0.69 mm / 0.097 deg from target
PASS cuj7: MoveWaypoints 1.120 s / PlanOnly 1.114 s; Jog moved joint 1 by 0.1047 rad; ComputeFk vs tool_pose 0.0000 mm
PASS cuj4: Stop -> operations/MoveJoints-7 CANCELLED ('cancelled (Stop, CancelOperation or ResetScene)'); next Home done in 0.66 s
PASS grasp: Grasp width 50.0 mm gripping; cube rose 49.9 mm
PASS obstacle: midpoint in collision; PlanOnly 1.12 s around the obstacle; 20 samples of the executed path collision-free
PASS cuj5: fault "'robot_hwm': simulation reset"; Home 0.00 s later -> FAILED_PRECONDITION "robot faulted: 'robot_hwm': simulation reset; call ClearFaults"; after ClearFaults Home done in 0.03 s
PASS misc: SetPayload(0.5) ok; SetSpeed(0.5) 0.41 -> 0.82 s; ResetScene ok; ListObjects ['hole_block', 'peg', 'plate', 'red_cube', 'table'] ground_truth; GetImage RGB/DEPTH 848x480 PNG; GetPointCloud 101760 points, z 0.81..5.43 m
elapsed 87.3
```

`.venv/bin/python -m pytest tests/ -q`: `26 passed`.

`container/build_intrinsic.sh --test`: `//ext/calib_poses:sample_poses_test
PASSED`, `//ext/ros2_hwm:ros2_hwm_logic_test (cached) PASSED`.

## Files

- **Created:**
  - `container/config/{calibration.runtime.txtpb, calibration.runtime.pb, zenohd.json5}`
  - `container/services/robot/calibration.py`
  - `ext/calib_poses/{BUILD, sample_poses.h, sample_poses.cc, sample_poses_test.cc}`
  - `robot_model/load_camera.py`
  - `tests/e2e/exp4/{exp4_ab.py, exp4_cd.py}`
  - generated stubs under `gen/intrinsic/{assets,perception}/proto/v1/`
  - this file
- **Modified:**
  - `container/{Dockerfile, entrypoint.sh, build_intrinsic.sh, stage_intrinsic.sh}`
  - `container/services/{platform_stubs.py, robot_service.py}`
  - `container/services/robot/{icon.py, motion.py, ops.py}`
  - `robot_service/robot_service.proto` and `gen/robot_service/*`
  - `sim/scene.py`
  - `tools/{make_charuco.py, build_catalog_meshes.py}`
  - `assets/calib/charuco_7x5.png`
  - `assets/objects/calib_board/{mesh.glb, model.usd}`
  - `tests/test_robot_service.py`
  - `tests/e2e/{phase3_hwm_check.py, phase3_cuj_check.py, phase3_scene_check.py}`
- **`~/intrinsic-core`:** only the `ext` symlink and its
  `.git/info/exclude` entry; `git status` there is clean.
