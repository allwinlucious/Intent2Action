# Phase 4: camera-to-robot calibration through `CalibrationService`

Part of [robot_platform.md](../robot_platform.md) (CUJ 3, G6). Facts:
[phase_4_inventory.md](phase_4_inventory.md) ("Inv §n"). Requires phase 3
done (`camera_sim`, `scene_sync`, registry entry, `robot_service`).

## Goal and done criteria

`tests/e2e/phase4_check.py`, in this order (each step's timeout in
brackets):

| Name | Condition |
|---|---|
| `service_up` | `calibration_service_main` runs in the container on 18006 and stays up [60 s] |
| `fresh_seed` | the script deletes `d455` from `init_world`, restarts the container [180 s]; `world`'s `d455` `sensor` pose is the seed: ≥ 2 cm and ≥ 2° from Isaac's true `Camera_Pseudo_Depth` pose, `GetPointCloud.camera_calibrated` false |
| `calibrate` | `SimService.AttachCalibrationBoard`; `RobotService.CalibrateCamera(wait=false)`, polled with `Operations.GetOperation` until done [15 min]; done without error, `poses_used` ≥ 12 |
| `accuracy` | `world` `d455` `sensor` pose (robot base frame) vs Isaac's true `Camera_Pseudo_Depth` pose (`SimService.GetTruePose`): < 2 mm and < 0.2° (G6); `camera_calibrated` true |
| `persist` | `docker restart intrinsic` [180 s]; `d455` pose in `world` equals the calibrated one within 0.1 mm / 0.01°; `camera_calibrated` still true |
| `detach` | `SimService.DetachCalibrationBoard`; `calib_board` is gone from Isaac's `/tf` and from `world` [10 s] |

## Decisions

1.  **Build:** `container/build_intrinsic.sh` adds
    `//intrinsic_perception/intrinsic/perception/calibration/services/v1:calibration_service_main`
    (Inv §1.1) and `//ext/calib_poses:sample_poses` (decision 8);
    `container/stage_intrinsic.sh` stages both.
2.  **Runtime config** `container/config/calibration.runtime.txtpb`
    (Create), encoded at image build like the planner's
    (`container/encode_runtime.sh container/config/calibration.runtime.txtpb container/config/calibration.runtime.pb`):
    ```textproto
    port: 18006
    config {
      [type.googleapis.com/intrinsic_proto.perception.v1.CalibrationServiceConfig] {
        intrinsic_runtime {
          name: "intrinsic_runtime"
          interfaces { key: "grpc://google.longrunning.Operations"          value { grpc { connection { address: "localhost:18010" } } } }
          interfaces { key: "grpc://intrinsic_proto.assets.v1.AssetInstances" value { grpc { connection { address: "localhost:18010" } } } }
          interfaces { key: "grpc://intrinsic_proto.data_logger.DataLogger"    value { grpc { connection { address: "localhost:18010" } } } }
          interfaces { key: "grpc://intrinsic_proto.world.ObjectWorldService"  value { grpc { connection { address: "localhost:18000" } } } }
        }
      }
    }
    ```
    (Inv §1.3, §2: any gRPC server passes the readiness check; DataLogger
    is only needed as a key.) Started by the entrypoint after `camera_sim`,
    before `robot_service`:
    `calibration_service_main --runtime_context_file=$CFG/calibration.runtime.pb --zenoh_router=tcp/127.0.0.1:7447`.
3.  **Stand-in** (`container/services/platform_stubs.py`, Modify):
    `AssetInstances.GetAssetInstance(name="d455")` → `AssetInstance` with
    `config.hardware_device.service.service_config` =
    Any(`CameraConfig{identifier{ros{device_id: "d455"}}}`); other names →
    `NOT_FOUND`. Stubs: `container/gen_protos.sh` for
    `intrinsic/perception/proto/v1/calibration_service.proto` and
    `intrinsic/assets/proto/v1/asset_instances.proto`.
4.  **Zenoh KV storage** (`CaptureData` waits up to 30 s for its KV write to
    read back without one, Inv §11): `container/config/zenohd.json5`
    (Create): listen `tcp/127.0.0.1:7447`, multicast scouting off,
    `plugins_loading: {enabled: true, search_dirs: ["/opt/zenoh"]}`,
    plugin `storage_manager` with storage `kv` on `kv_store/**`, volume
    `memory`. `container/Dockerfile` (Modify) copies
    `/libzenoh_plugin_storage_manager.so` from the zenohd image to
    `/opt/zenoh/`; `container/entrypoint.sh` (Modify) starts zenohd with
    `-c /opt/app/container/config/zenohd.json5`.
5.  **Bigger board** so markers are detectable at 0.6–0.8 m (a 22 mm
    marker is ≈ 9 px at 1 m, Inv review): 7 × 5 squares, 40 mm squares,
    30 mm markers, `DICT_5X5_100` → 0.28 × 0.20 × 0.005 m.
    `tools/make_charuco.py` (Modify: `0.04`, `0.03`, image 1400 × 1000 →
    still 200 px per square), `sim/scene.py` (Modify: `BOARD_SIZE_M`
    (0.28, 0.20, 0.005)), `tools/build_catalog_meshes.py` (Modify:
    `calib_board` mesh 0.28 × 0.20 × 0.005). Regenerate the PNG and the
    catalog entry.
6.  **Calibration start pose:** the board's textured face is its +z (the
    hand's +z, `sim/scene.py`), and pose sampling keeps the board facing
    the hemisphere it already faces (`pose_sampling.h:76-82`). So
    calibration starts from a pose whose hand +z points at the camera:
    flange at (0.60, 0.0, 0.50) m in the base frame, its z axis along the
    unit vector from there toward the camera position stored in `world`'s
    `d455` (the seed or the last calibration, never Isaac's true pose),
    its x axis perpendicular to z with no world-z component. Reached with
    `MoveToPose` (tool `FLANGE`, `ANY`).
7.  **Camera object in the worlds** (`robot_model/load_camera.py`, Create;
    the entrypoint runs it right after the Franka loader):
    1.  Pose source: if `init_world` has an object `d455` → its pose and
        its `user_data["calibrated"]`; else the **seed** = Isaac's
        nominal camera pose from `/opt/app/config/default.yaml` converted
        to the optical frame (as `tests/e2e/phase2_check.py` does), moved
        +0.03 m along world x and rotated +3° about world z, with
        `calibrated` false.
    2.  In `world`: delete `d455` if present, then create it with
        `create_single_entity_object`, entity local name `d455` (so a
        lookup by resource handle name `d455` finds it, Inv §11),
        `name_is_global_alias` true, `user_data["calibrated"]` =
        `BoolValue`, and a child frame `sensor` at identity
        (`CreateFrame`). `sensor` is the optical frame (z forward, x right,
        y down) = Isaac's `d455_optical`.
    3.  `CloneWorld(world_id="world", cloned_world_id="init_world", allow_overwrite=true)`,
        so robot and flange IDs match in both worlds (`Save` needs this,
        Inv §8.1).
8.  **Pose sampling with Intrinsic's `pose_sampling` library** through a
    CLI (no Python bindings, Inv §9): `ext/calib_poses/BUILD`,
    `sample_poses.cc` (Create), `cc_binary` `sample_poses`, deps
    `//intrinsic_perception/intrinsic/perception/calibration:pose_sampling`.
    Input JSON on stdin:
    `{"base_t_flange": P, "flange_t_board": P, "base_t_camera": P, "halfsize": [x,y,z], "tilt_deg": d, "roll_deg": d, "count": n, "seed": s}`
    with `P = {"position": [x,y,z], "quaternion_xyzw": [x,y,z,w]}`; output
    `{"poses": [P, …]}` (each a sampled `base_t_flange`), exit 1 with a
    message on bad input. Values used: start pose (decision 6),
    `flange_t_board` = rotation Rz(−π/4) then translation (0, 0, 0.12)
    in that rotated frame (board fixed 0.12 m along Isaac's `panda_hand`
    z), `base_t_camera` = `world`'s `d455` `sensor` pose, halfsize
    (0.08, 0.12, 0.08), tilt 15°, roll 20°, count 200, seed 0. Unit test
    `ext/calib_poses/sample_poses_test.cc` (Create, `cc_test`): 200 poses,
    all finite, board z of every pose has a positive dot product with the
    direction to the camera, poses differ.
9.  **Robot joints in `world`** come from the world service's
    `WorldUpdater` (phase 3 decision 14). `CaptureData` is called without
    `base_t_flange`; before each capture `robot_service` waits 0.2 s of sim
    time, then until `world`'s `franka` joints equal ICON's sensed joints
    within 1e-4 rad (poll `GetObject` every 20 ms, up to 2 s; on timeout
    skip the pose and log it).
10. **`AttachCalibrationBoard`** (`robot_service`, Modify):
    `SimLink.AttachBoard`; wait until `calib_board` is in `world` (≤ 5 s,
    `scene_sync` creates it); `ReparentObject` it under `franka`'s
    `flange`; `SimLink.SetAttached(calib_board, true)`.
    **`DetachCalibrationBoard`:** `SimLink.SetAttached(calib_board, false)`,
    `SimLink.DetachBoard`; `scene_sync` deletes it when it leaves `/tf`.
11. **Calibration flow** (`robot_service.CalibrateCamera`, replaces the
    phase 3 `UNIMPLEMENTED`; operation states `PLANNING` → `EXECUTING` →
    `DONE`/`FAILED`, `detail` = "capture k/20: <status>"):
    1.  Precondition: `calib_board` attached (decision 10) → else
        `FAILED_PRECONDITION` "calibration board not attached".
    2.  Move to the start pose (decision 6); failure → `ABORTED` with the
        planner's message.
    3.  `Initialize`: `pattern_detection_config{name "charuco",
        charuco_pattern_detection_config{pattern{squares_x 7, squares_y 5,
        square_length 0.04, marker_length 0.03, dictionary DICT_5X5_100}},
        publish_annotated_image true}`; one camera handle
        `{name "d455", connection_info.grpc.address "localhost:18005",
        resource_data["CameraConfig"] = Any(CameraConfig{identifier ros d455})}`;
        `arm_part` = `franka` by name.
    4.  Sample poses (decision 8); keep, in order, those with a
        collision-free `ComputeIk` and a successful `PlanTrajectory`, until
        20 are kept.
    5.  For each kept pose: `MoveToPose` (tool `FLANGE`); on failure skip
        and log. Sync wait (decision 9). `CaptureData`; count `NO_ERRORS`.
    6.  Fewer than 12 `NO_ERRORS` captures → `ABORTED`
        "only k board detections (need 12)".
    7.  `Calibrate{calibrate_camera_to_robot: true,
        camera_to_robot_calibration_type: CAMERA_SETUP_STATIONARY}`;
        `Save(result)`; set `user_data["calibrated"] = true` on `d455` in
        `world` and `init_world` (`UpdateObjectProperties`); `Home`.
    8.  Response `CalibrationResult` — `robot_service/robot_service.proto`
        (Modify): replace `reprojection_error_px` with
        `rms_translation_m` and `rms_rotation_deg` (from the service's
        camera-to-robot result) and keep `poses_used` = number of
        `NO_ERRORS` captures.
12. **`GetPointCloud.camera_calibrated`** (phase 3 decision 26, Modify):
    `user_data["calibrated"]` of `world`'s `d455`; `camera_pose` = its
    `sensor` pose.
13. **Design doc** (`robot_platform.md`, Modify): "Calibration flow" and
    CUJ 3 per decisions 5–11 (`Save` writes only `init_world`, Inv §12);
    "Ground truth and belief": the camera pose in `world` is a nominal
    seed flagged `calibrated: false` until calibration, never Isaac's true
    pose.

## Steps

1.  Decisions 1–5 and 7 (build, configs, stand-in, zenohd, board, camera
    loader); rebuild the image.
2.  **Experiments** (`tests/e2e/exp4/`, Create), results into
    `phase_4_notes.md`:
    a.  `GetObject(resource_handle_name="d455")` on `world` finds the
        decision 7 object.
    b.  `franka`'s object ID is the same in `world` and `init_world` after
        the clone.
    c.  Board attached, arm at the decision 6 start pose: `Initialize` +
        one `CaptureData` completes in < 5 s with `NO_ERRORS`.
    d.  Detection rate: 20 sampled poses (decision 8) → ≥ 16 `NO_ERRORS`.
    If any fails, stop and record it.
3.  Decisions 8–12; unit tests `tests/test_calibration_flow.py` (Create:
    the flow with fake services: precondition, skip on move failure,
    `ABORTED` below 12).
4.  `tests/e2e/phase4_check.py` (Create) until all checks pass; then
    decision 13; write `phase_4_notes.md`.

## Constraints

As phase 3: Bazel only via `container/build_intrinsic.sh`; no `git
commit`; Docker via `sg docker -c`; no `pkill -f`; do not modify
`~/intrinsic-core` except the `ext` symlink and `.git/info/exclude`.
Follow the design's Observability requirement for every new call.

## Phase 3 carry-overs (do first)

Apply the two items under "Part 3 resolution" in
`docs/designs/phases/phase_3_notes.md` (swept volume in `PlanOnly`; fault
watcher only while `/clock` advances), update the `PlanOnlyResponse`
comment in `robot_service/robot_service.proto`, remove the
`robot_service` pause from `tests/e2e/phase3_hwm_check.py`, and rerun
`phase3_hwm_check.py` and `phase3_cuj_check.py` (all pass).

## Phase 3 code-review fixes (do first, after the carry-overs)

Apply every finding in `docs/designs/reviews/phase_3/code_review_part3.md`
with its stated fix, plus these decisions:
*   `GetPointCloud` takes `GetPointCloudRequest{int32 stride = 1}` (pixel
    stride, default 4 when 0 → ≤ 101 760 points, < 1.3 MB); RPC comment
    says so.
*   `RobotService` gains `GetOperation(google.longrunning.GetOperationRequest)`
    and `WaitOperation(google.longrunning.WaitOperationRequest)` (both
    return `google.longrunning.Operation`), delegating to the operations
    table, so the generated MCP tools can poll `wait=false` calls.
*   Proto comments that become MCP descriptions: `GetStateResponse` says
    joints/tool_pose are unset while faulted; `colliding_pairs` says it is
    the planner's description of the collision.
Regenerate stubs; extend `tests/test_robot_service.py` for the operation
table's cancel/`done` semantics and the LINEAR replan acceptance check;
rerun `phase3_cuj_check.py` (all pass).
