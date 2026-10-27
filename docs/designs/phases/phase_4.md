# Phase 4: camera-to-robot calibration through `CalibrationService`

Part of [robot_platform.md](../robot_platform.md) (CUJ 3, G6). Facts:
[phase_4_inventory.md](phase_4_inventory.md) ("Inv §n"). Requires phase 3
done (`camera_sim`, `scene_sync`, registry entry, `robot_service`).

## Goal and done criteria

`tests/e2e/phase4_check.py`:

| Name | Condition |
|---|---|
| `service_up` | `calibration_service_main` runs in the container (port 18006) and stays up |
| `calibrate` | `SimService.AttachCalibrationBoard`, then `RobotService.CalibrateCamera(wait=true)` succeeds with ≥ 12 captures used |
| `accuracy` | `world` object `d455` frame `sensor` pose (robot base frame) vs Isaac's true `Camera_Pseudo_Depth` pose (`SimService.GetTruePose`): < 2 mm and < 0.2° (G6) |
| `initial_guess` | before calibrating, the `d455` pose in `world` was ≥ 2 cm and ≥ 2° away from the truth (so the check measures calibration, not the seed) |
| `persist` | after `docker restart intrinsic`, the `d455` pose in `world` equals the calibrated one within 0.1 mm |
| `detach` | `SimService.DetachCalibrationBoard` removes the board from Isaac and from `world` |

## Decisions

1.  **Build:** `container/build_intrinsic.sh` adds
    `//intrinsic_perception/intrinsic/perception/calibration/services/v1:calibration_service_main`
    (Inv §1.1); `container/stage_intrinsic.sh` stages it.
2.  **Runtime config** `container/config/calibration.runtime.txtpb`
    (encoded with `container/encode_runtime.sh`, as the planner's): `port:
    18006`, `config` = `CalibrationServiceConfig{intrinsic_runtime{name
    "intrinsic_runtime", interfaces: Operations → localhost:18010,
    AssetInstances → localhost:18010, DataLogger → localhost:18010,
    ObjectWorldService → localhost:18000}}` (Inv §1.3, §2: any server
    passes the readiness check; DataLogger is never really called). Flags:
    `--runtime_context_file`, `--zenoh_router=tcp/127.0.0.1:7447`. Started
    after the world service, before `robot_service`.
3.  **Stand-ins** (`container/services/platform_stubs.py`, Modify): add
    `AssetInstances.GetAssetInstance`: for name `d455` return an
    `AssetInstance` whose `config.hardware_device.service.service_config`
    = Any(`CameraConfig{identifier{ros{device_id: "d455"}}}`); other
    names → `NOT_FOUND` (Inv §11).
4.  **Zenoh KV storage:** `CaptureData` writes the KV store and waits for
    the value to read back (Inv §11). zenohd is started with a config file
    `container/config/zenohd.json5` (Create): listen
    `tcp/127.0.0.1:7447`, multicast scouting off, plugin
    `storage_manager` with one storage `kv` on key expression
    `kv_store/**`, volume `memory`. The Dockerfile copies
    `/libzenoh_plugin_storage_manager.so` from the zenohd image to
    `/opt/zenoh/` (Inv §11), and the config sets
    `plugins_loading: { enabled: true, search_dirs: ["/opt/zenoh"] }`.
5.  **Camera object in the worlds** (`robot_model/load_camera.py`, Create,
    run by the entrypoint right after the Franka loader):
    *   `world`: object `d455` created with `create_single_entity_object`
        whose entity local name is `d455` (so `GetObject` by resource
        handle name `d455` matches it through the entity-local-name rule,
        Inv §11), `name_is_global_alias` true, plus a child frame `sensor`
        at identity (`CreateFrame`). `sensor` is the camera's optical frame
        (z forward, x right, y down), the frame `d455_optical` images use.
    *   Pose: if `init_world` already has `d455` (a saved calibration),
        copy its pose; otherwise the seed pose = Isaac's nominal camera
        pose from `sim/scenes/default.yaml` (converted to the optical frame
        as `tests/e2e/phase2_check.py` does) offset by +3 cm in x and +3°
        about z (the `initial_guess` check).
    *   `init_world`: `CloneWorld(world_id="world", cloned_world_id="init_world", allow_overwrite=true)`
        after `d455` exists, so the robot and flange have the same IDs in
        both worlds (`Save` needs this, Inv §8.1).
6.  **Robot joints in `world`** come from the world service's
    `WorldUpdater` (phase 3 decision 14). `CaptureData` is called without
    `base_t_flange`; before each capture `robot_service` waits until
    `world`'s `franka` joints equal ICON's sensed joints within 1e-5 rad
    (poll `GetObject` every 20 ms, timeout 2 s), because the service reads
    the flange pose from `world` (Inv §5).
7.  **Pose sampling uses Intrinsic's `pose_sampling` library** through a
    small C++ CLI (no Python bindings exist, Inv §9):
    `ext/calib_poses/sample_poses.cc` (`cc_binary`
    `//ext/calib_poses:sample_poses`, deps
    `//intrinsic_perception/intrinsic/perception/calibration:pose_sampling`),
    args: JSON on stdin `{base_t_flange, flange_t_board, base_t_camera,
    halfsize: [0.15, 0.15, 0.10], tilt_deg: 20, roll_deg: 30, count: 200,
    seed}` → JSON list of `base_t_flange` poses on stdout. Staged into the
    image. `flange_t_board` = translation (0, 0, 0.12) rotated by
    Rz(−π/4) (board fixed to Isaac's `panda_hand` 0.12 m along its z,
    `sim/scene.py`).
8.  **Calibration flow** in `robot_service.CalibrateCamera` (replaces the
    phase 3 `UNIMPLEMENTED`):
    1.  Precondition: `calib_board` exists in `world` and is attached to
        the flange (`SimService.AttachCalibrationBoard` does
        `SimLink.AttachBoard`, then reparents `calib_board` under
        `franka`'s `flange` in `world` with `ReparentObject`, and marks it
        attached in `scene_sync`). Else `FAILED_PRECONDITION`.
    2.  `Initialize`: `pattern_detection_config{name "charuco",
        charuco_pattern_detection_config{pattern{squares_x 7, squares_y 5,
        square_length 0.03, marker_length 0.022, dictionary DICT_5X5_100}},
        publish_annotated_image true}`, one camera handle
        `{name "d455", connection_info.grpc.address "localhost:18005",
        resource_data["CameraConfig"] = Any(CameraConfig{identifier ros d455})}`,
        `arm_part` = `franka` by name; worlds default (`world`,
        `init_world`).
    3.  Sample 200 poses (decision 7) around the current pose; keep those
        with a collision-free `ComputeIk` and a successful `PlanTrajectory`,
        in sampled order, until 20 are kept.
    4.  For each kept pose: `MoveToPose` (tool `FLANGE`), wait for the
        world sync (decision 6) and 0.2 s of sim time, `CaptureData`.
        Count captures with status `NO_ERRORS`; fewer than 12 →
        `ABORTED` ("too few board detections").
    5.  `Calibrate{calibrate_camera_to_robot: true,
        camera_to_robot_calibration_type: CAMERA_SETUP_STATIONARY}`, then
        `Save` with the result, then `Home`.
    6.  Operation response `CalibrationResult{camera_pose, reprojection_error_px, poses_used}`
        (`reprojection_error_px` from the service's result if it reports
        one, else 0).
9.  **`DetachCalibrationBoard`:** `SimLink.DetachBoard`; `scene_sync`
    then deletes `calib_board` from the worlds when it leaves `/tf`.
10. **Design doc:** "Calibration flow" and CUJ 3 are updated to match
    decisions 5–9 (`Save` writes only `init_world`, Inv §12).

## Step 0 (experiments, before the code)

Each with a short script under `tests/e2e/exp4/` (Create); record results
in `phase_4_notes.md`:
1.  zenohd with decision 4's config: the calibration service's
    `Initialize` then one `CaptureData` (board attached, arm at home)
    completes in < 5 s (no 30 s KV timeout).
2.  `GetObject(resource_handle_name="d455")` on `world` finds the object
    of decision 5.
3.  `CloneWorld(world → init_world)` keeps object IDs (compare `franka`'s
    ID in both).
If one fails, stop and record it (the plan needs a new decision).

## Constraints

As phase 3 (Bazel only via `build_intrinsic.sh`; no `git commit`; Docker
via `sg docker -c`; no `pkill -f`; do not modify `~/intrinsic-core`
except the `ext` symlink).
