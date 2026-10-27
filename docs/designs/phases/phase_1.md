# Phase 1: the Franka in Intrinsic (world, planner, ICON on loopback)

Part of [robot_platform.md](../robot_platform.md). Facts with source
citations are in [phase_1_inventory.md](phase_1_inventory.md) ("Inv §n").
Phase 0 results and carried items: [phase_0_notes.md](phase_0_notes.md).

## Goal and done criteria

`tests/e2e/phase1_check.py` passes against the running container. Checks
(name: condition):

| Name | Condition |
|---|---|
| `objects` | `ObjectWorldService.ListObjects(world_id="world")` contains `franka` and `franka_hand` |
| `fk_20` | for 20 random joint vectors (seed 1, uniform within the joint limits), `ComputeFk` of frame `flange` (object `franka`) and frame `tcp` (object `franka_hand`), both relative to object `franka`, match Isaac's Lula FK of `panda_link8` and `right_gripper` within 1 mm and 0.1° (G3) |
| `fk_reference` | the three reference rows of Inv §6.2 (`panda_link8` and `right_gripper` rows for `default_q` and "ready") match within 1 mm / 0.1° (no Isaac needed) |
| `ik` | `ComputeIk` for the `flange` pose at "ready", starting from `default_q`, returns a solution whose `ComputeFk` is within 0.1 mm / 0.01° of the target |
| `plan` | `PlanTrajectory` from `default_q` to "ready" (joint target) returns OK with a non-empty `discretized` trajectory |
| `icon_status` | `IconApi.GetStatus` returns OK (carried from phase 0) |
| `icon_parts` | `IconApi.ListParts` returns OK and contains `arm` (carried from phase 0) |
| `icon_move` | a point-to-point action on part `arm` to "ready" finishes within 30 s; afterwards `GetStatus` reports the arm's sensed joint positions within 0.001 rad of "ready" |
| `icon_trajectory` | after `icon_move` (arm at "ready"): `PlanTrajectory` from "ready" to `default_q`; its `discretized` trajectory runs with `intrinsic.trajectory_tracking` on `arm` and finishes; final sensed positions within 0.001 rad of `default_q` |
| `idempotent` | restarting the container (`docker restart intrinsic`) and running the checks again passes (the loader replaces existing objects) |

`fk_20` needs Isaac's Python (Windows); the other checks need only the
container.

Not in this phase: gripper motion (phase 3), Isaac connection (phases 0.5,
2), `robot_service` (phase 3).

## Decisions made in this plan

*   **The SceneObject is built in Python straight from the URDF, without
    SDF or Bazel** (Inv §1.6 path A). Reason: the Bazel route produces a
    `.gzf` whose geometry must be unpacked, uploaded to CAS and re-referenced,
    which only Intrinsic's Go tools do (Inv §1.5); the Python route uploads
    meshes with `GeometryService.CreateGeometry` (Inv §2.2) and uses the
    same URDF Isaac uses, so there is one source of truth (G3). The design
    doc's "Franka model" section is updated to say this.
*   **Two objects**, following Intrinsic's arm + end-effector split:
    *   `franka`: `panda_link0` … `panda_link8`, joints `panda_joint1` … `panda_joint7`,
        and an attachment frame `flange` at `panda_link8` (identity). One
        leaf (`panda_link8`), so Cartesian planning and IK work (Inv §1.7).
        Root entity = `panda_link0` (URDF `base_link` is an empty link with
        an identity fixed joint to `panda_link0`, Inv §5.1, so poses are
        identical).
    *   `franka_hand`: one link entity holding the hand and both finger
        meshes as fixed geometry (fingers at the open position, 0.04 m
        each), created as a child of `franka`'s `flange`, with the URDF's
        `panda_hand_joint` transform (rpy 0 0 −π/4). It has a frame `tcp` =
        URDF `right_gripper` (0.1 m along `panda_link8` z, rpy 0 0 3π/4
        relative to `panda_link8`, Inv §5.2), expressed relative to the
        hand link. Finger motion is phase 3.
*   **Dropped URDF frames:** all `camera_*` (wrist camera; our D455 is fixed
    in the scene), `panda_face_*`, forearm/wrist points, fingertips.
*   **Limits** (system limits = application limits): position, velocity and
    effort from the URDF; acceleration
    `[15, 7.5, 10, 12.5, 15, 20, 20]` rad/s² and jerk
    `[7500, 3750, 5000, 6250, 7500, 10000, 10000]` rad/s³ from Isaac's Lula
    description (Inv §5.5).
*   **Cartesian limits** (Franka Panda datasheet): translational position
    ±2 m (each axis), translational velocity 1.7 m/s, acceleration 13 m/s²,
    jerk 6500 m/s³; rotational velocity 2.5 rad/s, acceleration 25 rad/s²,
    jerk 12500 rad/s³.
*   **Named configuration** `home` = "ready" `[0, -0.785, 0, -2.356, 0, 1.571, 0.785]`.
*   **IK solver** `kinematic_chain` (only solver for 7 DoF, Inv §3.3).
*   **Meshes:** collision = the URDF's STL files (uploaded as `stl_bytes`);
    visual = the URDF's DAE files converted once to GLB (uploaded as
    `gltf_bytes`, because `CreateGeometry` has no DAE input, Inv §2.2).
*   **Where loading runs:** inside the container, at start-up, after world
    and geometry are up and before the HWM and ICON start, so ICON's main
    loop sees the robot (Inv §4.3). Loading is delete-then-create, so
    restarts with the persisted world volume work (phase 0 notes).

## Constraints

*   Do not modify `/home/mew/intrinsic-core`. Do not run Bazel.
*   Do not run `git commit` (the lead commits).
*   Docker via `sg docker -c "<command>"`. Never `pkill -f` or `killall`.
*   Project Python: `/home/mew/untitled-project/.venv/bin/python`. New
    Python files: `pyink --pyink-indentation=2 --line-length=80`,
    `isort --profile=google`.
*   Isaac's Python is run as
    `cat script.py | cmd.exe /c "cd /d C:\isaac-sim && python.bat -"` (Inv
    §6.2); Lula needs no SimulationApp.

## Target files

### 1. `assets/robots/franka/` (Create)

Copied from Isaac (licence Apache 2.0, from `package.xml`):
*   `lula_franka_gen.urdf` ← `/mnt/c/isaac-sim/exts/isaacsim.robot_motion.motion_generation/motion_policy_configs/franka/lula_franka_gen.urdf`
*   `robot_descriptor.yaml` ← `.../franka/rmpflow/robot_descriptor.yaml`
*   `meshes/collision/*.stl` ← `/mnt/c/isaac-sim/exts/isaacsim.asset.importer.urdf/data/urdf/robots/franka_description/meshes/collision/`
*   `meshes/visual/*.glb`: generated by file 2 from `.../meshes/visual/*.dae`
*   `LICENSE.md`: one paragraph: source paths above, licence Apache 2.0.

### 2. `tools/convert_franka_meshes.py` (Create)

Converts each `.dae` in the Isaac visual mesh directory to
`assets/robots/franka/meshes/visual/<name>.glb` with `trimesh` (load with
`trimesh.load(path, force="scene")`, export `file_type="glb"`). Add
`trimesh` and `pycollada` to the venv with pip (dev-time only). Check after
conversion: each GLB's bounding box equals the DAE's within 1e-6 m; print
both. Run once; commit the GLBs (the lead commits).

### 3. `robot_model/franka_scene_object.py` (Create)

Library, standard library + `gen/` stubs only (it runs in the container).
Functions:

*   `parse_urdf(path) -> Urdf`: `xml.etree.ElementTree`; links (name,
    visual and collision mesh file names with origin), joints (name, type,
    parent, child, origin xyz/rpy, axis, limit lower/upper/effort/velocity).
*   `rpy_to_quaternion(r, p, y) -> (x, y, z, w)` (URDF fixed-axis XYZ
    convention: R = Rz(y)·Ry(p)·Rx(r)).
*   `build_arm(urdf, geometry_refs, lula_limits) -> SceneObject` and
    `build_hand(urdf, geometry_refs) -> SceneObject`, implementing the
    decisions above with these proto details:
    *   Entities: one `Link` entity per link (`geometry_component.named_geometries`
        with sets `"Intrinsic_Collision"` and `"Intrinsic_Visual"`, each a
        v1 `TransformedGeometry{geometry.geo_ref, ref_t_shape = URDF
        visual/collision origin}`), one `Joint` entity per joint
        (`kinematics_component`: `parent_t_inboard` = URDF joint origin,
        `outboard_t_child` = identity, `motion_type` REVOLUTE, `axis`
        (0,0,1), `system_limits` and `application_limits` as decided),
        entity tree link0 → joint1 → link1 → … → joint7 → link7, then a
        FIXED joint entity `panda_joint8` (origin z 0.107) → link `panda_link8`
        (no geometry), then a Frame entity `flange` (parent `panda_link8`,
        identity, `is_attachment_frame: true`). Entity proto:
        `intrinsic_apis/intrinsic/scene/proto/v1/entity.proto` (Inv §1.5).
    *   `properties.kinematics`: `ik_solvers` = one solver `kinematic_chain`,
        `named_configurations` = `home`, `limits` = the Cartesian limits.
    *   Hand: single Link entity `panda_hand` with geometry for hand, left
        finger (URDF finger origin + finger opening +0.04 m along +y) and
        right finger (opening along −y, plus the right finger's `rpy 0 0 π`
        mesh origin, Inv §5.1), and a Frame entity `tcp` (pose = transform
        from `panda_hand` to `right_gripper`: `panda_hand` is
        `panda_link8`·Rz(−π/4), `right_gripper` is `panda_link8`·(z 0.1,
        Rz(3π/4)), so `tcp` = (0, 0, 0.1) with Rz(π) relative to
        `panda_hand`).
*   `upload_meshes(geometry_stub, asset_dir) -> dict[str, GeometryStorageRefs]`:
    `CreateGeometry` with `stl_bytes` (collision) or `gltf_bytes`
    (visual); key = mesh file name.

### 4. `robot_model/load_franka.py` (Create)

CLI: `--world=localhost:18000 --geometry=localhost:18001
--assets=/opt/app/assets/robots/franka`. Steps: wait up to 60 s for both
channels; upload meshes; build both SceneObjects; if `franka_hand` exists
delete it, if `franka` exists delete it (`DeleteObject` with `force=true`);
`CreateObject` `franka` under root at identity; `CreateObject`
`franka_hand` with `parent_object` = `franka` entity filter on frame
`flange` (`ObjectReferenceWithEntityFilter`, Inv §2.1) and
`parent_object_t_created_object` = Rz(−π/4). Exit 0 on success, 1 with the
gRPC error otherwise. Acceleration and jerk are read from
`robot_descriptor.yaml` (`acceleration_limits`, `jerk_limits`; parse the
two lines with the standard library, no YAML dependency).

### 5. `container/config/icon.txtpb` (Modify)

Replace with the arm configuration of Inv §4.1 verbatim (world + geometry
interfaces at `localhost:18000/18001`, `kinematics_from_world_service:
true`, `allow_non_real_time_inverse_kinematics: true`, part `arm`,
`HalArmPart`, `hardware_resource_name: "franka"`, the four loopback
interfaces).

### 6. `container/entrypoint.sh` (Modify)

After the planner starts and before the HWM: wait for world and geometry,
then `python3 /opt/app/robot_model/load_franka.py` (run in the foreground;
if it exits non-zero, print `[loader] failed` and `exit 1`).

### 7. `container/Dockerfile` (Modify)

Add `COPY robot_model /opt/app/robot_model` and
`COPY assets/robots /opt/app/assets/robots`. Set
`ENV PYTHONPATH=/opt/app/gen`.

### 8. Protos (Run `container/gen_protos.sh`)

`intrinsic/geometry/proto/geometry_service.proto`,
`intrinsic/scene/proto/v1/scene_object.proto`,
`intrinsic/icon/actions/point_to_point_move.proto`,
`intrinsic/icon/actions/trajectory_tracking_action.proto`,
`google/rpc/code.proto` (Inv §2.2, §4.5). The generated files stay in
`gen/`.

### 9. `tests/e2e/isaac_lula_fk.py` (Create)

Runs in Isaac's Python (stdin pipe, Inv §6.2). Reads a JSON list of joint
vectors from the file `C:\isaac-sim-work\fk_in.json`, writes
`C:\isaac-sim-work\fk_out.json`: per vector, translation and row-major
rotation of `panda_link8` and `right_gripper` (Lula, URDF and descriptor
from the Isaac paths in file 1). The Windows directory is
`/mnt/c/isaac-sim-work/` from WSL (create it).

### 10. `tests/e2e/phase1_check.py` (Create)

Same structure as `phase0_smoke.py` (one function per check name, 5 s gRPC
deadlines except `icon_move`/`icon_trajectory` 60 s, `PASS`/`FAIL` lines,
exit code). Uses the gen stubs and, for ICON sessions,
`_session.Session` from `/home/mew/intrinsic-core/intrinsic_control/intrinsic/icon/python/`
(add `/home/mew/intrinsic-core/intrinsic_control` to `sys.path` after
`gen/`; install `absl-py` and `numpy` into the venv; Inv §4.5) with
`point_to_point_move_utils` and `trajectory_tracking_action_utils` from
`intrinsic_control/intrinsic/icon/actions/`. `fk_20` writes the joint
vectors, runs file 9 through `cmd.exe`, and compares. Rotation error =
angle of `R_a^T R_b`. The `idempotent` check is run by the script's
`--after-restart` flag (the lead runs it after `docker restart`); the
other checks run by default.

### 11. Documentation (Modify)

*   `container/README.md`: add `robot_model/` and the loader step.
*   `docs/designs/robot_platform.md`, section "Franka model": replace steps
    2–4 with "a Python builder (`robot_model/`) creates the SceneObjects
    `franka` and `franka_hand` from the URDF and uploads meshes with
    `GeometryService.CreateGeometry`; see phase 1"; add Alternative N
    "SDF + Bazel `sdf_scene_object`" with the reason from this plan.

## Steps

1.  Files 1 and 2 (convert meshes), 8 (protos).
2.  Files 3, 4; unit test `tests/test_franka_scene_object.py`:
    `rpy_to_quaternion` against three hand-computed cases; `parse_urdf`
    finds 7 revolute + 2 prismatic joints with the limits in Inv §5.1;
    `build_arm` produces exactly 7 REVOLUTE joints, one `flange` attachment
    frame, a `home` configuration with 7 entries.
    `.venv/bin/python -m pytest tests/` passes.
3.  Files 5, 6, 7; `container/stage_intrinsic.sh` is not needed again
    (binaries unchanged); rebuild the image
    (`sg docker -c "docker build -f container/Dockerfile -t intrinsic-core:dev ."`).
4.  `mkdir -p logs && container/run.sh > logs/phase1_run.log 2>&1 &`,
    then `.venv/bin/python tests/e2e/phase0_smoke.py` (all ten must pass
    now), then `tests/e2e/phase1_check.py`.
5.  `sg docker -c "docker restart intrinsic"`, then
    `tests/e2e/phase1_check.py --after-restart`.
6.  If a check fails and the fix lies outside this plan's files, stop and
    write the error into `docs/designs/phases/phase_1_notes.md`.
7.  Stop the container. Write `docs/designs/phases/phase_1_notes.md`
    (Create): outputs of both test scripts, the worst FK errors from
    `fk_20`, deviations.
