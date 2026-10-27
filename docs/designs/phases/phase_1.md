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
| `fk_20` | for 20 random joint vectors (`random.Random(1)`, each joint `uniform(lower, upper)`), `ComputeFk` of frame `flange` (object `franka`) and frame `tcp` (object `franka_hand`), both relative to object `franka`, match Isaac's Lula FK of `panda_link8` and `right_gripper` within 1 mm and 0.1° (G3) |
| `fk_reference` | the four reference rows of Inv §6.2 (`panda_link8` and `right_gripper` rows for `default_q` and "ready") match within 1 mm / 0.1° (no Isaac needed) |
| `ik` | `ComputeIk` for the `flange` pose given by `ComputeFk("ready")`, starting from `default_q`, returns a solution whose `ComputeFk` is within 0.1 mm / 0.01° of the target |
| `plan` | `PlanTrajectory` from `default_q` to "ready" (joint target) returns OK with a non-empty `discretized` trajectory |
| `icon_status` | `IconApi.GetStatus` returns OK (carried from phase 0) |
| `icon_parts` | `IconApi.ListParts` returns OK and contains `arm` (carried from phase 0) |
| `icon_move` | a point-to-point action on part `arm` to "ready" finishes within 30 s; afterwards `GetStatus` reports the arm's sensed joint positions within 0.001 rad of "ready" |
| `icon_trajectory` | after `icon_move` (arm at "ready"): `PlanTrajectory` from "ready" to `default_q`; its `discretized` trajectory runs with `intrinsic.trajectory_tracking` on `arm` and finishes; final sensed positions within 0.001 rad of `default_q` |
| `idempotent` | run with `--after-restart` after `docker restart intrinsic`: waits up to 180 s for ICON's port, asserts `ListObjects` has exactly one `franka` and one `franka_hand`, then runs every other check except `fk_20` |

`fk_20` needs Isaac's Python (Windows); the other checks need only the
container.

*   G3 = design doc goal G3 (FK of Isaac and Intrinsic agree within 1 mm / 0.1°).
*   `icon_status`/`icon_parts` failed in phase 0 only because no robot was in the world (phase_0_notes.md).
*   The loopback HWM starts at all zeros, which is outside `panda_joint4`'s range; ICON allows motion back into range, so `icon_move` from zeros is expected to work.

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
    *   `franka_hand`: one link entity `panda_hand` holding the hand and
        both finger meshes as fixed geometry (fingers at the open position,
        0.04 m each; Lula holds them at 0.025, which affects geometry only,
        not FK). It is created as a child of `franka`'s `flange` **at
        identity**: the world forces the pose of an object attached to an
        attachment frame to identity (`physical_object.cc:236-247`), so the
        URDF's `panda_hand_joint` rotation (rpy 0 0 −π/4) is built into the
        hand's shapes instead: each shape's `ref_t_shape` = Rz(−π/4) ·
        (URDF mesh origin). It has a Frame entity `tcp` = URDF
        `right_gripper` relative to `panda_link8`: translation (0, 0, 0.1),
        rotation Rz(3π/4) (Inv §5.2). Finger motion is phase 3.
    *   **Collision exclusion:** the hand mesh overlaps `panda_link7`'s by
        about 2.6 cm at the flange, and the world only excludes adjacent
        links inside one object. After creating the hand, the loader calls
        `ObjectWorldService.ToggleCollisions` with `toggle_mode` disabling
        collisions, `object_a` = `franka_hand` (entity filter
        `include_all_entities: true`), `object_b` = `franka` (entity filter
        `entity_names: ["panda_link7"]`).
*   **Dropped URDF frames:** all `camera_*` (wrist camera; our D455 is fixed
    in the scene), `panda_face_*`, forearm/wrist points, fingertips.
*   **Limits** (application velocity, acceleration and jerk = 0.95 × system; ICON requires ≥ 5% margin): position, velocity and
    effort from the URDF; acceleration
    `[15, 7.5, 10, 12.5, 15, 20, 20]` rad/s² and jerk
    `[7500, 3750, 5000, 6250, 7500, 10000, 10000]` rad/s³ from Isaac's Lula
    description (Inv §5.5).
*   **Cartesian limits** (chosen values; velocity, acceleration and jerk
    are Franka's commonly published Panda Cartesian limits, not re-verified
    against a datasheet). Each translational field is a 3-vector (x, y, z),
    and every `min_` must be ≤ 0:
    `min_translational_position = [-2, -2, -2]`, `max_translational_position = [2, 2, 2]`,
    `min_translational_velocity = [-1.7]*3`, `max_translational_velocity = [1.7]*3`,
    `min_translational_acceleration = [-13]*3`, `max_translational_acceleration = [13]*3`,
    `min_translational_jerk = [-6500]*3`, `max_translational_jerk = [6500]*3`,
    `max_rotational_velocity = 2.5`, `max_rotational_acceleration = 25`,
    `max_rotational_jerk = 12500`.
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
With `$MG` = `/mnt/c/isaac-sim/exts/isaacsim.robot_motion.motion_generation`
and `$FD` = `/mnt/c/isaac-sim/exts/isaacsim.asset.importer.urdf/data/urdf/robots/franka_description`:
*   `lula_franka_gen.urdf` ← `$MG/motion_policy_configs/franka/lula_franka_gen.urdf`
*   `robot_descriptor.yaml` ← `$MG/motion_policy_configs/franka/rmpflow/robot_descriptor.yaml`
*   `meshes/collision/*.stl` ← `$FD/meshes/collision/*.stl`
*   `meshes/visual/*.glb`: generated by file 2 from `$FD/meshes/visual/*.dae`
*   `LICENSE.md`: source paths above; meshes and `package.xml` of
    `franka_description`: Apache 2.0; `robot_descriptor.yaml`: Apache-2.0
    SPDX header; the URDF (generated by NVIDIA from the Apache-2.0
    `franka_description` xacro) has no header of its own.

### 2. `tools/convert_franka_meshes.py` (Create)

Converts each `.dae` in `$FD/meshes/visual/` to
`assets/robots/franka/meshes/visual/<name>.glb` with `trimesh` (load with
`trimesh.load(path, force="scene")`, export `file_type="glb"`). Add
`trimesh` and `pycollada` to the venv with pip (dev-time only). Check after
conversion: each GLB's bounding box equals the DAE's within 1e-6 m; print
both. Run once; leave the GLBs untracked (the lead commits them).

### 3. `robot_model/franka_scene_object.py` (Create)

Library: Python 3.10-compatible (the container's Python), standard
library + `grpcio` + `protobuf` + `gen/` stubs only. Imported everywhere as
`robot_model.franka_scene_object` (the container sets
`PYTHONPATH=/opt/app:/opt/app/gen`; tests get the repo root and `gen/` from
`tests/conftest.py`). Functions:

*   `parse_urdf(path) -> Urdf`: `xml.etree.ElementTree`; links (name,
    visual and collision mesh file names with origin), joints (name, type,
    parent, child, origin xyz/rpy, axis, limit lower/upper/effort/velocity).
*   `rpy_to_quaternion(r, p, y) -> (x, y, z, w)` (URDF fixed-axis XYZ
    convention: R = Rz(y)·Ry(p)·Rx(r)).
*   `build_arm(urdf, geometry_refs, lula_limits) -> SceneObject` and
    `build_hand(urdf, geometry_refs) -> SceneObject`, implementing the
    decisions above with these proto details:
    *   Entities: one `Link` entity per link (`geometry_component.named_geometries`
        with sets `"Intrinsic_Collision"` and `"Intrinsic_Visual"`; inner map
        key = the link name on arm links, and `hand`, `left_finger`,
        `right_finger` on the hand link; value = v1
        `TransformedGeometry{geometry.geo_ref, ref_t_shape}` where
        `ref_t_shape` is a `GeometricTransform` with
        `trs { translation, rotation_rpy }` taken from the URDF
        visual/collision `<origin xyz rpy>`). One `Joint` entity per
        revolute joint (`kinematics_component`: `parent_t_inboard` = URDF
        joint origin, `outboard_t_child` = identity, `motion_type`
        REVOLUTE, `axis` (0,0,1), `system_limits` and `application_limits`
        as decided). Entity tree link0 → joint1 → link1 → … → joint7 →
        link7, then a FIXED joint entity `panda_joint8` (`parent_t_inboard`
        = translation z 0.107, `outboard_t_child` = identity, `motion_type`
        FIXED, no axis, no limits) → link `panda_link8` (no geometry), then
        a Frame entity `flange` (parent `panda_link8`, identity,
        `is_attachment_frame: true`). Entity proto:
        `intrinsic_apis/intrinsic/scene/proto/v1/entity.proto` (Inv §1.5).
    *   `properties.kinematics`: `ik_solvers` = one solver `kinematic_chain`,
        `named_configurations` = `home`, `limits` = the Cartesian limits.
    *   Hand: single Link entity `panda_hand` (the object's root, at the
        flange) with geometry `hand`, `left_finger` (URDF finger joint
        origin z 0.0584 + opening 0.04 m along +y) and `right_finger`
        (opening along −y, plus the right finger's `rpy 0 0 π` mesh origin,
        Inv §5.1), every shape pre-multiplied by Rz(−π/4) as decided above,
        and a Frame entity `tcp` (parent `panda_hand`, translation
        (0, 0, 0.1), rotation Rz(3π/4)).
*   `upload_meshes(geometry_stub, asset_dir) -> dict[str, GeometryStorageRefs]`:
    `CreateGeometry` with `stl_bytes` (collision) or `gltf_bytes`
    (visual); key = path relative to `meshes/` (e.g. `collision/link1.stl`,
    `visual/link1.glb`). The builder maps the URDF's
    `package://franka_description/meshes/visual/<n>.dae` to `visual/<n>.glb`
    and `.../collision/<n>.stl` to `collision/<n>.stl`.

### 4. `robot_model/load_franka.py` (Create)

CLI: `--world=localhost:18000 --geometry=localhost:18001
--assets=/opt/app/assets/robots/franka`. Steps: wait up to 60 s for both
channels; upload meshes; build both SceneObjects; if `franka_hand` exists
delete it, if `franka` exists delete it (`DeleteObject` with `force=true`);
`CreateObject` `franka` with `name_is_global_alias: true` (required for
every by-name lookup: planner, ICON's `HalArmPart`, the hand's parent),
`parent_object { reference { id: "root" } entity_filter { include_final_entity: true } }`
(as the SDK does, `object_world_client.py:1369-1372`), identity pose;
`CreateObject` `franka_hand` with `name_is_global_alias: true`,
`parent_object { reference { by_name { object_name: "franka" } } entity_filter { entity_names: "flange" } }`,
identity pose; then `ToggleCollisions` as decided above. Exit 0 on success,
1 with the gRPC error otherwise. Acceleration and jerk are read from
`robot_descriptor.yaml` (`acceleration_limits`, `jerk_limits`; parse the
two lines with the standard library, no YAML dependency).

### 5. `container/config/icon.txtpb` (Modify)

Replace with the arm configuration of Inv §4.1 verbatim (world + geometry
interfaces at `localhost:18000/18001`, `kinematics_from_world_service:
true`, `allow_non_real_time_inverse_kinematics: true`, part `arm`,
`HalArmPart`, `hardware_resource_name: "franka"`, the four loopback
interfaces).

### 6. `container/entrypoint.sh` (Modify)

After the planner starts and before the HWM, run in the foreground
(the loader itself waits for world and geometry):
`python3 -m robot_model.load_franka || { echo "[loader] failed" >&2; exit 1; }`.

### 7. `container/Dockerfile` (Modify)

Add `COPY robot_model /opt/app/robot_model` and
`COPY assets/robots /opt/app/assets/robots`. Set
`ENV PYTHONPATH=/opt/app:/opt/app/gen`.

### 8. Protos (Run `container/gen_protos.sh`)

`intrinsic/geometry/proto/geometry_service.proto`,
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
angle of `R_a^T R_b`. With `--after-restart` the script runs the `idempotent` check as defined in the table; without it, every check except `idempotent`.

### 11. `tests/conftest.py` (Create)

Inserts the repo root and `gen/` at the front of `sys.path`.

### 12. `container/services/platform_stubs.py` (Modify)

Replace the `shortcut:` comment on `ResourceRegistry` with: `# shortcut:
no resource instances; ICON reads its config file and does not need one.
Add instances if a service starts requiring them.`

### 13. Documentation (Modify)

*   `container/README.md`: add `robot_model/` and the loader step.
*   `docs/designs/robot_platform.md`, section "Franka model": replace steps
    2–4 with "a Python builder (`robot_model/`) creates the SceneObjects
    `franka` and `franka_hand` from the URDF and uploads meshes with
    `GeometryService.CreateGeometry`; see phase 1"; replace step 1's
    "the phase 1 plan lists which frames are kept" with "phase 1 lists the
    dropped frames"; in the phase table, phase 1's row: "Franka in
    Intrinsic: SceneObjects from the URDF, loader, ICON with loopback";
    add Alternative N (after M) "SDF + Bazel `sdf_scene_object`" with the
    reason from this plan.

## Steps

1.  Files 1 and 2 (convert meshes), 8 (protos).
2.  `.venv/bin/pip install pytest absl-py numpy`. Files 3, 4, 11; unit
    test `tests/test_franka_scene_object.py` (Create):
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
    now), then `.venv/bin/python tests/e2e/phase1_check.py`.
5.  `sg docker -c "docker restart intrinsic"`, then
    `.venv/bin/python tests/e2e/phase1_check.py --after-restart`.
6.  If a check fails and the fix lies outside this plan's files, stop and
    write the error into `docs/designs/phases/phase_1_notes.md`.
7.  Stop the container. Write `docs/designs/phases/phase_1_notes.md`
    (Create): outputs of both test scripts, the worst FK errors from
    `fk_20`, deviations.
