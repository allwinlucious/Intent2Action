# Phase 1 implementation notes

Status: **stopped on a plan defect.** All world and planner checks pass,
including `fk_20` (G3). The four ICON checks fail because ICON rejects the
plan's decision "system limits = application limits". Fixing it means
choosing new limit values, which is a design decision, so the worker
stopped there and did not choose them (see "Blocking issue").

## Blocking issue: ICON needs application limits ≤ 95 % of system limits

ICON refuses to build part `arm`:

```
FAILED_PRECONDITION: Provided joint limits are inconsistent. The application
velocity limits must be at least 5% less than the system velocity limits.
Limit check returns violated=[vaj]
Application velocity limits: 2.175 ×4, 2.61 ×3
System velocity limits:      2.175 ×4, 2.61 ×3
; Error building Part 'arm'.
```

*   Source: `intrinsic_control/intrinsic/icon/control/parts/feature_interfaces/joint_limits.cc:101-143`
    (`JointLimitsFeature::Create`) checks application ≤
    `GetMaxAdmissibleApplicationLimits(system)`, with
    `kMaxApplicationLimitsMultiplier = 0.95`
    (`joint_limits_constants.h:25`). It checks velocity, acceleration and
    jerk separately (`violated=[vaj]` means all three fail).
*   The planner accepts the object but warns and clamps
    (`kinematics_component.cc:369-371`: "Setting application velocity limit
    above 0.950000 percent … Clamping -- this will be an error in the
    future"). So the planner already plans with 95 % of the limits.
*   Where the values are set: `robot_model/franka_scene_object.py`,
    `build_arm` → `_joint_limits` (the same `Limits` is copied into
    `system_limits` and `application_limits`).
*   For reference, UR sets application limits to 95 % of system
    (phase_1_inventory.md §1.4). Which side to scale (application = 0.95 ×
    URDF/Lula, or system = URDF/Lula / 0.95) is the lead's call.
*   Position limits (`fixed_limits`) are not part of this check.

## Test output (2026-10-27)

`tests/e2e/phase0_smoke.py`, after the loader ran (8 of 10; `icon_status`
and `icon_parts` fail on the issue above, ICON error text shortened):

```
PASS zenohd
PASS zenoh_sessions
PASS localcas
PASS stubs
PASS geometry
PASS world
PASS planner
PASS hwm
FAIL icon_status: FAILED_PRECONDITION: Provided joint limits are inconsistent. The application velocity limits must be at least 5% less than the system velocity limits. Limit check returns violated=[vaj] … ; Error building Part 'arm'.
FAIL icon_parts: FAILED_PRECONDITION: (same)
elapsed 0.7
```

`tests/e2e/phase1_check.py` (ICON error text shortened, same as above):

```
PASS objects
PASS fk_20: worst panda_link8 0.000000 mm / 0.000002 deg, right_gripper 0.000000 mm / 0.000002 deg
PASS fk_reference
PASS ik
PASS plan
FAIL icon_status: FAILED_PRECONDITION: Provided joint limits are inconsistent. … ; Error building Part 'arm'.
FAIL icon_parts: FAILED_PRECONDITION: (same)
FAIL icon_move: FAILED_PRECONDITION: (same)
FAIL icon_trajectory: FAILED_PRECONDITION: (same)
elapsed 1.8
```

`sg docker -c "docker restart intrinsic"`, then
`tests/e2e/phase1_check.py --after-restart`. The first run came about
8 s after the restart:

```
PASS idempotent
PASS idempotent/objects
FAIL idempotent/fk_reference: UNAVAILABLE: failed to connect to all addresses; … 127.0.0.1:18002: … Connection refused
FAIL idempotent/ik: UNAVAILABLE: (same)
FAIL idempotent/plan: UNAVAILABLE: (same)
FAIL idempotent/icon_status: FAILED_PRECONDITION: Provided joint limits are inconsistent. …
FAIL idempotent/icon_parts: FAILED_PRECONDITION: (same)
FAIL idempotent/icon_move: FAILED_PRECONDITION: (same)
FAIL idempotent/icon_trajectory: UNAVAILABLE: (same, planner)
elapsed 8.0
```

Second run, about 30 s after the restart:

```
PASS idempotent
PASS idempotent/objects
PASS idempotent/fk_reference
PASS idempotent/ik
PASS idempotent/plan
FAIL idempotent/icon_status: FAILED_PRECONDITION: Provided joint limits are inconsistent. …
FAIL idempotent/icon_parts: FAILED_PRECONDITION: (same)
FAIL idempotent/icon_move: FAILED_PRECONDITION: (same)
FAIL idempotent/icon_trajectory: FAILED_PRECONDITION: (same)
elapsed 0.3
```

The loader's log after the restart shows delete-then-create working:
`[loader] deleted existing franka_hand`, `[loader] deleted existing franka`,
`[loader] created franka and franka_hand`.

`.venv/bin/python -m pytest tests/`: 5 passed.

## FK (G3)

*   `fk_20` (20 vectors, `random.Random(1)`, uniform in the URDF limits):
    worst error for `flange` against `panda_link8` and for `tcp` against
    `right_gripper` is below 5e-10 m (prints as 0.000000 mm) and
    0.000002°.
*   `fk_reference`: all four Inv §6.2 rows match within 1 mm / 0.1°.
*   `ik`: `kinematic_chain` finds the "ready" flange pose from `default_q`
    within 0.1 mm / 0.01°.

## Deviations from the plan

*   **`tools/convert_franka_meshes.py` drops zero-area triangles.** The
    first loader run failed with `INTERNAL: Unsupported number of vertices
    in a face`, on `visual/link7.glb` only (each mesh was uploaded on its
    own to find it). Cause: the geometry service reads meshes with assimp's
    `aiProcess_FindDegenerates`
    (`intrinsic_sdk/intrinsic/geometry/internal/mesh/io/load_ai_scene_from_buffer.cc:57-59`),
    which turns degenerate triangles into lines or points, and
    `AiSceneToMesh` rejects any face that is not a triangle
    (`ai_scene_to_mesh.cc:61-62`). The converter now removes faces that
    trimesh's `nondegenerate_faces()` flags (link0: 5, link4: 1, link6: 1,
    link7: 5; others 0) and keeps the DAE's vertex normals. The bounding-box
    check is still run against the unmodified DAE and passes for all 10
    meshes. The fix is inside a plan file and does not change the geometry.
*   **The `idempotent` wait covers only ICON's port.** After a restart,
    ICON's port opens about 8 s in, while the planner listens on 18002 only
    about 11 s in (it pre-caches the world's geometry first), so the first
    `--after-restart` run fails the planner checks with `UNAVAILABLE`. The
    script follows the plan (it waits for ICON only); the outputs above
    show both runs.
*   `CreateGeometry` and the world RPCs in the loader use a 30 s deadline
    (the plan names none).
*   Joint entities get `parent_t_this` = `parent_t_inboard`, and links under
    a joint get identity. This matches the SDF converter
    (`joint_conversion.cc:65-67`, `scene_object_from_sdf.cc:245-246`); the
    world recomputes joint poses anyway (`RefreshDofParentTThis`,
    `world_object_internal.cc:1905-1913`).
*   Revolute axes, the `panda_joint8` offset, the hand rotation, finger
    origins and the `tcp` pose are read from the URDF rather than typed
    in. The values equal the plan's.
*   `phase1_check.py` prints `idempotent` as one check, then the other
    checks as `idempotent/<name>`.
*   Added packages in `.venv`: `trimesh`, `pycollada`, `pytest`,
    `absl-py`, `numpy` (plus their dependencies).
*   Image rebuilt twice (the mesh fix). `container/stage/` is unchanged.
    The container is stopped.

## Not done (blocked)

*   `icon_status`, `icon_parts`, `icon_move`, `icon_trajectory`: never got
    past part creation, so point-to-point and trajectory tracking on the
    loopback arm are untested. Also untested: whether a session needs an
    explicit `Enable` first (the script does not call it).

## Resolution (lead)

*   Application limits for velocity, acceleration and jerk set to 95% of
    the system limits (`APPLICATION_LIMIT_FRACTION` in
    `robot_model/franka_scene_object.py`), as Intrinsic's UR definitions
    do; positions unchanged.
*   `--after-restart` now waits for world, geometry, planner and ICON ports.
*   Result: `phase0_smoke.py` 10/10, `phase1_check.py` 9/9 (fk_20 worst
    error 0.000000 mm / 0.000002°), `--after-restart` 9/9.
*   Review code review of the phase 1 diff: pending (after WSL restart).
