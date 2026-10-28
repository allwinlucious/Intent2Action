# Phase 5b: force control (`MoveToContact`, `MoveCompliant`, CUJ 6)

Part of [robot_platform.md](../robot_platform.md) (G8, CUJ 6). Facts:
[phase_5b_inventory.md](phase_5b_inventory.md) ("Inv §n"). `TareWrench`
exists (phase 3). Runs after phase 5c (shares Isaac and the container).

Geometry (`tools/build_catalog_meshes.py`, `sim/scenes/default.yaml`):
`hole_block` at (0.60, 0.30, 0), top face z = 0.05, hole radius 0.011,
depth 0.04 (bottom z = 0.01); `peg` radius 0.010, length 0.08.

## Goal and done criteria

`tests/e2e/phase5b_check.py`; `ResetScene` before every check and every
trial. "Force along the push" = |projection of `GetWrench` force onto the
push direction expressed in the flange frame| (`GetWrench` reports the
flange frame, Inv §4.2; the sign is not asserted).

| Name | Condition |
|---|---|
| `contact_table` | gripper closed (`Grasp` with nothing), TCP 3 cm above the table at (0.45, −0.05, 0.03), tool pointing down; `TareWrench`; `MoveToContact((0,0,−1), 15, 0.01)` → done; force along the push 12–18 N (G8 ±20 %) |
| `no_contact` | TCP at (0.45, −0.05, 0.30); `MoveToContact((0,0,1), 10, 0.02)` → `ABORTED` "no contact within 0.15 m" |
| `bad_speed` | `MoveToContact((0,0,−1), 15, 0.2)` → `INVALID_ARGUMENT` naming the achievable speed range |
| `insert` | CUJ 6, 10 trials: grasp `peg` (phase 5c flow), `AttachObject("peg")`, `SetPayload(0.05)`; place the peg tip 2 cm above the block top, **3 mm off the hole axis in +x** (the test computes the TCP target from `SimService.GetTruePose("peg")` and the current TCP, so the offset is exact); `TareWrench`; `MoveToContact((0,0,−1), 15, 0.01)` → contact on the block's top face, force 12–18 N; `MoveCompliant(target = contact TCP pose moved to the hole axis in x/y and 0.03 m lower in z, stiffness 200, max_force 20)` → `done`; success = Isaac's true peg bottom ≥ 0.025 m below the top face, no ICON fault. ≥ 9/10 (G8) |
| `jam` | gripper closed, TCP 2 cm above the table at (0.45, −0.05, 0.02); `TareWrench`; `MoveCompliant(target 0.03 m below the table surface, 200, 20)` → `ABORTED` "settled before reaching the target" within 10 s sim |
| `logs` | each force call has a `robot_service.log` line with status, duration and final force along the push |

CUJ 6 in the design is updated to the off-axis start (decision 9).

## Decisions

1.  **Protos:** `container/gen_protos.sh intrinsic/icon/actions/force_primitive.proto intrinsic/icon/actions/cartesian_admittance_action.proto`
    (closure into `gen/`, committed). Plain protos; Intrinsic's Python
    helpers are not used (unbuilt C++ extensions, Inv §0.2).
2.  **Pure helpers** in `container/services/robot/force.py` (Create; no
    `rclpy`, no `_session` import):
    *   `contact_inertia(force, speed) -> float` =
        (force / (2·speed))² / (1234 + 60000) (Inv §1.4).
    *   `validate_contact(direction, force, speed)`: |direction| > 0,
        0 < force ≤ 85, 0 < speed; inertia must lie in [1, 200] kg, else
        `INVALID_ARGUMENT` "speed must be between v_min and v_max m/s for
        F N" (v from the inverse formula); returns the unit direction.
    *   `validate_compliant(stiffness, max_force)`: stiffness in
        [50, 5000], 0 < max_force ≤ 85.
    *   `make_contact_params(direction, force, inertia, flange_t_tcp)` →
        `ForcePrimitiveFixedParams` (two variants: approach, and stabilize
        with `disable_reference_lowpass_filter`), with `MakeContact`
        (`motion_direction` = direction, `max_contact_force` = force),
        `task_params` (`robot_base_t_task` identity,
        `robot_tip_t_robot_tool` = flange→`tcp`, `environment_stiffness`
        MEDIUM), `force_control_settings` (virtual translational inertia
        = inertia, rotational 1.0, deadband 1.5 N / 0.2 N·m).
    *   `admittance_params(target, stiffness, flange_t_tcp)` →
        `CartesianAdmittanceActionFixedParams`: reference pose = target
        (TCP), stiffness diag(k,k,k,30,30,30), inertia diag(2,2,2,0.05,
        0.05,0.05), damping unset (critical), `tool_reference_wrench`
        zero (pure spring toward the target), deadband 1.5 N / 0.2 N·m.
    *   `push_force(wrench_flange, direction_base, base_R_flange) -> float`.
3.  **`Icon.run_sequence`** (`container/services/robot/icon.py`, Modify):
    ```python
    run_sequence(
        parts: list[str],                       # session parts, e.g. ["arm", "ft_sensor"]
        actions: list[tuple[int, str, Message, dict[str, str] | str]],
            # (id, type_name, params, slot->part map or single part)
        transitions: list[tuple[int, Condition, int]],   # from, when, to
        aborts: list[tuple[list[int], Condition, str]],  # watched action ids, when, message
        success: tuple[int, Condition],                  # action id, when
        stop_id: int,                                    # the intrinsic.stop action in `actions`
        timeout_s: float, clock: Callable[[], float],    # sim-time timeout
        cancel: Callable[[], bool],
    ) -> SequenceResult(outcome: "ok" | message | "timeout" | "cancelled", status: GetStatus)
    ```
    Each transition is `add_transition`. Each abort becomes, on every
    watched action, `add_reactions(action, Reaction(cond,
    [Event(flag_i), StartActionInRealTime(stop_id)]))`. Success is a
    reaction raising `flag_ok` that also starts `stop_id`. The function
    waits (polling flags every 20 ms wall, like `run_action`) until a flag
    is raised, then until `stop_id`'s `is_settled` (max 2 s sim), then ends
    the session. Abort flags win over `flag_ok` if both are set. On
    timeout/cancel it starts `stop_id` itself, waits as above and returns
    `"timeout"`/`"cancelled"`.
4.  **`MoveToContact`** (`container/services/robot_service.py`, Modify;
    an operation via `Robot.start_arm` and `reply(wait)` like `MoveToPose`):
    validate (decision 2); actions (1) `intrinsic.tare_force_torque_sensor`
    on `ft_sensor`, (2) approach `intrinsic.force_primitive` and (3)
    stabilize `intrinsic.force_primitive` with slots
    `{"arm": "arm", "ft_sensor": "ft_sensor"}`, (4) `intrinsic.stop` on
    `arm`. Transitions: 1→2 on `is_done`; 2→3 when
    `elapsed_time_seconds` > 0.05 and `intrinsic.sensed_force` > switch,
    switch = max(0.5·F, 3); 3→4 when (`sensed_force` > switch and
    `intrinsic.is_settled`) or `elapsed_time_seconds` > 2. Success: 4
    `is_settled` (no stop reaction needed: 4 is the stop). Aborts on
    actions 2 and 3: `intrinsic.translational_distance_traveled` > 0.15 →
    "no contact within 0.15 m"; `sensed_force` > 85 → "excessive force".
    Timeout 60 s sim → `DEADLINE_EXCEEDED`; abort → `ABORTED` with the
    message. Response `MotionResult`; operation detail and the log line
    give the final force along the push.
5.  **`MoveCompliant`** (same file): validate; actions (1)
    `intrinsic.cartesian_admittance_action` (slots as above), (2)
    `intrinsic.stop` on `arm`. Success: 1 `intrinsic.is_done`. Aborts on
    1: `sensed_force` > max_force → "force limit reached"; 
    `intrinsic.settled_for_seconds` > 1.0 → "settled before reaching the
    target (position error e m)", e from `intrinsic.reference_position_error`
    in the final status. No tare inside (taring in contact would zero the
    contact force); the RPC comment says it uses the existing tare.
    Timeout 30 s sim.
6.  **Payload to the F/T compensation** (Inv §0.7): `Icon.set_payload`
    (Modify) sets the arm's `full_payload` **and** the `ft_sensor` part's
    `grasped_payload` (same mass and centre of mass); mass 0 clears both.
    Every caller (`SetPayload` RPC, `recover()`, `_reset()`) goes through
    it.
7.  **Contact accuracy in Isaac:** `sim/tools/build_catalog.py` (Modify)
    applies `PhysxCollisionAPI` `contactOffset` 0.001 and `restOffset`
    0.0 to every catalog object; regenerate all `assets/objects/*/model.usd`.
8.  **Proto comments** (`robot_service/robot_service.proto`, Modify):
    `MoveToContact` "tares, moves along the direction at about `speed`,
    stops when the contact force reaches `max_force`"; `MoveCompliant`
    "moves toward the TCP pose as a spring of `stiffness`, using the
    current tare; ends at the target, at `max_force`, or when it stops
    moving (jam)".
9.  **Design doc** (`robot_platform.md`, Modify): CUJ 6 — peg starts
    3 mm off the hole axis, contact on the top face, `MoveCompliant` to
    the axis 3 cm lower; G8 references the checks above.

## Steps

1.  Decisions 1, 2, 6, 7; unit tests `tests/test_force.py` (Create):
    `contact_inertia(15, 0.01)` ≈ 9.19; validation of direction 0,
    force 0/90, speed outside the range (message contains both bounds);
    `push_force` for a downward push with the tool pointing down.
2.  Decisions 3, 4, 5, 8; rebuild the image.
3.  `tests/e2e/phase5b_check.py` (Create) until all checks pass. If
    `insert` is below 9/10, record per trial: contact force, x/y error at
    contact, final peg depth, `MoveCompliant` outcome, in
    `phase_5b_notes.md`, and stop (the lead decides parameters).
4.  Decision 9; phases 3, 4 and 5c checks and `pytest` still pass; write
    `phase_5b_notes.md` (Create).

## Constraints

As phase 3 (no `git commit`; Docker via `sg docker -c`; no `pkill -f`;
Bazel only via `container/build_intrinsic.sh`; Observability). Files:
`container/services/robot/force.py` (Create), `container/services/robot/icon.py`,
`container/services/robot_service.py`, `robot_service/robot_service.proto`,
`sim/tools/build_catalog.py`, `assets/objects/*/model.usd`,
`docs/designs/robot_platform.md` (Modify), `gen/…` (regenerated),
`tests/test_force.py`, `tests/e2e/phase5b_check.py`,
`docs/designs/phases/phase_5b_notes.md` (Create).

## Addendum A (after run 2, phase_5b_notes.md findings 1–5)

1.  **Compliant contact in Isaac** (finding 1): `sim/tools/build_catalog.py`
    (Modify) gives every catalog object a `UsdPhysics.MaterialAPI` +
    `PhysxMaterialAPI` material with `compliantContactStiffness` 60000
    (N/m, MakeContact's MEDIUM model) and `compliantContactDamping` 600,
    friction static/dynamic 0.5/0.4; regenerate the USDs. Keep the 85 N
    excessive-force abort.
2.  **CUJ 6 becomes search-then-insert** (findings 2–3): after
    `MoveToContact((0,0,−1), 15, 0.01)` on the top face (peg 3 mm off
    axis), `MoveCompliant(target = contact TCP pose moved to the hole
    axis in x/y and 0.005 m lower in z, stiffness 1000, max_force 30)` →
    done (normal force ≈ 5 N, lateral pull ≈ 3 N); then
    `MoveToContact((0,0,−1), 15, 0.01)` → contact on the hole bottom.
    Success = peg bottom ≥ 0.025 m below the top face, no fault; ≥ 9/10.
    Update `tests/e2e/phase5b_check.py` and CUJ 6 in `robot_platform.md`
    (Step 4) accordingly.
3.  **`jam`** (finding 4): gripper closed, TCP 2 cm above the table;
    `TareWrench`; `MoveToContact((0,0,−1), 10, 0.01)`; then
    `MoveCompliant(target 0.03 m below the contact TCP pose, stiffness
    1000, max_force 40)` → `ABORTED` "settled before reaching the target"
    (spring ≈ 30 N < 40).
4.  **Checks poll** (finding 5): long calls in `phase5b_check.py` use
    `wait=false` + `WaitOperation` (timeout 120 s).
Then Steps 3–4.

## Addendum B (after run 3, phase_5b_notes.md findings 6–9)

1.  **Joint velocity from positions** (finding: phantom joint-7 velocity
    under contact while the position holds to 1e-5 rad): `ext/ros2_hwm`
    (Modify) writes `joint_velocity_state` = finite difference of
    consecutive `/joint_states` positions over their stamp difference,
    low-pass filtered (v ← 0.5·v + 0.5·v_fd); Isaac's reported velocity is
    no longer used. Reset the filter on an epoch change. Logic in
    `ros2_hwm_logic` with a unit test (constant position → 0; ramp →
    slope). Rebuild with `container/build_intrinsic.sh` (+ `--test`).
2.  **Search geometry:** sliding needs offset > μ·Δz regardless of
    stiffness. `MoveCompliant` search target = hole axis in x/y, **0.002 m**
    below the contact TCP z, stiffness **2000**, max_force 30 (normal
    ≈ 4 N, lateral ≈ 6 N at 3 mm). `peg` and `hole_block` friction
    static/dynamic **0.3/0.25** (`sim/tools/build_catalog.py` per-object
    material from `object.yaml` keys `friction_static`, `friction_dynamic`;
    default 0.5/0.4 for others) → stall offset ≈ 0.6 mm < 1 mm clearance.
    Then `MoveToContact((0,0,−1), 15, 0.01)` to the hole bottom as before.
3.  **`jam`:** `MoveCompliant(target 0.03 m below contact, stiffness 1000,
    max_force 60)` (spring 30 N plus the measured overshoot ≈ 43 N stays
    below 60) → `ABORTED` "settled before reaching the target".
4.  After the check passes, rerun the phase 3, 4 and 5c checks (the new
    contact materials and velocity source affect them) and do Step 4.

## Addendum C (after run 4: grasp slip, wrench chatter)

1.  **Grip** (peg slipped and pivoted ~8–10° in the fingers):
    *   `sim/scene.py` (Modify): a physics material on the Franka finger
        collision prims (`/World/franka/panda_leftfinger`,
        `/World/franka/panda_rightfinger`): friction static/dynamic
        1.2/1.0 with `frictionCombineMode = "max"` (PhysX gives `max`
        priority over `average`, so finger contacts use 1.2 while
        peg–hole stays at the peg/hole average).
    *   `container/config/icon.txtpb`: gripper `default_force` 30 → 60 N.
    *   `container/services/robot/grasp.py` depth (phase 5c decision 6.6):
        d = min(0.04, extent along the approach − 0.01) (was 0.02):
        shorter lever arm.
    *   Rerun `phase5c_check.py` after this (cube and peg ≥ 9/10).
2.  **Chatter sweep** (wrench alternating 22/57 N while still): for each
    combination in this order — admittance translational inertia
    (`MoveCompliant`, decision 2) ∈ {2, 5, 10} kg (outer loop) × catalog
    `compliantContactDamping` ∈ {600, 2000} (inner loop; regenerate
    USDs) — run `jam` and 5 `insert` trials. Choose the **first**
    combination where `jam` passes and ≥ 4/5 inserts succeed; then run
    the full check (≥ 9/10). Record every combination's results (peak
    and alternating wrench, insert depth) in `phase_5b_notes.md`. If none
    qualifies, stop and record.
3.  Then Step 4 and the phase 3, 4, 5c regression checks.

## Addendum D (after run 5)

Revert the grasp depth to phase 5c's rule `d = min(0.02, extent along the
approach − 0.01)` (`MAX_DEPTH_M` back to 0.02; restore the unit test
expectations). Keep the finger friction material and the 60 N grasp
force from Addendum C.1. Then rerun `phase5c_check.py` (≥ 9/10 each) and
continue with Addendum C.2 (the sweep), C.3 and Step 4 unchanged.

## Addendum E (after run 6, the sweep)

1.  **Chosen parameters:** `MoveCompliant` translational virtual inertia
    **5 kg**; catalog `compliantContactDamping` **2000** (sweep: jam aside,
    5/5 inserts, searches < 11 s). Set them in `container/services/robot/force.py`
    and `sim/tools/build_catalog.py`; regenerate the USDs.
2.  **`jam` uses a realistic jam** (bare fingers on the table chatter
    0.5↔30 N in Isaac; the peg on the block face does not): grasp `peg`,
    `AttachObject`, `SetPayload(0.05)`; peg tip 2 cm above the block top
    at 0.02 m from the hole axis (fully on the face); `TareWrench`;
    `MoveToContact((0,0,−1), 10, 0.01)`; `MoveCompliant(target 0.03 m
    below the contact TCP pose, stiffness 1000, max_force 60)` →
    `ABORTED` "settled before reaching the target".
3.  Then the full `phase5b_check.py` (all criteria, `insert` ≥ 9/10), C.3
    (phase 3, 4, 5c regressions) and Step 4.
