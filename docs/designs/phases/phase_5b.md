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
