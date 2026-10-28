# Phase 5b: force control (`MoveToContact`, `MoveCompliant`, CUJ 6)

Part of [robot_platform.md](../robot_platform.md) (G8, CUJ 6). Facts:
[phase_5b_inventory.md](phase_5b_inventory.md) ("Inv §n"). `TareWrench`
exists (phase 3). Runs after phase 5c (shares Isaac and the container).

## Goal and done criteria

`tests/e2e/phase5b_check.py`:

| Name | Condition |
|---|---|
| `contact_table` | TCP 3 cm above a free table spot (0.45, −0.05, 0.03); `TareWrench`; `MoveToContact(direction (0,0,−1), max_force 15, speed 0.01)` → done; `GetWrench` force along −z between 12 and 18 N (G8 ±20 %) |
| `no_contact` | from 0.3 m above the table, `MoveToContact(direction (0,0,1), 10, 0.02)` (upward, nothing there) → `ABORTED` "no contact within 0.15 m" |
| `insert` | CUJ 6, 10 trials (`ResetScene` before each): grasp `peg` (phase 5c flow), `AttachObject`, `SetPayload(0.05)`, move the peg 2 cm above the hole axis, `TareWrench`, `MoveToContact((0,0,−1), 15, 0.01)` → force 12–18 N, then `MoveCompliant(target = 3 cm lower, stiffness 200, max_force 20)`; success = Isaac's true peg bottom ≥ 25 mm below the hole block's top face and no ICON fault. ≥ 9/10 (G8) |
| `jam` | `MoveCompliant` to a target 3 cm below the table surface (peg blocked by the table) → `ABORTED` "settled before reaching the target" within 10 s sim time |
| `logs` | each force call has a `robot_service.log` line with status, duration and final force |

## Decisions

1.  **Protos:** generate `intrinsic/icon/actions/force_primitive.proto`
    and `intrinsic/icon/actions/cartesian_admittance_action.proto` (and
    their import closure) with `container/gen_protos.sh`. Parameters are
    built as plain protos; Intrinsic's Python helpers are not used (they
    need unbuilt C++ extensions, Inv §0.2). ICON computes damping itself
    for `force_primitive` (Inv §1.3).
2.  **Sessions with sequences and reactions:** `container/services/robot/icon.py`
    (Modify) gains `run_sequence(parts, actions, transitions, end)`: one
    session on `["arm", "ft_sensor"]`; `actions` = list of (id, type,
    params); `transitions` = list of (from_id, condition, to_id);
    `end` = list of (action_id, condition, outcome) where outcome is
    `"ok"` or an error message; built with `_session.Session.add_actions`,
    `add_transition`, `add_reaction` and the `reactions` helpers
    (`is_greater_than`, `is_true`, `all_of`, `any_of`, Inv §1.6). It
    returns the outcome and the last `GetStatus` wrench; overall timeout
    in sim time; cancel via the operation table.
3.  **`MoveToContact(direction, max_force, speed)`** — the sequence of
    Intrinsic's `move_to_contact` skill (Inv §5.1), on `force_primitive`
    `MakeContact`:
    *   Validate: `|direction|` > 0 (normalised), 0 < `max_force` ≤ 85 N,
        0 < `speed` ≤ 0.05 m/s, else `INVALID_ARGUMENT`.
    *   Parameters: `motion_direction` = direction (base frame,
        `robot_base_t_task` = identity), `max_contact_force` = F,
        `robot_tip_t_robot_tool` = flange→`tcp`, `environment_stiffness`
        MEDIUM (60 000 N/m), virtual translational inertia
        m = (F / (2 v))² / (1234 + 60000), clamped to [1, 200] kg (Inv
        §1.4), rotational inertia 1.0, deadband 1.5 N / 0.2 N·m.
    *   Actions: (1) `intrinsic.tare_force_torque_sensor`; (2) approach =
        `force_primitive`; (3) stabilize = `force_primitive` with
        `disable_reference_lowpass_filter`; (4) `intrinsic.stop` on `arm`.
    *   Transitions: 1 → 2 when tare done; 2 → 3 when
        `elapsed_time_seconds` > 0.05 and `intrinsic.sensed_force` >
        switch, switch = max(0.5 F, 3 N); 3 → 4 when (`sensed_force` >
        switch and `intrinsic.is_settled`) or `elapsed_time_seconds` > 2.
    *   End: 4 `is_settled` → ok; from 2: `intrinsic.translational_distance_traveled`
        > 0.15 → stop, `ABORTED` "no contact within 0.15 m"; any:
        `sensed_force` > 85 → stop, `ABORTED` "excessive force"; timeout
        60 s sim → `DEADLINE_EXCEEDED`.
    *   Result: `MotionResult` + the final wrench logged; operation detail
        "contact at F N".
4.  **`MoveCompliant(pose, stiffness, max_force)`** on
    `intrinsic.cartesian_admittance_action` (Inv §2):
    *   Validate: stiffness in [50, 5000] N/m, 0 < max_force ≤ 85, pose
        within the Cartesian limits, else `INVALID_ARGUMENT`.
    *   Parameters: `task_t_tool_reference_pose` = target (TCP),
        `robot_tip_t_robot_tool` = flange→`tcp`, `cartesian_stiffness` =
        diag(k, k, k, 30, 30, 30), `virtual_cartesian_inertia` = diag(2, 2,
        2, 0.05, 0.05, 0.05), damping unset (critical), deadband
        1.5 N / 0.2 N·m.
    *   No tare inside: the bias from the preceding `TareWrench` /
        `MoveToContact` is kept (taring while in contact would zero the
        contact force). Documented in the RPC comment.
    *   End: `intrinsic.is_done` → ok; `sensed_force` > max_force →
        stop, `ABORTED` "force limit reached (F N)"; `intrinsic.settled_for_seconds`
        > 1.0 and not `is_done` → stop, `ABORTED` "settled before reaching
        the target (position error e m)"; timeout 30 s sim.
5.  **Payload reaches the F/T compensation** (Inv §0.7): `SetPayload`
    (Modify) also sets the F/T part's `grasped_payload` (same mass and
    centre of mass) through ICON `SetPayload(part_name="ft_sensor",
    payload_name="grasped_payload")`; mass 0 clears both.
6.  **Contact accuracy in Isaac:** `sim/tools/build_catalog.py` (Modify)
    applies `PhysxCollisionAPI` with `contactOffset` 0.001 m and
    `restOffset` 0.0 to every catalog object; regenerate the USDs.
    `run_scene.py` unchanged (1 kHz physics already).
7.  **Proto comments** (`robot_service.proto`, Modify; they become MCP
    descriptions): `MoveToContact` says it tares first and stops at the
    force; `MoveCompliant` says it uses the existing tare and stops on
    target, force limit or jam.

## Steps

1.  Decisions 1, 2, 5, 6; unit tests in `tests/test_robot_service.py`
    (Modify): inertia formula (F 15, v 0.01 → 9.19 kg), clamping,
    validation errors, sequence/transition construction.
2.  Decisions 3, 4, 7; rebuild the image.
3.  `tests/e2e/phase5b_check.py` (Create) until all checks pass. If
    `insert` stays below 9/10 with stiffness 200 N/m, record per-trial
    peg depth and contact forces in `phase_5b_notes.md` and stop (the
    lead decides the parameters).
4.  Phases 3–5c checks and `pytest` still pass; write `phase_5b_notes.md`.

## Constraints

As phase 3 (no `git commit`; Docker via `sg docker -c`; no `pkill -f`;
Bazel only via `container/build_intrinsic.sh`; Observability).
