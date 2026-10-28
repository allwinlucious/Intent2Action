# Phase 5b notes

**Status: stopped at Step 3** (plan: "If `insert` is below 9/10, record per
trial … and stop (the lead decides parameters)"). `insert` was 0/10.
`contact_table` and `jam` also fail, for physics and parameter reasons
rather than wiring (see below). Step 4 was not done, so decision 9
(`robot_platform.md`) is untouched. Decisions 1–8 and Steps 1–2 are
implemented.

## Check results

| Check | Result | Log |
|-------|--------|-----|
| `phase5b_check.py` run 2 (final code) | contact_table FAIL, no_contact PASS, bad_speed PASS, insert FAIL 0/10, jam FAIL, logs PASS | `logs/phase5b_check_run2.log` |
| `phase5b_check.py` run 1 (before the three fixes below) | contact_table, insert 0/10, jam, logs FAIL | `logs/phase5b_check_run1.log` |
| phase 3 hwm / scene / cuj, phase 4, phase 5c (with phase 5b code) | all PASS on first run | `logs/phase5b_regress_*.log` |
| `pytest -q tests/` | 77 passed (25 new in `tests/test_force.py`) | |

Fixes made between run 1 and run 2 (implementation bugs, not design):
1. ICON rejects an admittance action without `reference_generators`
   (`REFERENCE_GENERATOR_NOT_SET`). `admittance_params` now sets the
   default `simple_lowpass` Cartesian and nullspace generators, the same
   as Intrinsic's `create_cartesian_admittance_action`.
2. When a transition and an abort fire in the same cycle, ICON ends the
   session ("The same part slot is requested from multiple actions during
   reaction handling: 2 and 3"). In `run_sequence`, transitions and the
   success reaction of a watched action now also require every abort
   condition to be false, so the abort wins.
3. Sequences that ICON ended had no `force` log line. `run_force` now
   logs one with `force_along_push_n=nan` before re-raising.

## Insert, run 2, per trial

Each trial started 3 mm off-axis in +x (target), 2 cm above the face, used
`TareWrench`, then `MoveToContact((0,0,−1), 15, 0.01)` and
`MoveCompliant(hole axis, 3 cm lower, 200, 20)`. Column meanings:
- **Start off axis**: true peg position minus the hole axis, before contact.
- **Contact force**: `GetWrench` along the push after `MoveToContact`.
- **x/y off axis**: true peg position minus the hole axis, at contact.
- **Peg bottom**: true peg bottom z at contact; the top face is at 0.050.
- **Depth**: below the top face after `MoveCompliant`; success needs ≥ 25 mm.
- **Peaks**: the largest `GetWrench` norm, sampled every 10 ms wall.

| # | Start off axis (mm) | MoveToContact | Contact force | x/y off axis at contact (mm) | Peg bottom (m) | MoveCompliant | Depth (mm) | Peak MTC / MC (N) |
|---|---|---|---|---|---|---|---|---|
| 1 | +2.92, +0.16 | done 6.96 s | 0.14 N | +0.84, −0.29 | 0.0475 | force limit, 2.06 s | 33.1 | 62 / 43 |
| 2 | +2.71, −0.23 | done 13.93 s | 14.31 N | +1.80, −0.28 | 0.0496 | force limit, 2.06 s | 0.4 | 66 / 69 |
| 3 | +2.80, −0.11 | done 7.70 s | 15.30 N | +1.25, −0.64 | 0.0501 | force limit, 0.98 s | −0.1 | 75 / 21 |
| 4 | +2.60, −0.29 | done 24.42 s | 13.12 N | +1.82, +0.04 | 0.0496 | force limit, 2.06 s | 0.4 | 71 / 104 |
| 5 | – | still running after the server's 60 s wall `wait`; the cleanup cancelled it at 37.5 s sim with 12.8 N | – | – | – | – | – | – |
| 6 | +2.57, −0.30 | done 8.08 s | 10.14 N | +1.69, −0.08 | 0.0496 | force limit, 1.35 s | 0.4 | 73 / 14 |
| 7 | +2.49, −0.33 | done 8.77 s | 10.56 N | +2.81, −0.07 | 0.0498 | force limit, 1.75 s | 0.3 | 42 / 84 |
| 8 | +2.88, −0.12 | excessive force, 6.25 s | – | – | – | – | – | 12 (spike missed by the sampler) |
| 9 | +2.73, −0.17 | excessive force, 7.74 s | – | – | – | – | – | 85 |
| 10 | +2.83, −0.15 | excessive force, 7.88 s | – | – | – | – | – | 77 |

No trial faulted ICON. Run 1 had the same pattern before its fixes:
- 3/10 excessive force;
- 3/10 where the peg slid into the hole during `MoveToContact` (peg bottom
  at 0.010, or "no contact within 0.15 m");
- contact forces of 12.7–15.4 N when contact landed on the face.

## Findings (for the lead)

The sequences behave as designed. The failures come from how stiff contact
is in PhysX and from the parameters the plan set.

1. **Contact impact spikes.** At 10 mm/s with 9.19 kg virtual inertia,
   `GetWrench` peaks at 40–136 N on touching the face or the table, even
   when contact succeeds. ICON samples at 1 kHz, so it sees spikes the
   10 ms client sampler misses (trial 8). The "excessive force" limit of
   85 N sits inside that spread: 3/10 trials and `contact_table` (136 N
   at 3.10 s) aborted on it. MakeContact's speed formula assumes an
   environment of 60 kN/m. PhysX rigid contact is far stiffer.
2. **MoveCompliant starts at the force limit.** After `MoveToContact`
   the peg is held at about 15 N. `max_force` 20 leaves 5 N of headroom,
   and all 6 trials that reached `MoveCompliant` hit "force limit
   reached". In 3 of them it fired within about 60 ms of starting:
   2.06 s is about 0.06 s plus the 2 s settle wait. In the other 3 the
   stop settled sooner, so the abort time is unknown. With stiffness 200 N/m
   and a 3 mm lateral offset, the spring pulls sideways with only about
   0.6 N, well under face friction at 6–15 N normal force. The peg would
   probably not reach the hole even without the limit; it would end as a
   jam.
3. **The peg drifts toward the axis during the approach.** The start
   offset is 2.5–2.9 mm, but at contact it is 0.8–2.8 mm. MakeContact's
   orthogonal compliance or the peg moving in the fingers lets it slip
   about 1 mm. Near 1 mm the peg catches the hole edge: trial 1 here, and
   trials 3, 4 and 10 in run 1.
4. **Jam start impact.** From 2 cm above the table, a 2 kg / 200 N/m
   spring toward a target 5 cm below reaches about 0.18 m/s at the table.
   The result was a 359 N peak and 164 N held after the stop, so the
   abort is "force limit reached", not a jam.
5. **Wait cap vs sim timeout.** The server's existing `wait` returns
   after 60 s wall. The sim runs slower than real time, so a 60 s-sim
   `MoveToContact` can outlive it (trial 5). The test should poll, or the
   cap should be raised.

Options, one decision per finding (not applied):
- (1) Make PhysX contacts compliant at about 60 kN/m (`PhysxMaterialAPI`
  compliant contact stiffness and damping on the catalog objects and the
  table), so the sim matches MakeContact's model. Or approach at 3–5 mm/s,
  or raise the excessive-force limit (e.g. 150 N).
- (2) Back off to about 2–5 N before `MoveCompliant` (lower `max_force`
  on contact, or a small lift). Raise `MoveCompliant` `max_force` to
  about 40 N. Raise the lateral stiffness to 1000–2000 N/m so 3 mm gives
  3–6 N.
- (3) Accept the drift, start 4 mm off the axis, or stiffen MakeContact's
  orthogonal directions.
- (4) Start the jam in contact (`MoveToContact` first), or put the start
  ≤ 5 mm above the table.
- (5) Poll the operation in the check.

## Rulings made while implementing (plan silent or not workable as written)

- **Reference generators:** the default admittance reference generator is
  set; ICON requires one (fix 1 above).
- **Abort priority:** transitions and success wait for no abort condition,
  so the abort wins. This also implements "abort flags win" inside ICON
  (fix 2).
- **Jam position error:** `e` is computed from FK of the final sensed
  joints against the target. `intrinsic.reference_position_error` is not
  in `GetStatus` (only `part_status`), so the client cannot read it.
- **MoveCompliant push direction:** the unit vector from the start TCP to
  the target. `push_force` returns the absolute value, since the plan
  does not assert the sign.
- **Rejected calls:** calls rejected by validation (`INVALID_ARGUMENT`)
  get the usual `rpc` log line, but no `force` line, because no
  operation ran.
- **Placement in the check:** the check repeats the placement move from
  the true peg pose, up to 3 moves with 0.3 mm tolerance, so the 3 mm
  offset is real. A single move was about 1 mm off in y. The
  `MoveCompliant` target uses the true peg position at contact.
- **Excessive torque:** 8.5 N·m, alongside the plan's 85 N, as the
  `force_control_settings` limits (the tutorial values).
- **FK per call:** each force RPC runs one FK call to get flange→TCP for
  the task frame.

## Files changed

Created:
- `container/services/robot/force.py`
- `tests/test_force.py`
- `tests/e2e/phase5b_check.py`
- this file

Modified:
- `container/services/robot/icon.py`: `set_payload` (decision 6),
  `run_sequence`, `SequenceResult`
- `container/services/robot_service.py`: `flange_t_tcp`, `run_force`,
  `MoveToContact`, `MoveCompliant`
- `robot_service/robot_service.proto`: comments
- `sim/tools/build_catalog.py`: `PhysxCollisionAPI` offsets
- `assets/objects/*/model.usd`: all 7 regenerated

Regenerated `gen/`:
- `intrinsic/icon/actions/{force_primitive,cartesian_admittance_action,cartesian_impedance,cartesian_impedance_reference_generators}_pb2*`
- `intrinsic/icon/control/…`, `intrinsic/icon/proto/matrix_pb2*`
- `intrinsic/manipulation/…`, `intrinsic/math/proto/twist_pb2*`
- `robot_service/robot_service_pb2_grpc.py` (docstrings only)

Other logs:
- `logs/phase5b_build_catalog.log`
- `logs/phase5b_docker_build.log`
- `logs/phase5b_robot_service.log`
