# Phase 5b notes

**Status: stopped at Step 3 again, after Addendum A** (run 3). `insert` is
0/10 and `jam` fails, and the causes need design decisions (see "Run 3"
below). Addendum A items 1–4 are applied. Compliant contact removed the
impact spikes, and `contact_table` now passes. Step 4 was not done:
`robot_platform.md` (CUJ 6, G8) is untouched, and the phase 3/4/5c checks
were not rerun with the new catalog materials. Decisions 1–8 and Steps 1–2
are implemented.

## Check results

| Check | Result | Log |
|-------|--------|-----|
| `phase5b_check.py` run 3 (Addendum A) | contact_table PASS, no_contact PASS, bad_speed PASS, insert FAIL 0/10, jam FAIL, logs FAIL (check bug, fixed after the run) | `logs/phase5b_check_run3.log` |
| `pytest -q tests/` after run 3 | 77 passed | |
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

## Run 3 (Addendum A)

Applied:
- **A1:** `sim/tools/build_catalog.py` binds a physics material
  (`/Object/physics_material`) to every catalog mesh: `compliantContactStiffness`
  60000, `compliantContactDamping` 600, and friction 0.5 static, 0.4 dynamic.
  All 7 `model.usd` files were regenerated (`logs/phase5b_build_catalog_a.log`).
  This includes `table`, which is a catalog object.
- **A2:** the insert trial is now: `MoveToContact` on the face, then
  `MoveCompliant` to the axis 5 mm lower (1000 N/m, 30 N), then
  `MoveToContact` to the hole bottom. The face-contact force (12–18 N) and
  top-face checks are kept from the original `insert` row.
- **A3:** `jam` runs `MoveToContact(10 N)`, then `MoveCompliant` 3 cm below
  the contact pose (1000 N/m, 40 N).
- **A4:** force calls use `wait=false` plus repeated `WaitOperation`, 120 s
  wall in total. This worked: operations of 62 s sim (about 93 s wall)
  ended with their own status.

### Insert, run 3, per trial

Columns: start offset = true peg position minus the hole axis before
contact. Final force = along the push, from the operation message.
Peak = largest `GetWrench` norm (10 ms sampler).

| # | Start off axis (mm), height | MoveToContact (face) | Final force / peak (N) | x/y off axis at contact (mm) | MoveCompliant (search) | Depth |
|---|---|---|---|---|---|---|
| 1 | +2.66, −0.27; 19.2 mm | DEADLINE_EXCEEDED at 62.08 s sim | 15.7 / 16.4 | – | not run | on the face |
| 2 | +2.98, −0.20; 19.6 mm | DEADLINE_EXCEEDED 62.07 s | 13.7 / 14.3 | – | not run | on the face |
| 3 | +3.06, −0.25; 19.6 mm | DEADLINE_EXCEEDED 62.08 s | 13.7 / 14.2 | – | not run | on the face |
| 4 | +2.60, −0.30; 19.1 mm | DEADLINE_EXCEEDED 62.06 s | 13.5 / 14.2 | – | not run | on the face |
| 5 | +3.30, −0.23; 19.7 mm | DEADLINE_EXCEEDED 62.08 s | 14.8 / 15.9 | – | not run | on the face |
| 6 | +2.91, −0.20; 19.5 mm | DEADLINE_EXCEEDED 62.08 s | 13.8 / 14.0 | – | not run | on the face |
| 7 | +2.61, −0.29; 19.1 mm | DEADLINE_EXCEEDED 62.08 s | 13.7 / 14.1 | – | not run | on the face |
| 8 | +2.84, −0.05; 19.7 mm | done 6.96 s | 14.3 (GetWrench 14.44) / 14.7 | +3.00, +0.21 (peg bottom z 0.0500) | DEADLINE_EXCEEDED at 32.05 s sim, 7.5 N, peak 14.8 | on the face |
| 9 | +2.64, −0.24; 19.2 mm | DEADLINE_EXCEEDED 62.08 s | 13.7 / 15.1 | about +2.7, −0.4 (probe; z 0.0495) | not run | 0.5 mm |
| 10 | +2.96, −0.16; 19.6 mm | DEADLINE_EXCEEDED 62.08 s | 13.9 / 14.3 | about +3.0, −0.4 (probe; z 0.0496) | not run | 0.4 mm |

The check printed no final depth for failed trials; this is fixed now.
For trials 9–10, a read-only probe (ICON `GetStatus` and `GetTruePose`
every 0.5 s) recorded the peg on the face, and the others ended at the
same stage. No trial faulted ICON. The second `MoveToContact` was never
reached.

`jam`: `MoveToContact(10 N)` done after 7.26 s, 9.7 N. `MoveCompliant`
returned ABORTED "force limit reached" after 0.09 s (2.09 s including the
stop wait), with a peak of 40.1 N.

`logs` showed 0 lines: a check bug. The `GetWrench` sampler logs about 100
RPC lines a second, which rotated `robot_service.log` (10 MB) during the
run, and the check counted lines from an index into the old file. The check
now reads the rotated files too and filters by timestamp. This is untested
against a live run.

### Findings 6–9 (for the lead)

6. **Compliant contact works.** Peaks are 14–16 N where run 2 saw 40–136 N.
   `contact_table` passes: 15.04 N, done after 14.41 s.
7. **ICON never sees the arm as settled when the peg is pressed on the
   face.** This causes the 8/10 face timeouts and probably the search
   timeout. During the stall the probe read:
   - arm joint positions constant to 1e-5 rad, the true peg pose constant
     to 0.1 mm, and commanded velocities 0;
   - joint 7 (coaxial with the peg) sensed velocity a steady 0.060–0.063
     rad/s, and joints 2–6 between 0.001 and 0.005 rad/s.

   At 0.06 rad/s, joint 7 would turn 0.36 rad in 6 s, yet its position did
   not change. So the reported velocity is not real motion; it is most
   likely a PhysX solver residual with the stock Franka's iteration counts,
   but that is not verified. ICON's `IsSettledCriterion`
   (`is_settled_criterion.cc`) uses the sensed joint velocities and
   settles only when their norm stays below about 0.026 rad/s. So the
   final `intrinsic.stop` never reports `is_settled`, and decision 4's
   success condition never fires. Without the peg (`contact_table`) it
   settled after roughly 8 s (14.41 s in total, for a 3 cm approach at
   1 cm/s and a stabilize phase of at most 2 s), so run 2's 7–24 s durations were probably the
   same effect, only milder. `intrinsic.settled_for_seconds` (decision 5's
   jam test) probably comes from the same signal; that was not checked.
8. **The search cannot reach the hole with an isotropic spring.** The
   target is d off axis and h = 5 mm below the face. The normal force is
   about k·h and the lateral pull about k·d, so the peg slides only while
   d > μ·h: 2.5 mm static, 2.0 mm dynamic. This holds for any k. The peg
   must get within the 1 mm clearance to drop in. In trial 8 it stayed at
   +3.00 mm (3 N pull against 2.5 N of static friction). Even without
   finding 7, the search would stall about 2–2.5 mm off axis, and the jam
   abort it would then need also depends on settling.
9. **Jam overshoot.** The admittance is 2 kg with critical damping for
   1000 N/m (about 89 N·s/m), and it acts in series with the 60 kN/m
   contact. That gives a damping ratio of about 0.13 and about 65 %
   overshoot. Stepping from 10 N to the 30 N spring force predicts a
   peak near 43 N, which is above the 40 N `max_force`. Measured: 40.1 N,
   aborted as "force limit reached".

Options, one decision per finding (not applied):
- (7) Choose one:
  - Sim: raise the Franka articulation's `solverVelocityIterationCount`
    (`PhysxArticulationAPI`), or publish joint velocities as
    differences of joint positions in `sim/graphs.py`. Either needs the
    phase 3/4/5c checks rerun.
  - Server: end `MoveToContact` when the stop is settled *or* has run
    for about 0.5 s, and detect the jam from sensed TCP motion
    (< 0.5 mm in 1 s) instead of `settled_for_seconds`. This changes
    decisions 4–5 and needs an image rebuild.
- (8) Choose one:
  - Put the search target past the axis, for example 3 mm beyond it, so
    the pull stays ≥ k·3 mm at the hole edge.
  - Use h ≤ 1.5 mm, so that μ·h < 0.8 mm.
  - Use anisotropic stiffness (lateral much stiffer than normal). This
    changes the `MoveCompliant` API or decision 2.
  - Use a scripted lateral search with `MoveToPose` steps.
- (9) Choose one:
  - Raise the jam `max_force` to about 60 N.
  - Lower the stiffness to about 500 N/m (spring about 15 N).
  - Set admittance damping for the contact, about 500 N·s/m, instead of
    "unset (critical)". This changes decision 2.

### Probe

`/tmp/probe5b.py` (not in the repo) read ICON `GetStatus` (port 18003)
and `SimService.GetTruePose` while the check ran. It is read-only; the
check owned Isaac and the container.

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
- `sim/tools/build_catalog.py`: `PhysxCollisionAPI` offsets; compliant
  physics material (Addendum A1)
- `assets/objects/*/model.usd`: all 7 regenerated (again for A1)
- `tests/e2e/phase5b_check.py` (run 3): Addendum A2–A4, failed trials
  report depth, `logs` reads the rotated log files

Regenerated `gen/`:
- `intrinsic/icon/actions/{force_primitive,cartesian_admittance_action,cartesian_impedance,cartesian_impedance_reference_generators}_pb2*`
- `intrinsic/icon/control/…`, `intrinsic/icon/proto/matrix_pb2*`
- `intrinsic/manipulation/…`, `intrinsic/math/proto/twist_pb2*`
- `robot_service/robot_service_pb2_grpc.py` (docstrings only)

Other logs:
- `logs/phase5b_build_catalog.log`, `logs/phase5b_build_catalog_a.log`
- `logs/phase5b_docker_build.log`
- `logs/phase5b_robot_service.log`
