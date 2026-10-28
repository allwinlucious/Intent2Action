# Phase 5b notes

**Status: stopped at Addendum E.3** (run 7). Addendum E.1 and E.2 are
applied: admittance inertia 5 kg, compliant damping 2000 (all 7 USDs
regenerated), and `jam` uses the grasped peg on the block face. The full
`phase5b_check.py` passes `insert` 9/10, `no_contact`, `bad_speed` and
`logs`. It fails `contact_table` (bare fingers on the table chatter and end
at 5.2 N) and `jam` (it holds at about 31 N but never reports "settled",
so it ends on the 30 s deadline). Neither can pass without a new decision
(findings 18–19). So C.3 (the phase 3, 4 and 5c regressions) and Step 4
were not run, and `robot_platform.md` is untouched. `pytest` passes (77).
E.1 and E.2 stay applied.

**Previous status (run 6): stopped at Addendum C.2.** Addendum D is applied: the
grasp depth is back to `min(0.02, extent − 0.01)`, and `phase5c_check.py`
passes with cube 10/10 and peg 10/10. In the sweep, no combination
qualifies. `jam` fails in all six combinations, always with "force limit
reached" and never with "settled before reaching the target". Three
combinations reach ≥ 4/5 inserts. As C.2 requires, the full
`phase5b_check.py`, C.3 and Step 4 were not run, and `robot_platform.md`
(CUJ 6, G8) is untouched. The sweep parameters are back at the committed
values (admittance inertia 2 kg, compliant damping 600; USDs as
committed). The phase 3 and 4 checks and pytest pass on the final tree
(see "Run 6" below). Decisions 1–8, Steps 1–2, Addenda A and B, C.1 (finger
material and the 60 N grip) and D are implemented.

## Check results

| Check | Result | Log |
|-------|--------|-----|
| `phase5b_check.py` run 7 (Addendum E) | contact_table FAIL (5.18 N), no_contact PASS, bad_speed PASS, insert PASS 9/10, jam FAIL (DEADLINE_EXCEEDED), logs PASS | `logs/phase5b_check_run7.log` (Isaac, container, service: `logs/phase5b_{isaac,container,robot_service}.log`) |
| `pytest -q tests/` (Addendum E) | 77 passed | |
| `phase5c_check.py` after D | all PASS: planner_up, cube 10/10, peg 10/10, too_wide, logs, errors | `logs/phase5b_addD_phase5c_check.log` (Isaac and container: `logs/phase5b_addD_phase5c_{isaac,container}.log`) |
| C.2 sweep, 6 combinations (`phase5b_check.py --sweep`) | none qualifies: `jam` FAIL in all 6; inserts 3/5, 4/5, 1/5, 5/5, 2/5, 4/5 | `logs/phase5b_sweep_i{2,5,10}_d{600,2000}.log` |
| phase 3 hwm / scene / cuj, phase 4 (final tree) | all PASS on first run | `logs/phase5b_addD_regress_*.log` |
| `pytest -q tests/` (final tree) | 77 passed | |
| `phase5c_check.py` after C.1 | planner_up PASS, cube FAIL 0/10, peg FAIL 0/10, too_wide PASS, logs PASS, errors PASS | `logs/phase5b_addC_phase5c_check.log` (Isaac and container: `logs/phase5b_addC_phase5c_{isaac,container}.log`) |
| `pytest -q tests/` after C.1 | 78 passed | |
| `phase5b_check.py` run 4 (Addendum B) | contact_table PASS, no_contact PASS, bad_speed PASS, insert FAIL 1/10, jam FAIL, logs PASS | `logs/phase5b_check_run4.log` |
| `container/build_intrinsic.sh --test` (B1) | build OK; `ros2_hwm_logic_test` PASSED (3 new `VelocityEstimator` tests) | `logs/phase5b_addB_build.log` |
| `pytest -q tests/` after run 4 | 77 passed | |
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

## Run 7 (Addendum E)

Applied:
- **E.1:** `ADMITTANCE_INERTIA` (`container/services/robot/force.py`) is
  (5.0, 0.05), and `tests/test_force.py` expects diag(5, 5, 5, 0.05, 0.05,
  0.05). `COMPLIANT_DAMPING` (`sim/tools/build_catalog.py`) is 2000. All 7
  `model.usd` were regenerated (`logs/phase5b_addE_build_catalog.log`), and
  the image was rebuilt (`logs/phase5b_addE_docker_build.log`; the image's
  `force.py` was checked to have 5.0).
- **E.2:** `check_jam` grasps the peg the same way as `insert`, through a
  shared helper `_peg_above_block(off_axis_m)` with the grasp, attach,
  payload and placement steps from `_insert_trial`. The peg starts 20 mm
  off the axis in +x and 2 cm above the top. Then: `TareWrench`,
  `MoveToContact(down, 10 N, 0.01)`, and `MoveCompliant` (3 cm below the
  contact TCP pose, 1000 N/m, 60 N). Geometry: the peg (radius 10 mm) at
  +20 mm covers +10 to +30 mm, and the hole radius is 11 mm, so a 1 mm
  sliver of the peg bottom hangs over the hole rim. The centre and nearly
  all of the bottom are on the face. `JAM_START_Z` is removed, and
  `contact_table` is unchanged.

### Insert, run 7, per trial

Same columns as run 4. Face/bottom = the `MoveToContact` result (sim time,
final force in the operation message). Contact = `GetWrench` along the push
afterwards.

| # | Start off axis (mm), height | Face | Contact (N) | Off axis at contact (mm), peg bottom z | Search (`MoveCompliant`) → off axis after (mm) | Bottom | Depth |
|---|---|---|---|---|---|---|---|
| 1 | +3.00, −0.07; 19.6 | 5.80 s, 14.3 N | 14.05 | +2.98, −0.21; 0.0497 | done 2.20 s → +0.91, −0.42 | 6.27 s, 12.98 N | 34.1 mm |
| 2 | +2.45, −0.34; 18.8 | 5.78 s, 13.1 N | 13.54 | +2.64, −0.32; 0.0496 | done 1.80 s → +0.87, −0.28 | 6.46 s, 12.00 N | 40.0 mm |
| 3 | +2.62, −0.24; 19.1 | 5.74 s, 12.6 N | 13.23 | +2.68, −0.32; 0.0496 | done 1.62 s → +0.76, −0.27 | 6.43 s, 12.50 N | 34.1 mm |
| 4 | +2.70, −0.26; 19.1 | 5.74 s, 13.0 N | 13.56 | +3.13, −0.04; 0.0497 | DEADLINE_EXCEEDED 30.29 s, 3.0 N | – | **1.1 mm (fail)** |
| 5 | +2.50, −0.32; 18.9 | 5.88 s, 13.2 N | 13.47 | +2.65, −0.47; 0.0496 | done 0.93 s → +0.31, −0.31 | 6.41 s, 12.44 N | 34.1 mm |
| 6 | +2.77, −0.37; 18.6 | 5.77 s, 13.6 N | 12.93 | +2.69, −0.34; 0.0496 | done 1.22 s → +0.42, −0.35 | 6.37 s, 12.88 N | 35.1 mm |
| 7 | +2.73, −0.30; 19.0 | 5.81 s, 11.2 N | 17.71 | +2.80, −0.37; 0.0496 | done 1.25 s → +0.53, −0.27 | 6.31 s, 13.05 N | 34.3 mm |
| 8 | +2.57, −0.37; 18.7 | 5.77 s, 12.7 N | 12.65 | +2.65, −0.41; 0.0496 | done 12.53 s → +1.10, −0.02 | 4.10 s, 0.81 N | 29.7 mm |
| 9 | +2.96, −0.13; 19.6 | 5.77 s, 13.6 N | 13.23 | +2.87, −0.25; 0.0497 | done 1.52 s → +0.61, −0.16 | 6.23 s, 12.07 N | 34.2 mm |
| 10 | +3.04, −0.13; 19.6 | 5.75 s, 12.7 N | 13.11 | +3.17, −0.10; 0.0498 | done 7.36 s → +0.79, −0.45 | 6.17 s, 12.66 N | 34.2 mm |

No trial faulted ICON. Every face contact was on the top face at
12.65–17.71 N (`GetWrench`), inside 12–18 N. In trial 7 the operation's
own final force was 11.2 N (face peak 18.4 N, p10/p90 4.3/16.6 N), but the
criterion uses `GetWrench`. Trial 8's bottom `MoveToContact` was "done" at
0.8 N and 29.7 mm. That passes the 25 mm criterion, but the peg did not
press on anything at 12–18 N. Trial 2 reached 40.0 mm (the hole bottom),
while the other successes stopped at 34.1–35.1 mm (finding 17).

### `contact_table` and `jam`, run 7

- `contact_table`: `MoveToContact(down, 15 N, 0.01)` was done after
  4.45 s with 5.3 N. `GetWrench` along the push was then 5.18 N. The peak
  was 48.3 N, and the second-half p10/p90 was 1.1/34.6 N.
- `jam`: the peg started at (+19.89, −0.08) mm and 19.5 mm up.
  `MoveToContact(10 N)` was done after 7.01 s with 8.2 N (`GetWrench`
  8.50 N; peak 10.1 N), and the peg bottom was at z 0.0500, 19.6 mm off the
  axis, on the face. `MoveCompliant` (3 cm down, 1000 N/m, 60 N) returned
  DEADLINE_EXCEEDED "not finished within 30 s of sim time", with a final
  force of 31.3 N, a peak of 37.7 N and a second-half p10/p90 of
  25.8/37.2 N. Neither the force limit nor the settled abort fired.

### Findings 18–20 (for the lead)

18. **`contact_table` fails because bare fingers on the table chatter.**
    It is the same contact that E.2 replaced in `jam` (finding 16), and
    `contact_table` still uses it. With damping 2000 the closed-finger
    `MoveToContact` on the table ended at 1.7–4.2 N of 10 N in the sweep,
    and at 5.3 N of 15 N here. At damping 600 with the C.1 finger material
    it ended at 7.0–7.9 N of 10 N, also below −20 %. So reverting damping
    alone probably would not pass either (inferred from the sweep's jam
    `MoveToContact`; `contact_table` was not run at 600 after C.1). It last
    passed in run 4, before the finger material. Against the peg-on-face
    contact, G8 accuracy holds: 10/10 face contacts at 12.65–17.71 N for
    15 N, and the jam's 10 N contact at 8.50 N.
19. **The peg jam holds but never reports "settled".** The force limit
    no longer trips (peak 37.7 N, under 60). The arm presses at about the
    30 N spring force for 30 s, and the force varies between 26 and 37 N
    (p10/p90). `intrinsic.settled_for_seconds` > 1.0 never fired. This
    matches finding 12, where searches ran into the deadline instead of
    aborting as settled. It probably comes from the same thing: the
    remaining force oscillation keeps ICON from settling. This is not
    verified, because the client cannot read that state variable, and the
    TCP motion during the jam was not sampled.
20. **The insert passes, with one search stall.** Trial 4 started the
    search 3.13 mm off the axis and stalled there at 3.0 N until the
    deadline (the same failure as run 4). Trial 10 started 3.17 mm off and
    succeeded after 7.36 s. Successful searches took 0.93–12.53 s.

Options (not applied):
- (18) Choose one:
  - Run `contact_table` with the grasped peg pressed on the block face
    away from the hole, as E.2 does for `jam`.
  - Give the finger material compliant-contact parameters, or remove it
    and keep only the 60 N grip. That would need 5c rerun, since C.1 fixed
    the grip.
  - Judge the force from ICON's filtered sensed force at the end of the
    operation, instead of a single `GetWrench` sample. The operation's own
    value was 5.3 N here as well, so this alone does not pass.
- (19) Choose one:
  - Detect the jam on the server from TCP motion, for example < 0.5 mm in
    1 s (run 3 option 7, run 4 option 12). This changes decision 5.
  - Detect the jam from force: the sensed force stays within a band (for
    example ±10 N) for 1 s while the position error stays above a
    threshold. This also changes decision 5.
  - Accept DEADLINE_EXCEEDED with a large position error as the jam
    outcome. This changes the `jam` criterion and the RPC contract.

## Run 6 (Addendum D, then the C.2 sweep)

Applied:
- **D:** `container/services/robot/grasp.py` and `tests/test_plan_grasp.py`
  are restored to their phase 5c content (`MAX_DEPTH_M` 0.02, the old
  depth expectations). The finger material and the 60 N grasp force from
  C.1 stay. Image rebuilt (`logs/phase5b_addD_docker_build.log`).
  `phase5c_check.py` passes.
- **C.2 sweep:** for each combination, `ADMITTANCE_INERTIA` in
  `container/services/robot/force.py` (translational only; rotational
  stays 0.05) and `COMPLIANT_DAMPING` in `sim/tools/build_catalog.py`
  were set. All 7 USDs were regenerated whenever the damping changed
  (`logs/phase5b_build_catalog_d{600,2000}.log`), the image was rebuilt,
  and `phase5b_check.py --sweep` ran (`jam`, then 5 `insert` trials).
  After the sweep, `force.py`, `build_catalog.py` and
  `assets/objects/*/model.usd` were restored to the committed files
  (2 kg, 600), and the image was rebuilt again.

### Sweep results

`jam` columns: the `MoveToContact(10 N)` result, and the `MoveCompliant`
(1000 N/m, 3 cm down, 60 N) result. Each has the final force along the
push, the peak `GetWrench` norm, and the p10/p90 of the second half of the
samples (the two levels of an alternating wrench). Inserts: depth per
trial in mm (success ≥ 25 mm). Search = the `MoveCompliant` (2 mm down,
2000 N/m, 30 N) outcome.

| Inertia (kg), damping | `jam` MoveToContact | `jam` MoveCompliant | Inserts | Depths (mm) | Search |
|---|---|---|---|---|---|
| 2, 600 | done 3.38 s, 7.1 N; peak 32.1, p10/p90 0.6/26.0 | force limit after 0.56 s, 25.9 N; peak 57.3, 26.1/28.5 | 3/5 | 32.8, 29.8, 25.0, 18.0, 20.1 | 5 done in 8.2–18.3 s; hole-bottom contact only 0.4–5.6 N |
| 2, 2000 | done 3.38 s, 1.7 N; peak 46.9, 0.4/31.0 | force limit after 0.34 s, 4.8 N; peak 82.6, 4.4/5.3 | 4/5 | 34.1, 34.1, 34.0, 34.2, 0.9 | 4 done in 0.74–0.90 s; 1 force limit after 0.59 s (6.0 N, peak 23.3) |
| 5, 600 | done 3.38 s, 7.0 N; peak 32.5, 0.6/25.8 | force limit after 0.59 s, 32.3 N; peak 60.5, 33.5/35.7 | 1/5 | 1.0, 0.9, 1.2, 1.3, 35.1 | 4 DEADLINE_EXCEEDED at 30.3 s (2.5–3.0 N); 1 done in 2.02 s |
| 5, 2000 | done 3.38 s, 3.8 N; peak 49.5, 0.4/29.2 | force limit after 0.44 s, 7.5 N; peak 56.6, 7.0/7.8 | **5/5** | 34.1, 34.1, 34.1, 34.3, 34.1 | 5 done in 1.24–7.03 s |
| 10, 600 | done 3.37 s, 7.9 N; peak 32.1, 0.6/25.8 | force limit after 4.41 s, 34.3 N; peak 61.1, 17.2/53.5 | 2/5 | 1.3, 28.0, 33.3, 1.2, 0.9 | 3 DEADLINE_EXCEEDED at 30.3 s (2.3–3.1 N); 2 done in 1.31–3.92 s |
| 10, 2000 | done 3.39 s, 4.2 N; peak 48.6, 0.4/29.7 | force limit after 0.44 s, 8.9 N; peak 70.5, 8.8/9.9 | 4/5 | 34.4, 1.1, 34.2, 33.4, 33.3 | 4 done in 1.02–10.72 s; 1 DEADLINE_EXCEEDED (2.8 N) |

In all 30 trials the face `MoveToContact` was done after 5.6–5.8 s of
sim time, on the top face, at 12.4–14.6 N, with a peak of 14–16 N. No
trial faulted ICON. The sweep does not run the `logs` check.

### Findings 14–17 (for the lead)

14. **The grip now holds the peg.** Every search that ended `done` left
    the peg 0.3–1.2 mm from the axis, with the peg bottom 2.8–3.3 mm below
    the face. In run 4 the peg moved < 0.1 mm and the hand moved instead.
    The phase 5c depth, the finger material and the 60 N grip together
    fix finding 10.
15. **`jam` fails the same way in every combination: "force limit
    reached".** It aborts after 0.34–0.59 s (4.41 s once), never on
    "settled". In the damping-2000 runs the force after the stop is only
    4.8–8.9 N (p10/p90 4.4–9.9 N), far below the 30 N spring force, so the
    60 N limit tripped on a short spike early in the move. The
    `GetWrench` peaks are 57–83 N, and ICON samples at 1 kHz, so it can
    see higher peaks than the 10 ms client sampler. Neither swept
    parameter removes it.
16. **The alternating wrench is in the closed-finger contact with the
    table, before `MoveCompliant` starts.** The jam's `MoveToContact`
    (which does not use the admittance inertia) already alternates
    between p10 0.4–0.6 N and p90 26–31 N, with peaks of 32–50 N. It ends
    at 1.7–7.9 N, below its 10 N target. Damping 2000 raises those peaks
    (47–50 N against 32 N). The peg-on-face contact does not alternate:
    its peak is within about 2 N of the final force. So `jam`, as
    specified with closed fingers on the table, starts from a chattering
    contact. The chattering is likely related to the finger geometry or
    the C.1 finger material against the compliant table, but that is not
    verified.
17. **Damping 2000 makes the search and the hole-bottom contact
    reliable.** With damping 2000, 13/15 searches were done in 0.7–11 s,
    and every hole-bottom contact reached 11.8–14.0 N at depth 33.3–34.4
    mm. With 600, 7/10 searches at 5 and 10 kg hit the 30 s deadline at
    about 3 N. At 2 kg, the hole-bottom `MoveToContact` was "done" at
    only 0.4–5.6 N, at depths of 18–33 mm. The depth stops at about 34 mm,
    not 40 mm (the hole bottom). Something stops the peg about 6 mm
    above the hole bottom (perhaps the fingers on the block top), but
    this was not checked.

Options (not applied), choose one for `jam`:
- Run the jam with the grasped peg pressed on the block's top face away
  from the hole (the contact that does not chatter, finding 16), instead
  of closed fingers on the table.
- Raise the jam `max_force` to 80 N (below the 85 N excessive-force
  abort). The measured `GetWrench` peaks reach 83 N, so this may still
  trip.
- Lower the jam stiffness to about 500 N/m (15 N spring), with
  `max_force` 60.
- Investigate the finger–table contact in Isaac, for example compliant
  parameters on the finger material.

For `insert` alone: if `jam` were dropped from the criterion, the first
qualifying combination in the C.2 order would be 2 kg with damping 2000
(4/5). The best was 5 kg with damping 2000 (5/5).

## Run 5 (Addendum C)

Applied (C.1, all three items):
- `sim/scene.py`: `/World/physics_materials/finger`, friction 1.2 static
  and 1.0 dynamic, `frictionCombineMode` "max". It is bound with purpose
  "physics" and `strongerThanDescendants` to `panda_leftfinger` and
  `panda_rightfinger`. Isaac logs the material each finger collider
  resolves to, and the run shows it on both colliders
  (`.../panda_leftfinger/geometry/panda_leftfinger`, and the same for the
  right finger).
- `container/config/icon.txtpb`: gripper `default_force` 60. `ros2_hwm`
  logs `gripper command width=0.075 force=60`.
- `container/services/robot/grasp.py`: `MAX_DEPTH_M` 0.04, so
  d = min(0.04, extent along the approach − 0.01). `tests/test_plan_grasp.py`
  now expects 0.04 for the cube, 0.04 for a box 0.08 m tall (the cap) and
  0.015 for a box 0.025 m tall, and the side grasp 0.015 past the cube's
  centre line.
- Image rebuilt (`logs/phase5b_addC_docker_build.log`).

### Finding 13 (for the lead): the new depth makes every grasp unreachable

`phase5c_check.py`: `PlanGrasp` returns OK for every trial (red_cube about
0.9 s, peg about 2.2 s). The check's `MoveToPose` to the deepened grasp
then fails every time with `NOT_FOUND: planning failed: IK could not find
a collision free configuration`. All 4 IK configurations reported collide:
- red_cube (0.05 m, d = 0.04): `franka_hand.panda_hand` against
  `table.base_link`, 3 collisions per configuration. The TCP is 1 cm above
  the table. At d = 0.02 (TCP 3 cm above the table) 5c passed.
- peg (0.08 m tall, top-down, d = 0.04): `franka_hand.panda_hand`
  against `peg.base_link`, 1 collision per configuration. The peg top is
  40 mm above the TCP. In run 4 (finding 10) the peg top reached the hand
  when it was about 35 mm above the TCP: the TCP-to-peg-bottom distance
  shrank from 59 mm to 45 mm before the force reached 13 N. So at this
  depth the peg top is inside the hand.

`PlanGrasp` does not catch this. Addendum B.2's collision check uses the
planner's IK joints for the grasp before it is deepened. `MoveToPose`
solves IK for the deepened pose and checks that.

The C.2 sweep was not started because C.1's precondition failed. The
tooling for it is ready:
- `tests/e2e/phase5b_check.py --sweep` runs `jam`, then 5 `insert`
  trials that need 4 to pass.
- Every force call prints the peak `GetWrench` norm and the 10th/90th
  percentile of the second half of the samples. These are the two levels
  of an alternating wrench.
- `sim/tools/build_catalog.py` has `COMPLIANT_DAMPING` (600) and prints
  it per object.
- The admittance inertia is `ADMITTANCE_INERTIA` in
  `container/services/robot/force.py`, still 2 kg. Changing it needs an
  image rebuild, which takes about 2 s since only the service layers change.

Options (not applied), choose one:
- Revert the depth to 0.02 and keep the finger friction and the 60 N grip.
  Then rerun 5c and the sweep. Whether friction and force alone stop the
  pivot is unknown.
- A depth limited by hand clearance and table clearance, for example
  d = min(0.03, extent − 0.02). That gives cube 0.03 (TCP 2 cm above the
  table) and peg 0.03 (peg top about 5 mm below the hand by the run 4
  estimate). Neither is verified against MoveIt's padding.
- Let the peg grasp touch the hand on purpose (finding 10's option): the
  planner would have to allow the target's contact with the hand, which
  changes phase 5c decision 6.

## Run 4 (Addendum B)

Applied:
- **B1:** `ros2_hwm_logic` gained `VelocityEstimator`. It takes the
  finite difference of consecutive `/joint_states` positions over their
  stamp difference and filters it (v ← 0.5·v + 0.5·v_fd). The first
  sample after a reset and any non-increasing stamp leave v unchanged.
  `ReadStatus` writes it to `joint_velocity_state`, and
  `joint_acceleration_state` is now the difference of that filtered
  velocity. The filter resets when the epoch baseline changes (that is,
  `ClearFaults` after a reset; while the epochs differ `ReadStatus`
  returns the fault before reading) and in `Prepare()`. Isaac's velocity
  is still parsed into `StateSample` but no longer used. Unit tests:
  constant position → 0, ramp → slope, repeated stamp keeps v, reset
  clears. Rebuilt with `build_intrinsic.sh --test`, then
  `stage_intrinsic.sh`, `stage_vendor.sh` (`stage_intrinsic.sh` deletes
  `stage/vendor`), and `docker build`.
- **B2:** `object.yaml` of `peg` and `hole_block` has
  `friction_static: 0.3` and `friction_dynamic: 0.25`. These come from
  `FRICTION` in `tools/build_catalog_meshes.py`, so regenerating keeps
  them. `sim/tools/build_catalog.py` reads them, with a default of
  0.5/0.4. Regenerating the USDs changed only `peg` and `hole_block`
  (`logs/phase5b_build_catalog_b.log`). The search is now 2 mm below the
  contact pose, at 2000 N/m and 30 N.
- **B3:** the `jam` `MoveCompliant` uses `max_force` 60.

**B1 works.** Every face `MoveToContact` was done after 5.3 s of sim time
(run 3: 8/10 hit the 62 s deadline). `contact_table` was done after 3.32 s
(run 3: 14.41 s). While the arm holds in contact, ICON's sensed joint
velocity norm is 0–0.012 rad/s (run 3: a steady 0.06 rad/s on joint 7).

### Insert, run 4, per trial

Column meanings: start off axis = true peg position minus the hole axis
before contact. Contact = `GetWrench` along the push after the face
`MoveToContact`. x/y at contact = true peg position minus the axis.
Depth = below the top face at the end of the trial.

| # | Start off axis (mm), height | MoveToContact (face) | Contact force (N) | x/y off axis at contact (mm), peg bottom z | MoveCompliant (search) | Depth |
|---|---|---|---|---|---|---|
| 1 | +2.76, −0.17; 19.4 mm | done 5.35 s, 13.6 N | 13.49 | +2.94, −0.25; 0.0494 | DEADLINE_EXCEEDED 30.29 s, 3.3 N | 0.7 mm |
| 2 | +2.85, −0.21; 19.5 mm | done 5.38 s, 12.9 N | 13.26 | +2.88, −0.37; 0.0497 | DEADLINE_EXCEEDED 30.29 s, 3.7 N | 0.5 mm |
| 3 | +2.74, −0.20; 19.4 mm | done 5.34 s, 13.0 N | 13.08 | +2.77, −0.47; 0.0495 | ABORTED "settled before reaching the target (position error 0.0021 m)" 3.15 s, 3.7 N | 0.5 mm |
| 4 | +2.89, −0.06; 19.8 mm | done 5.35 s, 13.3 N | 13.36 | +3.05, −0.08; 0.0495 | DEADLINE_EXCEEDED 30.30 s, 3.3 N | 0.6 mm |
| 5 | +2.65, −0.30; 19.2 mm | done 5.32 s, 13.4 N | 13.29 | +2.75, −0.41; 0.0495 | DEADLINE_EXCEEDED 30.29 s, 3.9 N | 0.5 mm |
| 6 | +3.05, +0.12; 20.0 mm | done 5.34 s, 12.8 N | 12.76 | +2.71, +0.16; 0.0500 | done 0.77 s; peg then −0.66, −0.40 mm, bottom z 0.0470; bottom `MoveToContact` done 4.78 s, 13.81 N | **33.4 mm (ok)** |
| 7 | +3.14, −0.13; 19.9 mm | done 5.30 s, 13.3 N | 13.39 | +2.99, −0.21; 0.0497 | DEADLINE_EXCEEDED 30.29 s, 4.0 N | 0.4 mm |
| 8 | +3.14, −0.20; 19.7 mm | done 5.26 s, 12.4 N | 12.43 | +3.35, −0.13; 0.0498 | DEADLINE_EXCEEDED 30.29 s, 3.8 N | 0.3 mm |
| 9 | +2.75, −0.20; 19.5 mm | done 5.31 s, 13.3 N | 13.31 | +2.77, −0.41; 0.0496 | DEADLINE_EXCEEDED 30.29 s, 3.8 N | 0.5 mm |
| 10 | +2.91, −0.11; 19.7 mm | done 5.32 s, 13.3 N | 13.13 | +3.07, −0.25; 0.0496 | DEADLINE_EXCEEDED 30.29 s, 3.4 N | 0.5 mm |

No trial faulted ICON. Every face contact was on the top face, at
12.4–13.5 N (inside 12–18 N).

`jam`: `MoveToContact(10 N)` was done after 2.17 s, at 9.3 N.
`MoveCompliant` (1000 N/m, 3 cm down, 60 N) returned ABORTED "force limit
reached" after 4.58 s, at 43.6 N, with a `GetWrench` peak of 67.4 N.

### Probes (read-only, while the check ran)

- `logs/phase5b_probe_run4.log` (`/tmp/probe5b.py`, as in run 3) covered
  trials 4–8. It sampled ICON joints, F/T and the true peg pose.
- `logs/phase5b_probe_tilt_run4.log` (`/tmp/probe5b_tilt.py`) covered
  trials 9–10 and `jam`. It sampled the true peg pose with its tilt from
  vertical, `GetState.tool_pose` (TCP) and `GetWrench`, every 0.5 s.
- For trial 4, `/tmp/fk5b.py` ran `ComputeFk` (TCP) on the joints at
  contact and at the end of the search.

### Findings 10–12 (for the lead)

10. **The peg slips and pivots in the grasp. It does not slide on the
    face.** Trials 9 and 10 measured the same thing:
    - Free approach: the TCP is 59.1–59.4 mm above the peg bottom, and the
      peg tilts 0.6°.
    - Face `MoveToContact`: once the peg touches (`GetWrench` ≈ 2 N), the
      TCP keeps going down 12–14 mm while the peg bottom stays at
      z ≈ 0.0496. The TCP-to-peg-bottom distance drops to 45.3 mm, and
      the tilt grows to 8.0°. The force reaches 13 N only after that,
      probably when the peg top reaches the hand (not verified).
    - Search `MoveCompliant`: the TCP moves −3.8 to −4.2 mm in x relative
      to the peg, onto the hole axis. In trial 4 FK gives TCP x
      0.60247 → 0.59852 and z −0.23 mm. The peg bottom moves < 0.1 mm,
      and the tilt grows to 10.5–10.8°. `GetWrench` lateral stays at
      about 1 N, where 6 N was expected.

    So the spring pulls the hand, not the peg. The lower peg friction
    (B2) is not shown to be the cause. PhysX combines friction by
    averaging by default, so the finger-to-peg friction dropped by about
    as much as the peg-to-block friction. But at a 30 N grasp force
    (`ros2_hwm` gripper command; Isaac caps it at 70 N), slip at about 2 N
    looks more like the peg rotating about the two-finger contact line
    than like plain friction. Run 3 did not measure the TCP-to-peg
    offset, so whether 0.5/0.4 friction slips the same way is unknown.
    Trial 6 (bottom z 0.0500, the least penetration) slid into the hole.
    It was not tilt-probed.
11. **The F/T reading toggles between two levels while the TCP is still.**
    During the search the TCP holds to 0.1 mm while `GetWrench` z
    alternates between about 6.2 and 9.7 N. In `jam` the TCP holds
    z 0.0119, and z alternates between 20–22 N and 53–58 N, with lateral
    readings of ±4–10 N. The closed fingers creep 1.8 mm in +x along the
    table over 6 s. The 60 N limit trips on the high level (peak 67.4 N),
    so `jam` still ends as "force limit reached". The spring force is
    30 N. This is likely chattering contact between the fingers and the
    compliant table (not verified).
12. **Search deadline instead of jam.** 8/9 failed searches hit the 30 s
    deadline rather than the 1 s "settled" abort (trial 3 got it). The
    hand is still in position, so the force toggling in finding 11
    probably keeps `settled_for_seconds` from accumulating (not verified;
    the client cannot read that state variable).

Options, one decision per finding (not applied):
- (10) Choose one:
  - Make the grasp hold the peg: grasp force 70 N, a finger physics
    material with higher friction, or a grasp height where the peg top
    already touches the hand, so the peg cannot slide up and can only
    pivot.
  - Restore peg friction to 0.5/0.4 and lower only `hole_block` (for
    example 0.1/0.1, which averages to 0.3/0.25 against the peg), so the
    grip friction stays as before.
  - Weld the peg to the hand in Isaac while attached (a fixed joint).
    This is the least realistic option.
- (11) Choose one:
  - Raise the jam `max_force` above the measured 67 N, to 80 N (below
    the 85 N excessive-force limit).
  - Lower the jam stiffness to about 500 N/m (15 N spring).
  - Look into the toggling in Isaac: finger material and contact
    settings, or the solver iteration counts of the wrist F/T joint.
- (12) Follows from (10) and (11). Alternatively, a server-side jam test
  on TCP motion (< 0.5 mm in 1 s) instead of `settled_for_seconds`, as
  run 3's option (7).

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
- `tests/e2e/phase5b_check.py` (run 4): B2 search (2 mm, 2000 N/m) and B3
  jam `max_force` 60
- `ext/ros2_hwm/ros2_hwm_logic.{h,cc}`, `ros2_hwm_logic_test.cc`,
  `ros2_hwm.{h,cc}`: `VelocityEstimator` (B1)
- `tools/build_catalog_meshes.py`, `assets/objects/{peg,hole_block}/object.yaml`:
  friction keys (B2); `sim/tools/build_catalog.py` reads them
- `assets/objects/{peg,hole_block}/model.usd`: regenerated (B2)
- Run 5 (C.1): `sim/scene.py` (finger material), `container/config/icon.txtpb`
  (`default_force` 60), `container/services/robot/grasp.py` (`MAX_DEPTH_M`
  0.04), `tests/test_plan_grasp.py` (depth expectations)
- Run 5 (C.2 tooling): `tests/e2e/phase5b_check.py` (`--sweep`,
  wrench percentiles), `sim/tools/build_catalog.py` (`COMPLIANT_DAMPING`
  constant, still 600; USDs not regenerated)
- Run 6 (D): `container/services/robot/grasp.py` and
  `tests/test_plan_grasp.py` restored to their phase 5c content. The sweep
  changed `force.py`, `build_catalog.py` and the USDs temporarily; all
  three are restored to the committed files.
- Run 7 (E): `container/services/robot/force.py` (`ADMITTANCE_INERTIA`
  5 kg), `tests/test_force.py` (expects 5 kg), `sim/tools/build_catalog.py`
  (`COMPLIANT_DAMPING` 2000), `assets/objects/*/model.usd` (all 7
  regenerated), `tests/e2e/phase5b_check.py` (`_peg_above_block`, peg jam,
  `JAM_OFF_AXIS_M`)

Regenerated `gen/`:
- `intrinsic/icon/actions/{force_primitive,cartesian_admittance_action,cartesian_impedance,cartesian_impedance_reference_generators}_pb2*`
- `intrinsic/icon/control/…`, `intrinsic/icon/proto/matrix_pb2*`
- `intrinsic/manipulation/…`, `intrinsic/math/proto/twist_pb2*`
- `robot_service/robot_service_pb2_grpc.py` (docstrings only)

Other logs:
- `logs/phase5b_build_catalog.log`, `logs/phase5b_build_catalog_a.log`,
  `logs/phase5b_build_catalog_b.log`
- `logs/phase5b_addB_{build,stage,stage_vendor,docker_build}.log`
- `logs/phase5b_docker_build.log`
- `logs/phase5b_robot_service.log` (now from run 7)
- Run 6: `logs/phase5b_addD_docker_build*.log`,
  `logs/phase5b_build_catalog_d{600,2000}.log`
- Run 7: `logs/phase5b_addE_build_catalog.log`, `logs/phase5b_addE_docker_build.log`, `logs/phase5b_check_run7.log`
