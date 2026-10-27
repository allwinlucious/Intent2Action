# Phase 3 implementation notes

## Part 1 (`ros2_hwm`)

Status: **done.** All Part 1 checks pass: `phase3_hwm_check.py` (seven of
seven, in two consecutive runs), `build_intrinsic.sh --test`, and
`phase1_check.py` with `ROBOT_BACKEND=loopback`. As a regression check,
`phase2_check.py` (now built on `tests/e2e/harness.py`) also passes ten of
ten. Two plan items could not be done as written. Each has a small fix that
keeps the plan's intent; both need the orchestrator's approval (see "Plan
deviations" 1 and 2). No containers or Isaac processes are left running.

### Check output

`.venv/bin/python tests/e2e/phase3_hwm_check.py` (final run, exit 0; also in
`logs/phase3_hwm_check.log`):

```
isaac READY pid=24216
ICON ENABLED after 29.6 s
PASS move: Isaac joint error 0.00090 rad
PASS trajectory: Isaac final error 0.00090 rad
PASS gripper: GRASP done True width ICON 0.0000 Isaac 0.0000 m; RELEASE done True width ICON 0.0750 Isaac 0.0750 m
PASS wrench: force norm 5.753 N before, 0.000 N after taring
PASS stop: moving at 0.205 rad/s, stopped (< 0.01 rad/s) 0.020 s sim time after intrinsic.stop
isaac READY pid=26276
PASS isaac_restart: FAULTED ('DEADLINE_EXCEEDED: BeginCycle: robot_hwm: Timeout after 20000.1 ms') 25 s after kill; ClearFaults OK after 1 attempt(s), ICON ENABLED; move: Isaac joint error 0.00090 rad
PASS epoch: /sim/reset 'epoch 1' -> FAULTED ("'robot_hwm': simulation reset"); ClearFaults OK after 1 attempt(s), ICON ENABLED
elapsed 91.0
```

`container/build_intrinsic.sh --test` (exit 0; `logs/phase3_build.log`):

```
//ext/ros2_hwm:ros2_hwm_logic_test                              (cached) PASSED in 0.0s
Executed 0 out of 1 test: 1 test passes.
```

The test itself ran in the first successful build ("PASSED in 0.0s,
Executed 1 out of 1 test"). The sources have not changed since.

`ROBOT_BACKEND=loopback container/run.sh` then
`.venv/bin/python tests/e2e/phase1_check.py` (exit 0;
`logs/phase3_phase1_loopback.log`). The container log starts with
`robot backend: loopback` and has no `[ros2_hwm]` lines.

```
PASS objects
PASS fk_20: worst panda_link8 0.000000 mm / 0.000002 deg, right_gripper 0.000000 mm / 0.000002 deg
PASS fk_reference
PASS ik
PASS plan
PASS icon_status
PASS icon_parts
PASS icon_move
PASS icon_trajectory
elapsed 11.8
```

`phase2_check.py` regression (exit 0; `logs/phase3_phase2_regression.log`):
all ten checks pass, including `tf` with the added `panda_hand` target.

### Plan deviations

1. **`//ext` cannot use `hardware_module_binary` with visibility checks
   on (decision 5).** The macro links
   `//intrinsic_control/intrinsic/icon/utils:malloc_guard`. Its visibility
   is only `//intrinsic_control/intrinsic/icon:hardware_module_authors` and
   `:rtcl_users`. Those package groups list `//intrinsic/...`,
   `//third_party/...`, `//incode/...` and `//intrinsic_*/...`, and not
   `//ext/...`. The first build failed with
   `target '//intrinsic_control/intrinsic/icon/utils:malloc_guard' is not visible from target '//ext/ros2_hwm:ros2_hwm_main'`.
   The inventory says it did not check every dependency's visibility.

   The three possible fixes:
   * Add `//ext/...` to the package group. This modifies `~/intrinsic-core`,
     which is forbidden.
   * Write our own `cc_binary`. That still needs `malloc_guard`, because
     `hardware_module_main.cc` calls it.
   * Pass Bazel's `--nocheck_visibility`.

   **Done:** `build_intrinsic.sh` builds the six Intrinsic targets as before
   and then runs one more `bazel build --nocheck_visibility
   //ext/ros2_hwm:ros2_hwm_main`. The flag affects analysis only: that build
   had 12,930 action-cache hits and ran 81 actions, all for `//ext`. If this
   is not acceptable, the alternative is a one-line `//ext/...` entry in
   `hardware_module_authors`, which requires lifting the "do not modify
   `~/intrinsic-core`" rule.
2. **`num_acceptable_constant_readings: 4294967295` is impossible
   (decision 7, Inv §C.10).** The proto field is `uint32`, but ICON copies it
   into an `int` (`force_torque_sensor.h:167`,
   `force_torque_sensor.cc:454-457`), so 4294967295 becomes −1. ICON faults
   when `num_constant_readings() > num_acceptable_constant_readings`
   (`:717`), which is true from the first reading. The first run faulted at
   start-up with `INTERNAL: Force sensor reporting constant values: count= 0`.
   **Done:** `container/config/icon.txtpb` uses 2147483647 (INT32_MAX), the
   largest value that keeps the plan's intent of never faulting. Inv §C.10
   and decision 7 should say 2147483647.
3. **Commands are published from the tick thread (decision 11 says "the ROS
   thread").** Only the tick thread knows that a tick has returned and
   whether `ApplyCommand` ran in it (the command buffer's stamp equals the
   ticked stamp). It publishes `/joint_command`, and `/gripper_command` when
   that changed, right after `TickBlockingWithTimeout` returns. That thread
   is not an RT thread, and `rclcpp` publishers are thread-safe. The ROS
   executor thread only receives messages and runs the 1 s status timer.
4. **`sim/graphs.py` is unchanged.** `Graphs.set_tf_targets` already
   re-creates the Transform Tree node whenever the target list changes, so
   decision 13 only needed `HAND_PRIM` in `run_scene.py`'s target list. No
   Part 1 check reads `panda_hand` from `/tf`. The phase 2 `tf` check passes
   with it added, but Part 3 (`GetTruePose`) is the first user of that pose.
5. **Extra test file `tests/e2e/phase3_probe.py`.** The ROS probes that run
   in the image are not in the Part 1 file list. "Like phase 2's" implies a
   probe script, so this file reads one `/joint_states` message and calls
   `/sim/reset`. Both run over Fast DDS, because services do not work
   between DDS vendors.
6. **`harness.run_probe` writes `logs/<name>.log`.** Phase 2's probe logs
   are now `logs/phase2_hwm.log` and `logs/phase2_io.log` (before:
   `logs/phase2_phase2_*.log`).
7. **Generated protos.** Running `container/gen_protos.sh` on
   `intrinsic/icon/actions/simple_gripper.proto` and
   `tare_force_torque_sensor.proto` added them to `gen/` (part of decision
   4's list; the check needs them). The `proto_includes.sh` and
   `gen_protos.sh` changes in decision 4 are for `robot_service`'s protos
   (Part 3) and were not made.

### How the checks measure

* `move` / `trajectory`: ICON moves the arm, then a probe reads Isaac's
  `/joint_states`. The 0.0009 rad error is the same arm steady-state offset
  that phase 2's `reset` check reports.
* `gripper`: runs `intrinsic.simple_gripper` GRASP and then RELEASE, waiting
  up to 5 s for each, then reads the width from ICON
  (`linear_gripper_state.sensed_width`) and from Isaac (2 × finger).
  ICON's RELEASE commands `default_release_width` (0.075), not `max_width`.
  Both commands finish ("done"): with ICON's 30 N default force, Isaac
  reaches both targets to 0.1 mm. Phase 2's probe commands 20 N and still
  shows a 1.9 mm finger offset in the regression run, so the offset depends
  on the force (not investigated further).
* `stop`: sets ICON's speed override to 0.1, so a 0.7 rad joint-1 move is
  slow. After 2 s it starts `intrinsic.stop` in the same session. It
  measures the ICON part-status `timestamp_ns` (sim time) from just before
  the stop to the first status with every |sensed velocity| < 0.01 rad/s.
  The override is set back to 1.0 afterwards.
* `isaac_restart`: after the kill, ICON faults with its own 20 s
  `BeginCycle` timeout. The restarted Isaac's sim time starts again near 0.
  ICON accepts that after `ClearFaults`, because `Prepare` resets the clock.
  No "ignore older stamps" rule is needed, and none is implemented (decision
  11 has none).
* `epoch`: the probe calls `/sim/reset`, and the next `ReadStatus` returns
  "simulation reset". ICON reports it as `'robot_hwm': simulation reset`.

### `ros2_hwm` logging (decision 28)

The final run logged: init; epoch baseline; prepared/activated; motion
enabled; gripper command changes; taring started/done; lockstep ↔ "no state
for > 1 s"; epoch change; fault "simulation reset"; faults cleared; and a
10 s summary (`ticks=1683 dropped=0 …`). Nothing is logged per cycle in
`ReadStatus`/`ApplyCommand`, and there was no malloc-guard fault. The
entrypoint's `run` helper now goes through `container/logpipe.py`, which
prints `[<name>] <line>` and appends to `/var/lib/intrinsic/logs/<name>.log`
(10 MB × 3 rotation; in the loopback run, files exist for every service).

### Files

Created: `ext/ros2_hwm/{BUILD, ros2_hwm_config.proto, ros2_hwm_logic.h,
ros2_hwm_logic.cc, ros2_hwm_logic_test.cc, ros2_hwm.h, ros2_hwm.cc}`,
`container/logpipe.py`, `container/config/{icon_loopback.txtpb,
ros2_hwm.txtpb}`, `tests/e2e/{harness.py, phase3_hwm_check.py,
phase3_probe.py}`, `gen/intrinsic/icon/actions/{simple_gripper,
tare_force_torque_sensor}_pb2{,_grpc}.py`, this file.
Modified: `container/{build_intrinsic.sh, stage_intrinsic.sh,
entrypoint.sh, run.sh, Dockerfile}`, `container/config/icon.txtpb`,
`sim/run_scene.py`, `tests/e2e/phase2_check.py`.
Outside the repo: `~/intrinsic-core/ext → /home/mew/untitled-project/ext`
and `ext` in `~/intrinsic-core/.git/info/exclude` (`git status` there is
clean).

## Part 1 resolution (lead)

Both workarounds accepted: `--nocheck_visibility` for the separate `//ext`
Bazel call (keeps `~/intrinsic-core` unmodified), and
`num_acceptable_constant_readings: 2147483647` (ICON reads the uint32 into
an int).
