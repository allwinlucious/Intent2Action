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

## Part 2 (`scene_sync`, `camera_sim`, registry)

Status: **done.** `phase3_scene_check.py` passes five of five in two
consecutive runs. `phase3_hwm_check.py` still passes seven of seven with the
new image. No plan decision proved impossible. The items under "Plan
deviations and choices" are small; items 1, 6 and 8 need a look from the
orchestrator. No containers or Isaac processes are left running.

### Check output

`.venv/bin/python tests/e2e/phase3_scene_check.py` (second run, exit 0; also
in `logs/phase3_scene_check.log`). The first run printed the same results
(joints 0.00130 rad, restart 16 s), with the weaker restart check described
in item 9.

```
isaac READY pid=23764
ICON ENABLED after 29.9 s
PASS objects: sim_world: worst 0.00 mm, sources ground_truth; world: worst 0.00 mm, sources ground_truth
PASS spawn: extra_cube in both worlds 0.2 s after Spawn, gone 2.1 s after Delete
PASS joints: world franka joints within 0.00210 rad of ICON's
PASS camera: RGB + depth 848x480, fx 420.08, stamped +0.052 s after the clock read before Capture, depth median 2.048 m; DescribeCamera [(1, 'color', [1], (848, 480)), (4, 'depth', [2], (848, 480))]
PASS restart: objects re-created 13 s after docker restart; exactly one of each in both worlds: ['franka', 'franka_hand', 'hole_block', 'peg', 'plate', 'red_cube', 'table']
elapsed 64.0
```

`.venv/bin/python tests/e2e/phase3_hwm_check.py` (regression, exit 0;
`logs/phase3_hwm_check_part2.log`):

```
isaac READY pid=18600
ICON ENABLED after 29.7 s
PASS move: Isaac joint error 0.00090 rad
PASS trajectory: Isaac final error 0.00090 rad
PASS gripper: GRASP done True width ICON 0.0000 Isaac 0.0000 m; RELEASE done True width ICON 0.0750 Isaac 0.0750 m
PASS wrench: force norm 5.753 N before, 0.000 N after taring
PASS stop: moving at 0.205 rad/s, stopped (< 0.01 rad/s) 0.020 s sim time after intrinsic.stop
isaac READY pid=10048
PASS isaac_restart: FAULTED ('DEADLINE_EXCEEDED: BeginCycle: robot_hwm: Timeout after 20001.7 ms') 25 s after kill; ClearFaults OK after 1 attempt(s), ICON ENABLED; move: Isaac joint error 0.00090 rad
PASS epoch: /sim/reset 'epoch 1' -> FAULTED ("'robot_hwm': simulation reset"); ClearFaults OK after 1 attempt(s), ICON ENABLED
elapsed 91.1
```

In that run `scene_sync` lived through the Isaac kill and restart and the
reset. It deleted nothing, logged no error, and logged `sim epoch 1 (was 0)`.

### How the checks measure

* `objects`: waits until the five default objects exist in both worlds.
  Then `phase3_probe.py tf` (new command, Fast DDS in the image) collects
  Isaac's `/tf` for 2 s. Each object's `parent_t_this` (`ListObjects`, FULL)
  is compared with it. Each name must occur once, with parent `root` and
  `user_data["source"]` = `ground_truth`.
* `spawn`: `SimLink.Spawn(red_cube, extra_cube, (0.30, −0.30, 0))`, polls
  both worlds (5 s limit), then `SimLink.Delete`. The 2.1 s until it is gone
  is the 2 s absence rule.
* `joints`: an ICON point-to-point move to home + 0.3 rad on joint 1, then
  `world`'s `franka` `kinematic_object_component.joint_positions` against
  ICON's sensed joints. The world service fills these from the new registry
  entry: its log shows `Added robot subscription: arm` on
  `/icon/robot/robot_status_throttle`.
* `camera`: `ListAvailableCameras` = [d455]. `SimLink.GetClock`, then
  `Capture` (no timeout set, so the default 5 s applies). It requires
  sensors 1 and 4, raw, INTERLEAVED, INTENSITY/3/UINT8 and DEPTH/1/FLOAT32,
  848×480, the right byte counts, `fx` 420.08 ± 0.01, and
  `acquisition_time` after the clock read. Then `DescribeCamera` must match
  decision 17.
* `restart`: notes the object IDs, `docker restart intrinsic`, waits until
  every default object in both worlds has a new ID (that is, the new
  `scene_sync` replaced it), waits 3 s more, then counts names in both
  worlds.

### Plan deviations and choices

1. **`DescribeCamera` is `UNAVAILABLE` until a frame was captured.** This
   follows decision 17 as written. But Isaac publishes `/camera/camera_info`
   only with a requested frame (phase 2 addendum), so after start-up there
   is none until the first `Capture` (or a `scene_sync` spawn or delete,
   which also requests renders). The check therefore captures first. Phase
   4's calibration `Initialize` calls `DescribeCamera` first (Inv §E.2). It
   will fail unless something captured before. Fixing it needs a decision
   (e.g. `camera_sim` requests one frame at start-up).
2. **`Capture` also waits for `/camera/camera_info` stamped ≥ the
   request**, besides RGB and depth, so `camera_params` come from the same
   render. Isaac publishes all three in one render. Also: a request with
   `capture_result_location` (KV store) returns `UNIMPLEMENTED`;
   `post_processing_by_sensor_id` is ignored; `distortion_params` are left
   unset; `camera_t_sensor` is identity for both sensors.
3. **Absence is measured between `/tf` messages.** A name is deleted when a
   `/tf` message with objects arrives and the name was last seen more than
   2 s (wall) before. While no `/tf` arrives (Isaac stopped, or rendering),
   nothing is deleted. That is why the Isaac kill and restart above deleted
   nothing.
4. **`SetAttached` names are skipped in both worlds' batches.** This is the
   literal reading of decision 15 ("per world, skipping names marked
   attached"). Part 3 should confirm that `sim_world` may also stop
   following an attached object.
5. **`GetTruePose` knows only frames whose `/tf` parent is `world`**
   (objects, `panda_hand`, `Camera_Pseudo_Depth`). Other frames, such as
   `d455_optical` (identity under `Camera_Pseudo_Depth`), get `NOT_FOUND`.
6. **The mirrored `calib_board` sits 2.5 mm off Isaac's board.** Decision
   16 puts the catalog mesh's origin at the bottom centre, but
   `sim/scene.py` `attach_board` puts the board prim's origin at the box
   centre. The `/tf` pose is the prim origin, so in the worlds the board's
   box is shifted 2.5 mm along the board normal. This matters for phase 4
   only if it uses the world board geometry. `calib_board` `object.yaml`:
   dynamic, 0.2 kg (as `sim/scene.py`), colour white. `obstacle`: static,
   grey 0.5. `build_catalog.py` also wrote `calib_board/model.usd` (it
   globs every catalog directory); Isaac's board does not use it.
7. **Decision 4 is done now** (`proto_includes.sh` adds the project root;
   `gen_protos.sh` maps paths under it). `simlink.proto` imports
   `robot_service.proto`. All protos in decision 4's list are generated.
   `robot_service.proto` is unchanged: Part 3 adds `OperationInfo.state`
   and `detail` and must regenerate it. The existing `gen/` files came out
   byte-identical.
8. **Stale objects in the persisted worlds are not cleaned up.** On
   start-up `scene_sync` replaces only the names it sees on `/tf`. An
   object left in `world`/`sim_world` by an earlier run whose name is no
   longer in Isaac (for example the container killed while `extra_cube`
   existed) stays forever. `sim_world` is cloned only when it does not
   exist. The plan does not cover this.
9. **Extra files and edits:**
   * `.dockerignore` re-includes `sim/scenes/default.yaml`. `sim` is
     ignored, so the Dockerfile's copy of `default.yaml` failed.
   * `tests/e2e/phase3_probe.py` has the `tf` command, so `objects` checks
     against Isaac directly, not through `scene_sync`.
   * The `restart` check first only waited for the objects to be present.
     The worlds are persisted, so that could count before the new
     `scene_sync` ran. It now waits for new IDs.
10. **Robustness choices in `scene_sync`:**
    * `Spawn` validates the name (`[A-Za-z][A-Za-z0-9_]*`,
      `INVALID_ARGUMENT`) and the catalog (`NOT_FOUND`); a zero quaternion
      means identity.
    * Isaac result codes map to `NOT_FOUND`, `ALREADY_EXISTS` or
      `INVALID_ARGUMENT`, else `INTERNAL`, with Isaac's message.
    * Isaac calls time out after 10 s (`DEADLINE_EXCEEDED`), or return
      `UNAVAILABLE` if the service is missing.
    * After a failed world batch, it forgets objects that are missing from
      a world, so they are re-created on their next sight. Repeated errors
      are logged once per 10 s.
11. With an instance in the registry, the world service also subscribes to
    `/assets/instances/robot/joint_state` (its ROS joint-state path; nothing
    publishes there). This is harmless.

### Logging (decision 28)

Both processes log `<time> <LEVEL> <component> <message> key=value…` with
Python `logging`. `scene_sync` logs one line per object create and delete
(name, catalog, world, replaced, pose), `world created`, every SimLink call
(request ≤ 200 chars, ms, status, result or message), epoch changes,
unknown `/tf` names (once each) and errors. `camera_sim` logs start-up and
each capture (request stamp, image stamp, wait ms) or its failure. Both go
through `logpipe.py`: `scene_sync.log` and `camera_sim.log` exist in
`/var/lib/intrinsic/logs/` on the volume. Example:
`[camera_sim] 2026-10-27 23:39:51,890 INFO camera_sim capture stamp=11.212 image_stamp=11.214 wait_ms=148 sensors=[1, 4]`.

### Files (Part 2)

Created: `container/services/{scene_sync.py, camera_sim.py}`,
`config/robot_service.yaml`, `assets/objects/{calib_board,obstacle}/{mesh.glb,
object.yaml, model.usd}`, `tests/e2e/phase3_scene_check.py`, generated
`gen/robot_service/`, `gen/intrinsic/{perception, platform, icon/equipment}/`,
`gen/intrinsic/icon/actions/joint_jogging_pb2{,_grpc}.py`,
`gen/intrinsic/icon/proto/v1/jogging_service_pb2{,_grpc}.py`,
`gen/intrinsic/hardware/gpio/gpio_service_equipment_pb2{,_grpc}.py`.
Modified: `container/services/platform_stubs.py`, `container/Dockerfile`,
`container/entrypoint.sh`, `container/proto_includes.sh`,
`container/gen_protos.sh`, `tools/build_catalog_meshes.py`,
`tests/e2e/phase3_probe.py`, `.dockerignore`, this file. Image
`intrinsic-core:dev` rebuilt (no Bazel run in Part 2).

## Part 3 (Part 1 fixes, Part 2 follow-ups, `robot_service`)

Status: **done, with one plan item impossible.** `phase3_cuj_check.py` passes
eight of eight, `phase3_scene_check.py` five of five, `phase3_hwm_check.py`
seven of seven (all on the final image), and `pytest tests/` 19 of 19.
`PlanOnly`'s swept-volume bounds cannot be computed: the open-source planner
does not implement swept volumes (deviation 1). The fields are left unset,
and the orchestrator needs to decide what to do with them. Deviations 2 to 5
are fixes for problems found in live runs, and 6 and 7 change how two checks
measure. No containers or Isaac processes are left running.

### Check output

`.venv/bin/python tests/e2e/phase3_cuj_check.py` (exit 0;
`logs/phase3_cuj_check.log`):

```
isaac READY pid=18140
robot_service ready after 32.1 s
PASS logging: unreachable MoveToPose -> ABORTED "operations/MoveToPose-1: planning failed: IK solver couldn't find any solutions for the given constraints. [...]"; logged: 2026-10-28 00:22:54,943 WARNING robot_service rpc MoveToPose request={pose { x: 2.0 z: 0.5 qw: 1.0 } wait: true} ms=5042 status=ABORTED message=operations/MoveT[...]
PASS cuj2: LINEAR MoveToPose: true TCP 0.70 mm / 0.098 deg from target
PASS cuj7: MoveWaypoints 1.120 s / PlanOnly 1.114 s; Jog moved joint 1 by 0.1035 rad; ComputeFk vs tool_pose 0.0000 mm
PASS cuj4: Stop -> operations/MoveJoints-7 CANCELLED ('cancelled (Stop, CancelOperation or ResetScene)'); next Home done in 0.66 s
PASS grasp: Grasp width 50.0 mm gripping; cube rose 49.9 mm
PASS obstacle: midpoint in collision; PlanOnly 1.12 s around the obstacle; 20 samples of the executed path collision-free
PASS cuj5: fault "'robot_hwm': simulation reset"; Home 0.00 s later -> FAILED_PRECONDITION "robot faulted: 'robot_hwm': simulation reset; call ClearFaults"; after ClearFaults Home done in 0.03 s
PASS misc: SetPayload(0.5) ok; SetSpeed(0.5) 0.41 -> 0.82 s; ResetScene ok; ListObjects ['hole_block', 'peg', 'plate', 'red_cube', 'table'] ground_truth; GetImage RGB/DEPTH 848x480 PNG; GetPointCloud 407040 points, z 0.81..5.43 m
elapsed 85.0
```

`.venv/bin/python tests/e2e/phase3_scene_check.py` (exit 0;
`logs/phase3_scene_check_final.log`):

```
isaac READY pid=8688
ICON ENABLED after 29.5 s
PASS objects: sim_world: worst 0.00 mm, sources ground_truth; world: worst 0.00 mm, sources ground_truth
PASS spawn: extra_cube in both worlds 0.2 s after Spawn, gone 2.1 s after Delete
PASS joints: world franka joints within 0.00000 rad of ICON's
PASS camera: RGB + depth 848x480, fx 420.08, stamped +0.052 s after the clock read before Capture, depth median 2.048 m; DescribeCamera [(1, 'color', [1], (848, 480)), (4, 'depth', [2], (848, 480))]
PASS restart: objects re-created 13 s after docker restart; exactly one of each in both worlds: ['franka', 'franka_hand', 'hole_block', 'peg', 'plate', 'red_cube', 'table']
elapsed 63.7
```

`.venv/bin/python tests/e2e/phase3_hwm_check.py` (exit 0;
`logs/phase3_hwm_check_final.log`):

```
isaac READY pid=19340
ICON ENABLED after 29.7 s
PASS move: Isaac joint error 0.00090 rad
PASS trajectory: Isaac final error 0.00090 rad
PASS gripper: GRASP done True width ICON 0.0000 Isaac 0.0000 m; RELEASE done True width ICON 0.0750 Isaac 0.0750 m
PASS wrench: force norm 5.753 N before, 0.000 N after taring
PASS stop: moving at 0.205 rad/s, stopped (< 0.01 rad/s) 0.020 s sim time after intrinsic.stop
isaac READY pid=23556
PASS isaac_restart: FAULTED ('DEADLINE_EXCEEDED: BeginCycle: robot_hwm: Timeout after 20001.3 ms') 25 s after kill; ClearFaults OK after 1 attempt(s), ICON ENABLED; move: Isaac joint error 0.00090 rad
PASS epoch: /sim/reset 'epoch 1' -> FAULTED ("'robot_hwm': simulation reset"); ClearFaults OK after 1 attempt(s), ICON ENABLED
elapsed 91.5
```

`.venv/bin/python -m pytest tests/` (`logs/phase3_pytest.log`): 19 passed
(13 in `tests/test_robot_service.py`). `container/build_intrinsic.sh --test`
after fixes 1 to 5 passed (`logs/phase3_part3_build.log`).

### Part 1 review fixes (1 to 6)

All six are applied as listed. After the rebuild, `phase3_hwm_check.py`
passed 7/7 (`logs/phase3_hwm_check_part3.log`) before any Part 3 work began.
* Fix 1: `last_gripper_` is reset in `Prepare()`, and in the tick thread
  when the baseline epoch changes. The container log shows the open command
  re-sent after the Isaac restart and after `/sim/reset`.
* Fix 2: a gripper command is published only when its force is > 0 and it
  differs from the last one sent. No zero-force command appears in the log.
* Fix 3: `ReadStatus` keeps its own copy of the baseline epoch and clears
  `taring_` when it changes. It reads only an atomic and does not allocate.
* Fix 4: tick failures log with `LOG_EVERY_N_SEC(WARNING, 5)`, and a dropped
  entry logs one line (`tick entry dropped stamp_ns=… after N attempts`).
  `tickStopping()` is checked before `Reset` and again after the back-off.
* Fix 5: the pairer is cleared on the "no state for > 1 s" transition.
* Fix 6: `logpipe.py` prefixes each file line with
  `YYYY-MM-DD HH:MM:SS.mmm` unless the line already starts with an ISO time.
  stdout is unchanged. The Python services already used the required
  `basicConfig` format.

### Part 2 follow-ups (7 to 9)

* Follow-up 7: `camera_sim` has a thread that publishes `/camera/request`
  (stamp = latest `/clock`) once per second until a `camera_info` has
  arrived. It logs `camera_info received width=… fx=…`. In the scene check's
  container log, that line appears at start-up, before the first `Capture`.
  The check itself still calls `Capture` before `DescribeCamera`.
* Follow-up 8: `calib_board`'s mesh is now centred (`object.yaml` has
  `origin: center`). Its `mesh.glb` and `model.usd` were regenerated with
  `build_catalog.py` and pulled back with `sync_isaac.sh --pull-catalog`.
  No other catalog file changed.
* Follow-up 9: 5 s after its first `/tf`, `scene_sync` deletes from both
  worlds every `ground_truth` object that is not on `/tf`. Each deletion is
  logged (`reason=stale_not_on_tf`). No check covers this, and it was not
  tested by hand.

### `robot_service`

`container/services/robot_service.py` runs `RobotService`, `SimService` and
`Operations` on 127.0.0.1:18100. The helpers in `container/services/robot/`
are `icon.py`, `motion.py`, `ops.py`, `gripper.py` and `png.py`. The
entrypoint starts it in the isaac backend only, with CycloneDDS `rclpy` for
`/clock`. `container/stage_vendor.sh` copies the nine ICON Python client
modules from `~/intrinsic-core` into `container/stage/vendor/` and checks
that they import. Run it after `stage_intrinsic.sh`, which wipes `stage/`.
The image adds `absl-py==2.5.1` and `/opt/app/vendor`. Decisions 19 to 27
are implemented as written, apart from the deviations below. Decision 28:
an interceptor logs every RPC (`rpc <Method> request={≤200 chars} ms=
status= message=`), each operation logs its end state, and errors carry
specific codes. `OperationInfo.state`/`detail` were already in
`robot_service.proto` and `gen/`.

### Plan deviations and choices (Part 3)

1. **Impossible: `PlanOnlyResponse.swept_min`/`swept_max`.** With
   `compute_swept_volume`, the planner returns `ComputeSweptVolumeFromPath is
   not implemented`, so even plans that would otherwise succeed fail.
   `PlanOnly` therefore leaves both fields unset. Needs a decision: drop
   the fields, or define bounds `robot_service` can compute itself (for
   example, from FK of the robot's links along the discretized trajectory).
2. **LINEAR pose targets are replanned to the reached joints.** Most LINEAR
   moves failed with `Could not generate Cartesian linear path to the desired
   motion target joint configuration …`. The planner resolves the pose to a
   joint target by IK. Its straight-line path IK from the start then ends at
   another solution of the redundant arm, about 3 mrad away, which is over
   its tolerance. The error also arrives as a structured
   `MotionPipelineError` in `grpc-status-details-bin`. `Planner.plan`
   replans with that segment's target set to the reached joints, which is
   what the planner's message advises. This gives the same pose (FK error
   0.0 mm), still LINEAR. With blending, the failing range is reported as
   (first, last), and the reached joints belong to the last segment.
   `container/gen_protos.sh` generated `motion_planning_error_pb2` for this.
3. **A motion finishes only when the arm has reached the trajectory end.**
   Isaac's joint drives lag ICON's setpoints by up to about 0.13 rad after a
   fast move. The next trajectory starts from the sensed joints, and ICON
   rejects it ("Difference between previous setpoint and position command
   (0.131954) exceeds 0.1") and disables the arm. After ICON reports the
   action done, `robot_service` waits until every sensed joint is within
   0.01 rad of the last point (at most 2 s of sim time; otherwise
   `DEADLINE_EXCEEDED`). `MotionResult.duration_s` still ends at ICON's
   "done", so it stays comparable with `PlanOnly`.
4. **`DetachObject` waits for `world` (with GROUND_TRUTH).** `scene_sync`
   skips attached objects, so right after detach `world` still has the cube
   inside the hand. The next plan then fails with "Invalid initial joint
   configuration … franka_hand / red_cube". `DetachObject` waits until the
   object's `world` pose is within 5 mm of its true pose (at most 5 s;
   otherwise `DEADLINE_EXCEEDED`). `AddObject` and `RemoveObject` likewise
   wait up to 5 s for `world`.
5. **`GetState.speed_factor` is the value `robot_service` applied.** ICON's
   `GetStatus.current_speed_override` reads 0. The override itself works:
   at 0.5, the measured duration doubled (0.41 s to 0.82 s).
6. **Obstacle check: the executed path is sampled.** `PlanOnlyResponse` has
   no path. After `PlanOnly`, the check runs the same `MoveWaypoints`,
   samples `GetState` joints 20 times at `planned_duration_s` / 20, and
   calls `CheckCollisions` on each sample. It first checks that the
   straight-line midpoint (inside the obstacle) is in collision, so the
   check cannot pass trivially.
7. **`GetPointCloud` z bound (0.2, 6) m, not (0.2, 4) m.** The floor is
   1.65 m below the camera, which is pitched 45°. With a 59.5° vertical
   field of view, the top image row meets the floor at 5.43 m optical
   depth, which matches the measured maximum. The server keeps every finite
   depth > 0, as decision 26 says.
8. **`phase3_hwm_check.py` pauses `robot_service` for `isaac_restart` and
   `epoch`.** The decision 24 watcher clears faults after 2 s. With Isaac
   dead, `ClearFaults` leaves ICON DISABLED (and the HWM times out again
   20 s later), so the check saw DISABLED instead of FAULTED. These checks
   test ICON's own fault handling, so the check sends `robot_service`
   SIGSTOP and then SIGCONT (`wait -n` in the entrypoint ignores stopped
   children). Behaviour to decide on: while Isaac is down, the watcher
   cycles ICON between FAULTED and DISABLED about every 22 s. It could clear
   faults only while `/clock` advances.
9. **Smaller choices:**
   * `READY` in decision 25 is ICON's `ENABLED`, because ICON has no READY
     state.
   * Jog: `GetStatus` has no velocity limits, so the joint maximum comes
     from `GetConfig` application limits, capped at ICON's jogging limit of
     30°/s.
   * `wait=true` on an operation that failed returns the operation's error
     as the RPC status. A successful or still-running operation returns
     the Operation.
   * `planned_duration_s` = planner duration / speed factor. The motion
     timeout (2 × planned + 2 s of sim time) uses it.
   * `blend_radius`: the planner takes one blending setting per request.
     The largest radius becomes Cartesian corner rounding, and every
     waypoint must be LINEAR (otherwise `INVALID_ARGUMENT`).
   * `colliding_pairs` holds the planner's `collision_debug_msg` text,
     because `CheckCollisions` returns no pairs.
   * `GetPointCloud` is about 4.9 MB, so clients need a larger
     `grpc.max_receive_message_length` (the check uses 64 MB).
   * The calibration board attach and detach use SimLink and then the world
     reparent, as in the design doc. Phase 4 will exercise them.
10. **Name clash.** `container/services/robot_service.py` would shadow the
    generated `robot_service` package, because both proto directories are
    namespace packages and a regular module wins over them. The fix:
    `robot_service.py` imports the package with its own directory removed
    from `sys.path` first, and `scene_sync.py` drops that directory. In the
    entrypoint, `PYTHONPATH` prepends to ROS's value instead of replacing it.
11. **WSL clock jump.** In one dev run the container's wall clock stepped
    back 2.7 s (log times went from 00:16:21.6 to 00:16:18.9). The world
    service then rejected every update older than the last stamp
    ("timestamp … is newer than the given timestamp") until the clock
    caught up. `scene_sync` logged this every 55 ms, because the repeat
    filter compared whole messages. It now ignores digits when comparing,
    so repeats are logged once per 10 s.

### Files (Part 3)

Created: `container/services/robot_service.py`,
`container/services/robot/{__init__, icon, motion, ops, gripper, png}.py`,
`container/stage_vendor.sh`, `tests/test_robot_service.py`,
`tests/e2e/phase3_cuj_check.py`,
`gen/intrinsic/motion_planning/proto/v1/motion_planning_error_pb2{,_grpc}.py`.
Modified: `ext/ros2_hwm/{ros2_hwm.cc, ros2_hwm.h}`, `container/logpipe.py`,
`container/services/{camera_sim.py, scene_sync.py}`,
`container/{Dockerfile, entrypoint.sh}`, `tools/build_catalog_meshes.py`,
`assets/objects/calib_board/{mesh.glb, object.yaml, model.usd}`,
`tests/e2e/phase3_hwm_check.py`, this file. Image `intrinsic-core:dev`
rebuilt (Bazel through `build_intrinsic.sh --test`, after fixes 1 to 5).

## Part 3 resolution (lead)

*   Swept volume: `PlanOnly` computes `swept_min`/`swept_max` itself:
    `ComputeFk` of the TCP and every arm link origin on every 10th
    `discretized` trajectory point, bounds enlarged by 0.08 m (link
    radius). Documented in the proto comment as an approximation. Done
    as the first step of phase 4.
*   Fault watcher: clears faults only if `/clock` advanced within the last
    1 s (Isaac running); otherwise it leaves the fault visible. Done as
    the first step of phase 4; `phase3_hwm_check.py` then no longer needs
    to pause `robot_service`.
