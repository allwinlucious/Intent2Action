# Phase 2: the Isaac side (`sim/run_scene.py`)

Part of [robot_platform.md](../robot_platform.md). Facts with citations:
[phase_2_inventory.md](phase_2_inventory.md) ("Inv §n"); experiment
scripts in `C:\isaac-sim-work\phase2\scripts\` ("expN"). Earlier:
[phase_0_5_notes.md](phase_0_5_notes.md), [phase_1_notes.md](phase_1_notes.md).
D7 = design owner decision "Isaac I/O only through the ROS 2 bridge's own
components". The design doc already reflects this plan (G5, channel table,
"Timing: Reset", Alternative I); it needs no edit in this phase.

## Goal and done criteria

`sim/run_scene.py` runs Isaac headless with the default scene and talks
only ROS 2. `tests/e2e/phase2_check.py` (WSL) starts it, runs two
container probes, and prints `PASS`/`FAIL`:

| Name | Condition |
|---|---|
| `lockstep` | `phase2_peer_hwm.py` (CycloneDDS) acts as a fake `ros2_hwm` for 2000 cycles: for each `/joint_states` + `/wrist_wrench` pair with the same stamp *t* (match within 1 µs) it publishes `/joint_command` stamped *t*: `panda_joint1` = home + 0.3·sin(2π·0.5·t) rad with velocity feed-forward, other joints home. Pass: `missed == 0` (a missed cycle = a state stamp the peer saw with no command reply within 1 s); joint-1 tracking error (command at *t* vs. state at the next stamp) max < 0.01 rad. RTF printed |
| `free_run` | the peer pauses for 3 s after cycle 1000, then resumes. Pass: state stamps keep advancing during the pause (> 0.2 s of sim time; the first 1 s is the free-run timeout and free-run cycles run at RTF ≈ 0.4), and lockstep resumes within 10 cycles after the peer resumes |
| `gripper` | `/gripper_command` width 0.0 m, effort 20 N → `panda_finger_joint1` < 0.002 m within 1 s sim time; width 0.08 → > 0.038 m |
| `wrench` | static `/wrist_wrench` force (frame `panda_hand`) within 0.3 N of (−2.394, −0.085, +5.231) N at home (the hand's load, Inv §2.4); after `/sim/attach_board` its magnitude rises by 0.2 kg × 9.81 ± 5 %; after `/sim/detach_board` it returns within 0.1 N |
| `tf` | `/tf` (parent `world`) has frames `table`, `red_cube`, `plate`, `peg`, `hole_block`, `Camera_Pseudo_Depth` and `d455_optical`; `table`, `hole_block` and `Camera_Pseudo_Depth` within 1 mm / 0.1° of the scene file; the dynamic objects within 5 mm of their scene positions after 2 s of settling |
| `camera` | after a `/camera/request`, the container receives `/camera/rgb` (`rgb8` 848×480), `/camera/depth` (`32FC1` 848×480, metres) and `/camera/camera_info` (`K` = [420.08, 0, 424; 0, 420.08, 240; 0, 0, 1] ± 0.1), `frame_id` `d455_optical`, stamped ≥ the request stamp |
| `camera_fresh` | after moving `panda_joint1` by +0.5 rad: a requested RGB frame differs from the one before the move (mean absolute pixel difference > 2) |
| `entities` | Fast DDS client: `GetEntities` (filter `^/World/objects/[^/]+$`) lists `/World/objects/red_cube`; `SpawnEntity` name `/World/objects/extra_cube`, uri `C:/isaac-sim-work/repo/assets/objects/red_cube/model.usd`, succeeds and `extra_cube` appears on `/tf` within 5 s wall; `DeleteEntity` removes it (client timeouts 10 s per call) |
| `reset` | after moving joint 1 and spawning `extra_cube`: `/sim/reset` returns success; joints at home within 0.001 rad after 1 s sim time with no peer commanding; `extra_cube` gone; default objects back at their scene poses; `/sim/epoch` = previous + 1 |
| `standalone` | `run_scene.py` with no container running keeps stepping for 30 s and its log contains no line with `Error` or `Traceback` |

Also recorded in `phase_2_notes.md`: real-time factor with and without one
camera request per second, render cost, and `camera_request_renders`.

## Decisions made in this plan

1.  **D7 compliance and its one exception.** State, wrench, clock, joint
    command, gripper command, camera request, epoch, `/tf`, camera and
    `simulation_interfaces` use bridge components (on-demand OmniGraph
    graphs, Camera Helper, `isaacsim.ros2.sim_control`). The script's own
    `rclpy` serves only `/sim/reset`, `/sim/attach_board`,
    `/sim/detach_board` (`std_srvs/Trigger`): the bridge's service nodes
    reply in the evaluation that receives the request, before the script
    can act (Inv §6.1).
2.  **Noisy depth is produced by `camera_sim` (phase 3b), not Isaac.**
    Isaac's depth sensor gives 12–15 % valid pixels, only along intensity
    edges of the colour image, with values ≈ 10⁴ × metres (Inv §4.3); a
    real D455 has an IR projector and gives dense depth (Intel D455
    datasheet, external). The Camera Helper publishes ground-truth depth
    only (Inv §4.5). The depth-sensor post-process is not enabled.
3.  **Renders: on request and every 0.5 s of wall time.** Renders cost
    up to 115–210 ms (an upper bound: the GPU was shared during
    measurement, Inv §4.4). `run_scene.py` renders (a) on a
    `/camera/request` (`std_msgs/Header`; `stamp` = the earliest sim time
    the image must show): it keeps stepping until sim time ≥ that stamp,
    then does `camera_request_renders` renders; (b) every
    `render_period_wall_s` = 0.5 s, because `sim_control`'s handlers and
    the Camera Helper writers run only inside renders (Inv §5.2, §3.4);
    images are therefore also published at about 2 Hz without requests.
    The synchronous rendering flags of exp2e are set (Inv §4.4).
4.  **Physics:** 1 kHz, 4 substeps per 250 Hz cycle (`config/timing.yaml`);
    USD writeback stays on (rendering correctness).
5.  **Reset without a timeline stop.** `/sim/reset`: delete the direct
    children of `/World/objects`, re-create the defaults, set the robot
    to home (positions, drive targets, zero velocities; gripper open),
    set the held free-run command to home, and publish `/sim/epoch` + 1.
    Simulation time continues.
6.  **Epoch:** `std_msgs/UInt32` on `/sim/epoch`, reliable,
    transient-local, depth 1, bridge generic publisher; value 0 published
    once after start-up, then once after each reset.
7.  **Gripper command:** `sensor_msgs/JointState` on `/gripper_command`
    (`name ["panda_finger_joint1"]`, `position [width/2]`,
    `effort [max force N]`); a new command = a new `outputs:timeStamp` on
    its Subscribe Joint State node. Only `panda_finger_joint1` has a drive
    (Inv §1.3); `run_scene.py` sets its target and drive max force =
    effort clamped to [0, 70] N (Franka Hand continuous grasp force 70 N,
    Franka datasheet, external).
8.  **Wrench:** force and torque = row 8 of `get_measured_joint_forces()`
    (Inv §2.4), each rotated by `R(localRot1)` of `panda_hand_joint`, then
    negated, so `/wrist_wrench` reports the **load on the sensor** (the
    hand's weight points along +z of `panda_hand` at home), as a real F/T
    sensor does. If `panda_hand_joint`'s `localPos1` is not zero, shift the
    torque to the `panda_hand` origin (τ′ = τ − r × F); record the value.
    `frame_id` `panda_hand`, stamp = the cycle's sim time.
9.  **Stamps:** every message of a cycle carries `sim.current_time`.
    Graph publishers use Read Simulation Time (= `current_time`, Inv §3.4);
    the script-written wrench stamp uses `sec = floor(t)`,
    `nanosec = round((t − sec)·1e9)`. All matching (Isaac command wait,
    peers, later `ros2_hwm`) compares stamps within 1 µs.
10. **`/tf`:** bridge Publish Transform Tree, `parentPrim` blank (frame
    `world`), Read Simulation Time for its stamp, evaluated every 8th
    cycle (31.25 Hz sim time). Targets: the direct children of
    `/World/objects` and the camera prim
    `/World/d455/RSD455/Camera_Pseudo_Depth`; re-read after every render
    (spawn/delete only happen inside renders) and after a reset. A bridge
    Publish Raw Transform Tree node publishes `Camera_Pseudo_Depth` →
    `d455_optical` = rotation 180° about x (USD camera −z forward, y up →
    ROS optical z forward, y down) in the same graph. `/clock`: ROS2
    Publish Clock in the `state` graph, every cycle.
11. **Objects** are direct children `/World/objects/<name>`; catalog USDs
    carry the custom attribute `catalog:name`.
12. **Catalog USDs are written with `pxr`** from `mesh.glb` (Isaac's
    bundled `trimesh`), Z-up, metres, with physics; not with the asset
    converter (Y-up, cm, no physics, Inv §7.2).
13. **Configuration files are YAML read with PyYAML:** Isaac has it in
    `$ISAAC/exts/omni.pip.compute/pip_prebundle` (with `trimesh` and
    `cv2`); the image gets `python3-yaml`; the WSL venv gets `pyyaml`.
14. **DDS:** `<maxMessageSize>1400</maxMessageSize>` in the Fast DDS UDP
    transport (Inv §3.3), both sides. `cyclonedds.xml` is unchanged (its
    topics are < 1.3 KB; Cyclone's large-message behaviour is untested).
15. **`simulation_interfaces` in the container:** built from source at tag
    `1.1.0`, the version Isaac bundles (Inv §5.1), so both sides use the
    same message definitions (Humble matches types by name only).
16. **Files reach Windows by copy:** `tools/sync_isaac.sh` copies `sim/`,
    `config/` and `assets/` to `C:\isaac-sim-work\repo\` and
    `container/dds/fastdds.xml` to `C:\isaac-sim-work\fastdds.xml`;
    `tools/sync_isaac.sh --pull-catalog` copies generated
    `assets/objects/*/model.usd` back. Isaac runs
    `python.bat C:\isaac-sim-work\repo\sim\run_scene.py`; `run_scene.py`
    puts `C:\isaac-sim-work\repo` on `sys.path`.

## Constraints

*   No Bazel; no `git commit`; Docker via `sg docker -c "…"`; no `pkill -f`
    or `killall`. Windows processes are stopped with
    `taskkill.exe /PID <pid> /T /F`, using the PID `run_scene.py` prints.
*   `sim/` code runs on Isaac's Python 3.11; container probes on the
    image's Python 3.10; style as before
    (`pyink --pyink-indentation=2 --line-length=80`, `isort --profile=google`).
*   Image rebuild = `sg docker -c "docker build -f container/Dockerfile -t intrinsic-core:dev ."`
    only (`container/stage/` exists; do not run `build_intrinsic.sh`).

## Target files

### 1. `config/timing.yaml` (Create)

```yaml
control_hz: 250
substeps: 4            # physics_dt = 1 / (control_hz * substeps)
free_run_timeout_s: 1.0
free_run_wait_s: 0.005
render_period_wall_s: 0.5
camera_request_renders: 3   # step 5 measures the minimum and updates this
tf_every_n_cycles: 8
stamp_match_tolerance_s: 1.0e-6
```

### 2. `container/dds/fastdds.xml` (Modify)

Decision 14.

### 3. `tools/sync_isaac.sh` (Create)

Decision 16 (plain `cp -r`/`rsync`-free; `mkdir -p` the targets).

### 4. `tools/build_catalog_meshes.py` (Create; WSL venv)

`pip install pyyaml manifold3d opencv-python-headless` into `.venv`
(record versions in `requirements-dev.txt`, Create). Writes
`assets/objects/<name>/mesh.glb` and `object.yaml`:

| Name | Shape (metres) | Static | Mass kg | Colour RGB |
|---|---|---|---|---|
| `table` | box 1.2 × 0.8 × 0.75 | yes | — | 0.55, 0.45, 0.35 |
| `red_cube` | box 0.05³ | no | 0.10 | 0.8, 0.1, 0.1 |
| `plate` | cylinder r 0.09, h 0.012 | no | 0.30 | 0.9, 0.9, 0.9 |
| `peg` | cylinder r 0.010, h 0.08 | no | 0.05 | 0.2, 0.4, 0.8 |
| `hole_block` | box 0.08 × 0.08 × 0.05 minus a cylinder r 0.011, depth 0.04, open at the top centre | yes | — | 0.3, 0.3, 0.3 |

Mesh origin: bottom centre. `trimesh.creation.box/cylinder`,
`trimesh.boolean.difference(..., engine="manifold")` for the hole; assert
the result is watertight. `object.yaml` keys: `static`, `mass`, `color`,
`collision` (`convexHull` for dynamic objects, `none` = triangle mesh for
static ones; PhysX allows concave meshes only on static bodies).

### 5. `tools/make_charuco.py` (Create; WSL venv)

`cv2.aruco.CharucoBoard((7, 5), 0.03, 0.022, getPredefinedDictionary(DICT_5X5_100)).generateImage((1400, 1000), marginSize=0)`
→ `assets/calib/charuco_7x5.png` (Inv §7.3, without margin so squares are
exactly 30 mm on a 0.21 × 0.15 m face).

### 6. `sim/tools/build_catalog.py` (Create; Isaac Python)

Reads `C:\isaac-sim-work\repo\assets\objects\*\` and writes `model.usd`
next to each `mesh.glb`: Z-up, `metersPerUnit` 1, default prim `/Object`
(Xform, attribute `catalog:name`), child `UsdGeom.Mesh` (points, face
indices from `trimesh`), `displayColor`, `UsdPhysics.CollisionAPI` +
`MeshCollisionAPI` (`approximation` from `object.yaml`), and for dynamic
objects `RigidBodyAPI` + `MassAPI`. Run:
`tools/sync_isaac.sh && cmd.exe /c "cd /d C:\isaac-sim && python.bat C:\isaac-sim-work\repo\sim\tools\build_catalog.py" && tools/sync_isaac.sh --pull-catalog`.

### 7. `sim/scenes/default.yaml` (Create)

```yaml
robot: {position: [0, 0, 0]}             # Franka base at the world origin (phase 1 loader)
ground_z: -0.75
objects:                                  # bottom-centre position (m), yaw (deg)
  table:      {catalog: table,      position: [0.45, 0.0, -0.75], yaw: 0}
  red_cube:   {catalog: red_cube,   position: [0.50, 0.15, 0.0],  yaw: 0}
  plate:      {catalog: plate,      position: [0.50, -0.20, 0.0], yaw: 0}
  peg:        {catalog: peg,        position: [0.35, 0.30, 0.0],  yaw: 0}
  hole_block: {catalog: hole_block, position: [0.60, 0.30, 0.0],  yaw: 0}
camera: {position: [1.3, 0.0, 0.9], euler_deg: [0, 45, 180]}   # Inv §4.1; converted with isaacsim.core.utils.rotations.euler_angles_to_quat(..., degrees=True)
```

### 8. `sim/scene.py` (Create)

*   `build(stage, scene, asset_root)`: ground plane at `ground_z`; Franka
    `<asset root>/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd` at
    `/World/franka` (Inv §1.1–1.2); objects at `/World/objects/<name>`
    referencing `C:/isaac-sim-work/repo/assets/objects/<catalog>/model.usd`;
    D455 `<asset root>/Isaac/Sensors/Intel/RealSense/rsd455.usd` at
    `/World/d455` with `physics:kinematicEnabled = True` on
    `/World/d455/RSD455` (Inv §4.1). Camera:
    `isaacsim.sensors.camera.Camera(prim_path="/World/d455/RSD455/Camera_Pseudo_Depth", resolution=(848, 480))`,
    `initialize()`, `set_clipping_range(0.05, 20.0)`; its
    `get_render_product_path()` feeds the Camera Helpers. Record the
    resulting intrinsics.
*   `reset_objects(stage, scene)`: delete the direct children of
    `/World/objects`, re-create the defaults.
*   `attach_board(stage, robot)`: if `/World/objects/calib_board` exists →
    return `(False, "already attached")`. Board =
    `/World/objects/calib_board`: box 0.21 × 0.15 × 0.005 m, mass 0.2 kg,
    created at the current world pose of `panda_hand` ⊕ (0, 0, 0.12)
    (beyond the fingertips) so the joint starts satisfied; its top face
    (the face pointing along the hand's +z) carries `charuco_7x5.png`
    through `UsdPreviewSurface` ← `UsdUVTexture` ← `UsdPrimvarReader_float2`
    with an `st` primvar (Inv §7.3). Joint
    `/World/objects/calib_board/attach_joint`: `UsdPhysics.FixedJoint`,
    body0 `/World/franka/panda_hand`, body1 the board,
    `localPos0` (0, 0, 0.12), `localPos1` (0, 0, 0), both rotations
    identity (Inv §6.2).
*   `detach_board(stage)`: if absent → `(False, "not attached")`; else
    remove `/World/objects/calib_board` (joint included).

### 9. `sim/graphs.py` (Create)

On-demand graphs (`GRAPH_PIPELINE_STAGE_ONDEMAND`, `OnTick` entry, Inv
§3.4), QoS JSON written into `inputs:qosProfile` (reliable, keep-last 1,
volatile unless stated):
*   `state`: Read Simulation Time → Publish Joint State (`/joint_states`,
    target `/World/franka`), generic Publisher `geometry_msgs/WrenchStamped`
    (`/wrist_wrench`; flat inputs written by the script, Inv §3.4), ROS2
    Publish Clock (`/clock`).
*   `command`: Subscribe Joint State `/joint_command` and Subscribe Joint
    State `/gripper_command`.
*   `camera_request`: generic Subscriber `std_msgs/Header` on
    `/camera/request` → `omni.graph.action.Counter` (`outputs:count`
    tells the script a request arrived; the script reads the stamp from
    the subscriber's outputs).
*   `epoch`: generic Publisher `std_msgs/UInt32` `/sim/epoch`
    (transient-local).
*   `tf`: Read Simulation Time → Publish Transform Tree (decision 10) and
    Publish Raw Transform Tree (`Camera_Pseudo_Depth` → `d455_optical`).
*   `camera`: Camera Helper `rgb` (`/camera/rgb`), Camera Helper `depth`
    (`/camera/depth`), CameraInfo Helper (`/camera/camera_info`),
    `frameId` `d455_optical`; evaluated once to create the writers (Inv
    §3.4, §4.5).

### 10. `sim/run_scene.py` (Create)

1.  `sys.path` += `C:\isaac-sim-work\repo` and the `pip_prebundle` path.
    Environment as `sim/ros_probe.py` (ROS distro, RMW, PATH,
    `FASTRTPS_DEFAULT_PROFILES_FILE=C:\isaac-sim-work\fastdds.xml`, domain
    0). `SimulationApp` headless with `--/app/vulkan=false` and the exp2e
    synchronous rendering flags; enable `isaacsim.ros2.bridge` and
    `isaacsim.ros2.sim_control`; do not call `rclpy.init()` (shared
    context, Inv §3.1).
2.  `SimulationContext(physics_dt=1/(control_hz·substeps))`, build the
    scene, play, initialise the articulation, home
    `[0, -0.785, 0, -2.356, 0, 1.571, 0.785]`, fingers open (0.04).
    Publish epoch 0. Print `READY pid=<os.getpid()>` (flush).
3.  `rclpy` node `run_scene` with the three Trigger services (decision 1),
    `spin_once(timeout_sec=0)` once per cycle.
4.  Cycle: 4 × `sim.step(render=False)`; compute and write the wrench
    inputs (decisions 8, 9); evaluate `state`; then busy-poll `command`
    (evaluate, read `outputs:timeStamp`) until `|timeStamp − t| < 1 µs`
    or `free_run_timeout_s` of wall time passes. On a match: apply
    `ArticulationAction(joint_positions, joint_velocities)` to the 7 arm
    joints (Inv §2.3) and remember it as the held command. Free-run: hold
    the held positions with zero velocity; each cycle busy-poll at most
    `free_run_wait_s` for a command matching the latest *t*. Gripper:
    decision 7. Every `tf_every_n_cycles`: evaluate `tf`. Each cycle:
    evaluate `camera_request`; when its counter advances, remember the
    request stamp; when a request is pending and `t ≥` its stamp, do
    `camera_request_renders` × `sim.render()`, then re-read `tf` targets.
    If `render_period_wall_s` passed since the last render: one
    `sim.render()`, then re-read `tf` targets.
5.  Never call `app.update()` in the loop (it steps physics, Inv §2.1).
6.  Log every 10 s: cycles, RTF, free-run cycles, renders.
7.  Exit cleanly on `simulation_app.is_running()` false.

### 11. `container/Dockerfile` (Modify)

Add `python3-yaml` to the apt line. Add a build step: `git clone --depth 1
--branch 1.1.0 https://github.com/ros-simulation/simulation_interfaces
/opt/ws/src/simulation_interfaces && . /opt/ros/humble/setup.sh && colcon
build --base-paths /opt/ws/src --merge-install --install-base /opt/ros_ext`
(colcon, git and `rosidl_default_generators` are in the base image).
`COPY config /opt/app/config`. The entrypoint is **not** changed (its
processes do not use ROS; sourcing ROS under `set -u` aborts it).

### 12. Probes and the check (all Create)

Probe commands run as
`docker run --rm --network host -v /home/mew/untitled-project/tests/e2e:/probe:ro -e FASTRTPS_DEFAULT_PROFILES_FILE=/opt/app/container/dds/fastdds.xml -e CYCLONEDDS_URI=file:///opt/app/container/dds/cyclonedds.xml -e RMW_IMPLEMENTATION=<rmw> --entrypoint bash intrinsic-core:dev -c 'set +u; source /opt/ros/humble/setup.bash && source /opt/ros_ext/setup.bash && python3 /probe/<probe> <args>'`
(as in phase 0.5).

*   `tests/e2e/phase2_peer_hwm.py` (CycloneDDS): args `--cycles 2000
    --pause-at 1000 --pause-s 3`. Prints one JSON line at the end:
    `{"cycles", "missed", "max_track_err_rad", "rtf", "pause_sim_advance_s", "resume_cycles"}`.
*   `tests/e2e/phase2_peer_io.py` (Fast DDS): acts as the lockstep peer
    (echoes state stamps with a held target; moves joint 1 by +0.5 rad over
    1 s when a check needs it) and runs `gripper`, `wrench`, `tf`, `camera`,
    `camera_fresh`, `entities`, `reset` in that order. Prints one JSON
    line: `{"<check>": {"pass": bool, "detail": str}, ...}`. The scene
    poses for `tf` are passed in as a JSON argument by `phase2_check.py`
    (which reads `sim/scenes/default.yaml` with PyYAML).
*   `tests/e2e/phase2_check.py` (WSL): run `tools/sync_isaac.sh`; start
    `run_scene.py` (`subprocess.Popen`, log to `logs/phase2_isaac.log`);
    wait for `READY pid=` (180 s); run `standalone` (30 s, no peer); run
    the hwm probe, then the io probe; evaluate; stop Isaac with
    `taskkill.exe /PID <pid> /T /F`.

## Steps

1.  Files 1–7; run file 4, file 5, then file 6 as written in its section.
2.  Files 8–10; run `run_scene.py` alone (after `tools/sync_isaac.sh`)
    until it prints `READY` and runs 30 s without errors.
3.  File 11; rebuild the image.
4.  File 12; `.venv/bin/python tests/e2e/phase2_check.py` until all
    checks pass.
5.  `camera_request_renders`: the smallest N for which `camera_fresh`
    passes in 10/10 trials (run the io probe's `camera_fresh` alone with
    `--repeat 10`); set it in `config/timing.yaml`.
6.  Write `docs/designs/phases/phase_2_notes.md` (Create): check output,
    RTF with no camera requests and with one request per second, render
    cost, N, `panda_hand_joint` `localPos1`, the camera intrinsics, any
    deviation.
7.  If a check cannot pass without a change outside this plan, stop and
    record it.

## Addendum A (finish phase 2)

State after the lead's fixes (lights added, `d455_optical` = identity
child of `Camera_Pseudo_Depth`, wrench expectation (0, 0, 5.75) N,
periodic render every 5 s with `sim_control` callers sending
`/camera/request` to get renders): 8–9 of 10 checks pass, but three are
unstable across runs:

1.  `camera_fresh`: with `camera_request_renders` 3 the image after a
    0.5 rad joint-1 move differed by mean 1.11 (< 2); with 6 the probe
    received no frame at all. Find the cause (annotator lag, Camera Helper
    publishing on which render, the probe taking the first message per
    topic, synchronous rendering flags actually applied?) and fix it so a
    requested frame always shows the state at the request. You may change
    `sim/run_scene.py`, `sim/graphs.py`, `config/timing.yaml` and the
    probe. Then do step 5 (minimum N, 10/10).
2.  `reset` / `/sim/epoch`: the Fast DDS probe received no epoch value,
    not even the one published by the reset it requested. Find why (QoS
    match? transient-local + Fast DDS across Windows ↔ WSL? the generic
    publisher's QoS JSON?) and make a Fast DDS subscriber that subscribes
    before the reset receive it. Also add to `phase2_peer_hwm.py`
    (CycloneDDS, the real consumer, `ros2_hwm` later) a check that it
    receives the latched epoch when it starts after Isaac; report it in
    its JSON as `epoch_seen`. Data-sharing OFF in `fastdds.xml` was tried
    and did not help; do not keep it unless it is shown to be needed.
3.  `lockstep` once reported max tracking error 0.29 rad (other runs
    0.0016 rad, 0 missed). Find the cause (e.g. a command applied at the
    wrong cycle after a render, or a held command) and fix it.

Run `tests/e2e/phase2_check.py` three times in a row; all ten checks must
pass in all three runs. Record findings and fixes in `phase_2_notes.md`
under "Addendum A". Same constraints as above.
