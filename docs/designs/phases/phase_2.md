# Phase 2: the Isaac side (`sim/run_scene.py`)

Part of [robot_platform.md](../robot_platform.md). Facts with citations:
[phase_2_inventory.md](phase_2_inventory.md) ("Inv §n"); earlier:
[phase_0_5_notes.md](phase_0_5_notes.md), [phase_1_notes.md](phase_1_notes.md).

## Goal and done criteria

`sim/run_scene.py` runs Isaac headless with the default scene and talks
only ROS 2. `tests/e2e/phase2_check.py` (WSL) starts it, runs container-side
probes, and prints `PASS`/`FAIL`:

| Name | Condition |
|---|---|
| `lockstep` | a CycloneDDS probe in the container acts as a fake `ros2_hwm` for 2000 cycles: on each `/joint_states` + `/wrist_wrench` pair with stamp *t* it publishes `/joint_command` stamped *t* (a 0.3 rad, 0.5 Hz sine on `panda_joint1` around home, with velocity feed-forward). Pass: 0 missed cycles; joint 1 tracking error (commanded vs. next state) max < 0.01 rad; real-time factor printed |
| `free_run` | the probe stops for 3 s, then resumes: `run_scene.py` keeps stepping (stamps advance ≥ 2 s of sim time in that window) and returns to lockstep within 10 cycles of the probe resuming |
| `gripper` | `/gripper_command` width 0.0, effort 20 N → finger joint position < 0.002 m within 1 s sim time; width 0.08 → > 0.038 m |
| `wrench` | static `/wrist_wrench` force magnitude 5.75 N ± 0.3 (hand + fingers, Inv §2.4); after `/sim/attach_board` it rises by the board's weight ± 5 %; after `/sim/detach_board` it returns within 0.1 N |
| `tf` | `/tf` contains `red_cube`, `plate`, `peg`, `hole_block`, `table` and `d455` with poses equal to the scene file within 1 mm (static objects) |
| `camera` | after `/camera/request`, the container receives `/camera/rgb` (`rgb8` 848×480), `/camera/depth` (`32FC1` 848×480, metres, ground truth) and `/camera/camera_info` (`K` = [420.08, 0, 424; 0, 420.08, 240; 0, 0, 1] ± 0.1) stamped ≥ the request stamp |
| `camera_fresh` | moving `panda_joint1` by 0.5 rad, then requesting: the RGB image differs from the one before the move (mean absolute pixel difference > 2) |
| `entities` | a Fast DDS client in the container calls `simulation_interfaces` `GetEntities` (filter `^/World/objects/`) and sees `red_cube`; `SpawnEntity` of the catalog `red_cube` USD as `extra_cube` succeeds and it appears on `/tf`; `DeleteEntity` removes it |
| `reset` | after moving joint 1 and spawning `extra_cube`: `/sim/reset` returns success; joints are at home within 0.001 rad; `extra_cube` is gone; default objects are at their scene poses; `/sim/epoch` incremented by 1 |
| `standalone` | `run_scene.py` started with no container running keeps stepping (free-run) and logs no error for 30 s |

Plus a written record of the camera timing (renders needed for fresh
content, render cost) in `phase_2_notes.md`.

## Decisions made in this plan (and design doc changes)

1.  **D7 compliance and its exceptions.** State, wrench, joint command,
    gripper command, epoch, `/tf`, camera and `simulation_interfaces` use
    bridge components (OmniGraph nodes on on-demand graphs, Camera
    Helper, `isaacsim.ros2.sim_control`). The script's own `rclpy` is used
    only for the three `std_srvs/Trigger` servers `/sim/reset`,
    `/sim/attach_board`, `/sim/detach_board`: the bridge's service-server
    nodes send the reply in the same evaluation that receives the request,
    before the script can act (Inv §6.1).
2.  **Noisy depth moves to `camera_sim`** (design G5 and Alternative I are
    changed): Isaac's depth sensor gives 12–15 % valid pixels along image
    edges and values ≈ 10⁴ × metres (Inv §4.3), while a real D455 projects
    an IR pattern and gives dense depth; the Camera Helper also cannot
    publish the sensor's output (Inv §4.5). Isaac publishes ground-truth
    depth only (`/camera/depth`); `camera_sim` (phase 3b) applies a D455
    noise model and keeps the clean frame as ground truth. So
    `/camera/depth_gt` is dropped.
3.  **Camera renders on demand.** Rendering costs 115–210 ms (Inv §4.4)
    and would cut the real-time factor to 0.12 if done at 30 Hz (Inv
    §3.2). `run_scene.py` renders only (a) when `/camera/request`
    (`std_msgs/Header`, `stamp` = the requester's minimum sim time) arrives,
    doing `camera_request_renders` renders, and (b) every
    `render_period_wall_s` = 0.5 s of wall time, because
    `sim_control`'s handlers and the Camera Helper writers only run inside
    renders (Inv §5.2, §3.4). Synchronous rendering flags (Inv §4.4,
    exp2e) are on so a frame reflects the state at the render.
4.  **Physics:** 1 kHz (`physics_dt` 0.001), 4 substeps per 250 Hz control
    cycle, all from `config/timing.yaml`. USD writeback stays on
    (rendering correctness; turning it off is a later optimisation,
    recorded in the notes with the measured cost).
5.  **Reset without a timeline stop.** `/sim/reset` deletes every prim
    under `/World/objects`, re-creates the default objects from the scene
    file, sets the robot to home (positions and targets, velocities zero,
    gripper open), and increments `/sim/epoch` (`std_msgs/UInt32`,
    reliable, transient-local, published by a bridge generic publisher).
    Simulation time continues. `ros2_hwm` (phase 3) treats an epoch change
    as the reset signal (design "Timing: Reset" is changed from "time
    restarts at 0" to the epoch topic). `sim_control`'s
    `ResetSimulation` is not used (Inv §5.4).
6.  **Gripper command** is `sensor_msgs/JointState` on `/gripper_command`:
    `name ["panda_finger_joint1"]`, `position [width/2]`,
    `effort [max effort N]` (`control_msgs` exists on neither side, Inv
    §3.5). Only `panda_finger_joint1` has a drive (`panda_finger_joint2`
    mimics it, Inv §1.3). `run_scene.py` sets its target and its drive's
    max force = effort clamped to [0, 70] N (Franka Hand continuous grasp
    force 70 N).
7.  **Wrench** = row 8 of `get_measured_joint_forces()` (Inv §2.4),
    rotated by `panda_hand_joint`'s `localRot1` into `panda_hand` axes,
    published with `frame_id` `panda_hand`.
8.  **Object poses** come from `/tf` (bridge Publish Transform Tree, every
    8th cycle = 30 Hz sim time), not `GetEntitiesStates` (3–4 renders per
    call, Inv §5.2). Targets: every prim under `/World/objects` plus the
    camera prim; refreshed at each periodic render. Parent frame `world`.
    `sim_control` stays for spawn/delete/list (rare).
9.  **Objects live under `/World/objects/<name>`**; catalog USDs carry a
    custom attribute `catalog:name`. `SpawnEntity` requests use that path.
10. **Catalog USDs are written directly with `pxr`** from `mesh.glb`
    (Isaac's bundled `trimesh` loads it), Z-up, metres, with physics, not
    with the asset converter (Y-up, centimetres, no physics, Inv §7.2).
11. **DDS large messages:** `<maxMessageSize>1400</maxMessageSize>` in the
    Fast DDS UDP transport (both sides) and `MaxMessageSize` 1400 B in
    `cyclonedds.xml` (Inv §3.3: UDP datagrams above ~1.4 KB are lost on
    the mirrored loopback).
12. **`simulation_interfaces` in the container** is built from source at
    tag `1.1.0` (the version bundled in Isaac, Inv §5.1), not apt's 1.4.0:
    Humble matches types by name only, so a different message layout would
    corrupt data silently.

## Constraints

*   No Bazel; no `git commit`; Docker via `sg docker -c "…"`; no `pkill -f`
    or `killall`; stop Windows processes by PID (`taskkill /PID`).
*   Isaac's Python: `cmd.exe /c "cd /d C:\isaac-sim && python.bat <script>"`
    with Windows paths, or stdin as in phase 0.5. Files Isaac reads live in
    `C:\isaac-sim-work\` and are copied there by the scripts that need
    them (the repository stays the source).
*   New Python in `sim/` runs on Isaac's Python 3.11; style as before
    (`pyink --pyink-indentation=2 --line-length=80`, `isort --profile=google`).

## Target files

### 1. `config/timing.yaml` (Create)

```yaml
control_hz: 250
substeps: 4            # physics_dt = 1 / (control_hz * substeps)
free_run_timeout_s: 1.0
free_run_wait_s: 0.005
render_period_wall_s: 0.5
camera_request_renders: 3   # phase 2 measures the minimum and updates this
tf_every_n_cycles: 8
```

Read with a 10-line parser (`key: value` per line, `#` comments) shared by
`sim/` and container code: `config/timing.py` (Create) with
`load(path) -> dict[str, float]`.

### 2. `container/dds/fastdds.xml`, `container/dds/cyclonedds.xml` (Modify)

Decision 11.

### 3. `tools/build_catalog_meshes.py` (Create; WSL venv, `trimesh`)

Writes `assets/objects/<name>/mesh.glb` and `object.yaml` for:

| Name | Shape (metres) | Static | Mass kg | Colour |
|---|---|---|---|---|
| `table` | box 1.2 × 0.8 × 0.75 | yes | — | (0.55, 0.45, 0.35) |
| `red_cube` | box 0.05³ | no | 0.10 | (0.8, 0.1, 0.1) |
| `plate` | cylinder r 0.09, h 0.012 | no | 0.30 | (0.9, 0.9, 0.9) |
| `peg` | cylinder r 0.010, h 0.08 | no | 0.05 | (0.2, 0.4, 0.8) |
| `hole_block` | box 0.08 × 0.08 × 0.05 minus a cylinder r 0.011, depth 0.04, centred on top | yes | — | (0.3, 0.3, 0.3) |

Mesh origin: bottom centre (z = 0 at the base). `object.yaml`: `static`,
`mass`, `color`, `collision` (`convexHull` for non-static, `meshSimplification`
for static; the hole block needs the concave mesh, which PhysX allows
only for static bodies). Use `trimesh.creation.box/cylinder` and
`trimesh.boolean.difference` for the hole (engine `manifold`; add
`manifold3d` to the venv).

### 4. `sim/tools/build_catalog.py` (Create; Isaac Python)

For each `assets/objects/<name>/`: load `mesh.glb` with the bundled
`trimesh` (`$ISAAC/exts/omni.pip.compute/pip_prebundle` on `sys.path`, Inv
§7.2), write `model.usd`: Z-up, `metersPerUnit` 1, default prim `/Object`
(Xform) with `UsdGeom.Mesh` child, `displayColor`, `UsdPhysics.CollisionAPI`
+ `MeshCollisionAPI` approximation from `object.yaml`, and for non-static
objects `RigidBodyAPI` + `MassAPI` (mass). Custom attribute
`catalog:name` (string) on `/Object`. Commit-ready output under
`assets/objects/<name>/model.usd`.

### 5. `sim/scenes/default.yaml` (Create)

```yaml
robot: {position: [0, 0, 0]}             # Franka base at the world origin (phase 1 loader assumption)
ground_z: -0.75
objects:                                  # name: catalog entry, position (bottom centre), yaw (deg)
  table:      {catalog: table,      position: [0.45, 0.0, -0.75], yaw: 0}
  red_cube:   {catalog: red_cube,   position: [0.50, 0.15, 0.0],  yaw: 0}
  plate:      {catalog: plate,      position: [0.50, -0.20, 0.0], yaw: 0}
  peg:        {catalog: peg,        position: [0.35, 0.30, 0.0],  yaw: 0}
  hole_block: {catalog: hole_block, position: [0.60, 0.30, 0.0],  yaw: 0}
camera: {position: [1.3, 0.0, 0.9], euler_deg: [0, 45, 180]}   # Inv §4.1
```

Parsed with a small hand-written parser in `sim/scene.py` (Isaac's Python
has PyYAML? check `import yaml`; if present use it, else use the
`config/timing.py`-style parser extended to one level of `{…}` maps — the
worker checks once and records which).

### 6. `sim/scene.py` (Create)

*   `build(stage, scene, asset_root, repo_assets)`: ground plane at
    `ground_z`; Franka from `<asset root>/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd`
    at `/World/franka` (Inv §1.1–1.2); objects at `/World/objects/<name>`
    referencing `C:\isaac-sim-work\assets\objects\<catalog>\model.usd`;
    D455 from `rsd455.usd` at `/World/d455` with
    `physics:kinematicEnabled = True` on `/World/d455/RSD455` (Inv §4.1),
    camera `Camera_Pseudo_Depth`, resolution 848×480, clipping range
    (0.05, 20) (Inv §7.3).
*   `reset_objects(stage, scene)`: delete `/World/objects/*`, recreate the
    defaults.
*   `attach_board(stage)` / `detach_board(stage)`: board =
    `/World/objects/calib_board`: a 0.21 × 0.15 × 0.005 m box, mass 0.2 kg,
    whose top face carries the ChArUco texture (7 × 5 squares, 30 mm
    squares, 22 mm markers, `DICT_5X5_100`, margin 0, generated once with
    the bundled `cv2` into `assets/calib/charuco_7x5.png`, Inv §7.3);
    `UsdPhysics.FixedJoint` body0 `/World/franka/panda_hand`, body1 the
    board, `localPos0` (0, 0, 0.12) (beyond the fingertips), board normal
    along the hand's +z (Inv §6.2).

### 7. `sim/graphs.py` (Create)

On-demand OmniGraph graphs (Inv §3.4), QoS JSON written straight into
`inputs:qosProfile` (reliable, keep-last 1, volatile unless stated):
*   `state`: Read Simulation Time → Publish Joint State (`/joint_states`,
    target `/World/franka`) and generic Publisher `geometry_msgs/WrenchStamped`
    (`/wrist_wrench`; the script writes the flat inputs each cycle).
*   `command`: Subscribe Joint State `/joint_command` and Subscribe Joint
    State `/gripper_command`; the script reads `outputs:timeStamp`,
    `positionCommand`, `velocityCommand`, `effortCommand`.
*   `camera_request`: generic Subscriber `std_msgs/Header` on
    `/camera/request` with an `omni.graph.action.Counter` on its
    `execOut` (the script detects new requests by the counter).
*   `epoch`: generic Publisher `std_msgs/UInt32` `/sim/epoch`
    (transient-local).
*   `tf`: Publish Transform Tree, parent `/World`, targets set by the
    script.
*   `camera`: Camera Helper `rgb` (`/camera/rgb`), Camera Helper `depth`
    (`/camera/depth`), CameraInfo Helper (`/camera/camera_info`),
    `frame_id` `d455_optical`, on the D455 render product; evaluated once
    to create the writers (Inv §3.4, §4.5).

### 8. `sim/run_scene.py` (Create)

1.  Environment as `sim/ros_probe.py` (ROS distro, RMW, PATH,
    `FASTRTPS_DEFAULT_PROFILES_FILE=C:\isaac-sim-work\fastdds.xml`,
    domain 0); `SimulationApp` headless with
    `--/app/vulkan=false` and the synchronous rendering flags of Inv §4.4
    (exp2e); enable `isaacsim.ros2.bridge` and `isaacsim.ros2.sim_control`
    (Inv §3.1); do not call `rclpy.init()` (shared context, Inv §3.1).
2.  `SimulationContext(physics_dt=1/(control_hz*substeps))`, build the
    scene, play, initialise the articulation, set home
    `[0, -0.785, 0, -2.356, 0, 1.571, 0.785]`, fingers open.
3.  `rclpy` node `run_scene` with the three Trigger services (decision 1),
    spun with `spin_once(timeout_sec=0)` once per cycle.
4.  Cycle (lockstep, design "Timing"): 4 × `step(render=False)`; write
    wrench inputs; evaluate `state`; then wait for a command with
    `timeStamp == t` by evaluating `command` repeatedly (every 0.1 ms)
    until it matches or `free_run_timeout_s` (wall) passes; apply
    `ArticulationAction(joint_positions, joint_velocities)` for the 7 arm
    joints (Inv §2.3). Gripper: if `/gripper_command` changed, apply
    decision 6. In free-run: hold the last positions with zero velocity,
    wait at most `free_run_wait_s` per cycle for a command stamped with the
    latest *t*. Every `tf_every_n_cycles`: evaluate `tf`. Each cycle:
    evaluate `camera_request`; if the counter advanced, do
    `camera_request_renders` × `sim.render()`. If `render_period_wall_s`
    elapsed since the last render: one `render()` and refresh `tf`
    targets.
5.  Stamps: physics time (`IsaacReadSimulationTime`, Inv §3.4). Never
    `app.update()` in the loop (it would step physics, Inv §2.1).
6.  Log every 10 s: cycles, real-time factor, free-run count.
7.  Exit on SIGINT/`simulation_app.is_running()` false.

### 9. `container/Dockerfile` (Modify)

Build `simulation_interfaces` 1.1.0 from source (decision 12): clone
`https://github.com/ros-simulation/simulation_interfaces` at tag `1.1.0`
into `/opt/ws/src`, `colcon build --merge-install --install-base
/opt/ros_ext` (install `python3-colcon-common-extensions` and
`ros-humble-ament-cmake`, `ros-humble-rosidl-default-generators` if not
present), and `source /opt/ros_ext/setup.bash` in the entrypoint and in
probe commands.

### 10. Probes and the check

*   `tests/e2e/phase2_peer_hwm.py` (container, CycloneDDS): fake
    `ros2_hwm` for `lockstep`/`free_run` (command = sine; pauses on a
    command-line flag schedule); prints per-cycle stats as JSON at the end.
*   `tests/e2e/phase2_peer_io.py` (container, Fast DDS): gripper, wrench,
    tf, camera, entities, reset checks as subcommands; prints JSON results.
*   `tests/e2e/phase2_check.py` (WSL): copies `fastdds.xml`, `assets/`,
    `sim/` to `C:\isaac-sim-work\`; starts `run_scene.py`
    (`subprocess.Popen`, logs to `logs/phase2_isaac.log`), waits for its
    `READY` log line (printed after the first cycle); runs the peers with
    `docker run --rm --network host` like phase 0.5 (with
    `-v /home/mew/untitled-project/tests/e2e:/probe:ro`); evaluates the
    table; stops Isaac by PID.

## Steps

1.  Files 1–5 (+ run `tools/build_catalog_meshes.py`, then
    `sim/tools/build_catalog.py` in Isaac).
2.  Files 6–8; run `run_scene.py` alone until `standalone` holds.
3.  File 9; rebuild the image.
4.  File 10; `.venv/bin/python tests/e2e/phase2_check.py` until all
    checks pass.
5.  Measure `camera_request_renders`: the smallest N for which
    `camera_fresh` passes in 10/10 trials; set it in `config/timing.yaml`.
6.  Write `docs/designs/phases/phase_2_notes.md` (Create): check output,
    real-time factor with and without a camera request per second,
    render cost, N, PyYAML presence, deviations.
7.  If a check cannot pass without a change outside this plan, stop and
    record it.
