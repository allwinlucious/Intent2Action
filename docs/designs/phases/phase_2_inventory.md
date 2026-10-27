# Phase 2 inventory: `sim/run_scene.py` on Isaac Sim 5.1

This is a factual inventory for [robot_platform.md](../robot_platform.md)
("Isaac Sim side", "Timing", "Isaac start-up", "Object catalog", Q7 and
D7). It covers what `sim/run_scene.py` needs:

- the Franka asset
- physics stepping and articulation I/O
- ROS 2 from a script that steps physics itself
- the D455 depth camera on the Direct3D 12 renderer
- `isaacsim.ros2.sim_control`
- the `/sim/*` Trigger services
- catalog assets

**Citation conventions.**

| Shorthand | Path |
|---|---|
| `$ISAAC/` | `/mnt/c/isaac-sim/` (`C:\isaac-sim`) |
| `$BR/` | `$ISAAC/exts/isaacsim.ros2.bridge/` |
| `$SC/` | `$ISAAC/exts/isaacsim.ros2.sim_control/isaacsim/ros2/sim_control/impl/` |
| `$PRIMS/` | `$ISAAC/exts/isaacsim.core.prims/isaacsim/core/prims/impl/` |
| `$CAMX/` | `$ISAAC/exts/isaacsim.sensors.camera/isaacsim/sensors/camera/` |
| `$SIMCTX` | `$ISAAC/exts/isaacsim.core.api/isaacsim/core/api/simulation_context/simulation_context.py` |
| `$AC/` | `$ISAAC/extscache/omni.kit.asset_converter-5.0.17+107.3.1.wx64.r.cp311.u353/` |
| `W/` | `C:\isaac-sim-work\phase2\` (`/mnt/c/isaac-sim-work/phase2/`) |
| `project/` | `/home/mew/untitled-project/` |
| `ic/` | `/home/mew/intrinsic-core/` |

"Observed (expN)" means the output of the experiment script `W/scripts/expN*.py`. Its log is `W/scripts/expN*.log` and its JSON result `W/expN.json` where one exists; §0.1 lists the experiments. Every Isaac run:

- used `$ISAAC/python.bat` headless with `--/app/vulkan=false` (D3D12)
- used physics on the CPU at 1 kHz
- set the ROS environment of `project/sim/ros_probe.py:15-19`

The ROS peers were throwaway `docker run --rm` containers of
`intrinsic-core:dev` (Humble, `rmw_fastrtps_cpp`, `--network host`). This is the only
repository file written. Bazel was not run and nothing was committed.

**Caveat on all wall-clock numbers involving rendering.** During the
camera experiments `nvidia-smi` showed 95 % GPU use and 6.5 GB used by
another application (a game), with Isaac not running. Render times below
are therefore upper bounds, not budgets. Physics-only numbers are CPU-bound
and less affected.

---

## 0. Key findings

1.  **Camera images do not cross the Windows ↔ WSL boundary with the
    current DDS profile** (observed, exp4). Any message larger than about
    1.3 KB is lost in both directions of QoS, while ≤ 1300 B payloads
    arrive. That is the IP-fragmentation limit of the WSL mirrored loopback
    (1500 B MTU). With `<maxMessageSize>1400</maxMessageSize>` added to the
    UDP transport on **both** sides, every size up to 1.6 MB arrives. The
    profile used was a copy of `project/container/dds/fastdds.xml` plus that
    one line. Publishing then costs about 11 ms per 1.2 MB message
    (§3.3). The repository profile has no such line
    (`project/container/dds/fastdds.xml:7-12`).
2.  **Q7 is answered yes.** `SingleViewDepthSensorAsset` with `rsd455.usd`
    renders RGB, noisy depth and ground-truth depth headless on D3D12 at
    848×480 (observed, exp2). There are two surprises:
    - The noisy `DepthSensorDistance` output is about 10 000 × the ground
      truth in a 1 m-per-unit stage. The documentation says meters.
    - Only 12-15 % of its pixels are valid at the default settings, and
      the valid pixels follow image texture (§4.3).
3.  **The bridge's Camera Helper `depth` type publishes ground truth only**
    (`DistanceToImagePlane`, `$BR/isaacsim/ros2/bridge/ogn/python/nodes/OgnROS2CameraHelper.py:127-139`).
    No Camera Helper type publishes the noisy depth sensor output (§4.5).
4.  **Lockstep works with bridge OmniGraph nodes alone.** The setup is
    on-demand graphs and `og.Controller.evaluate_sync`, run after 4 ×
    `step(render=False)`:
    - Publish Joint State stamped by Read Simulation Time
    - a generic `WrenchStamped` publisher
    - Subscribe Joint State

    Over 1000 cycles to the container: RTF 0.61, 0 misses, stamps equal to
    `sim.current_time` (observed, exp6). The script's own `rclpy`
    reached RTF 0.54 (exp3b). Physics dominates: 1.26 ms per 1 kHz step
    (§2.2).
5.  **`isaacsim.ros2.sim_control` handlers run only inside
    `app.update()`/`render()`.** No call completed during 3 s of
    physics-only stepping. Each completed after 3-4 `render()` calls
    (observed, exp3b; mechanism at `$SC/simulation_control.py:334-351`).
    `ResetSimulation` stops and plays the timeline (`:976-1005`). That
    invalidates the script's articulation handle and resets the robot to
    its USD pose (§5.4).
6.  **The Franka asset differs from the design in two ways:**
    - `panda_finger_joint2` has no drive. It is a PhysX mimic joint of
      `panda_finger_joint1`, so only one finger joint can be commanded.
    - The finger drive's max force is 7.2 N, not the URDF's 20 N (§1.3).

    `panda_hand_joint` is a separate fixed joint that stays in the
    articulation, as the design needs. Its measured force is reported in
    the joint's child frame, which is not `panda_hand` axes (§2.4).
7.  **FK matches Lula** (`lula_franka_gen.urdf`) within 0.0003 mm and
    0.016° for links 1, 4, 7 and `panda_hand` in 3 configurations.
    `panda_link8` is not a body in the USD (§1.4).
8.  **The bridge's generic service server replies before the script can
    act.** It answers in the same `evaluate_sync` that receives the
    request, with whatever the response inputs held at that moment
    (observed, exp7). The script's own `rclpy` `Trigger` server, called
    from the container, works with 3-12 ms round trips (§6.1).

### 0.1 Experiments

| Script (`W/scripts/`) | What it does | Result file |
|---|---|---|
| `exp1_franka.py` | Franka load, joints, drives, FK vs Lula, measured forces, runtime attach, tracking, timing | `W/exp1.json` |
| `exp1b_timing.py`, `exp1c_timing.py` | Step timing with USD writeback on (1b) and off (1c) | logs |
| `exp2a_rsd455.py` | `rsd455.usd` structure | log |
| `exp2_camera.py`, `exp2b`-`exp2e` | Depth sensor frames, noise, latency (2e with synchronous rendering flags) | `W/exp2*.json`, `W/*.png`, `W/*.npy` |
| `exp3_ros.py` (default DDS profile), `exp3b_ros.py` (fragmenting profile) + `peer.py` | rclpy lockstep, rclpy camera publish, Camera Helper, sim_control calls, timeline stop/play | `W/exp3.json` (from the exp3b run) |
| `exp4_size.py`, `exp4a.py`, `exp4b.py` + `peer_size.py` | DDS message-size sweep Windows → container | logs |
| `exp5_assets.py`, `exp5b`-`exp5e` | Props listing, GLB → USD, ChArUco board render + detection | `W/exp5*.json`, `W/charuco_*.png` |
| `exp6_graph_lockstep.py` + `peer.py` | Lockstep using OmniGraph bridge nodes only | `W/exp6.json` |
| `exp7_trigger.py` + `peer_trigger.py` | `std_srvs/Trigger` served by bridge nodes and by rclpy, called from the container | `W/exp7.json` |

`W/fastdds_mms1400.xml` is the test DDS profile (finding 1).

---

## 1. Franka asset (Q1)

### 1.1 Path and asset resolution

- **USD:** `/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd` under the
  asset root. This is the path Isaac's own Lula FK test uses
  (`$ISAAC/exts/isaacsim.robot_motion.motion_generation/isaacsim/robot_motion/motion_generation/tests/test_kinematics.py:128-144`).
- **The asset root is an HTTPS S3 URL, not a local path.**
  - `get_assets_root_path()` reads
    `/persistent/isaac/asset_root/default`
    (`$ISAAC/exts/isaacsim.storage.native/isaacsim/storage/native/nucleus.py:33,461-496`).
  - The default is
    `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1`,
    with a 5 s timeout
    (`$ISAAC/exts/isaacsim.storage.native/config/extension.toml:27,29`).
  - The user config
    `C:\Users\Allwin\AppData\Local\ov\data\Kit\Isaac-Sim Full\5.1\user.config.json`
    holds the same URL.
  - Observed (exp1): `omni.client.stat` of the Franka URL returned
    `Result.OK`, and the load took 0.64 s. The stage's used layers are
    all under that URL (`Props/panda_link*.usd`, `Materials/Materials.usd`,
    `configuration/…`).
- **Offline behaviour.** Downloaded files are cached in
  `C:\Users\Allwin\AppData\Local\ov\cache\client\https\` (about 480 MB of
  `.usd` files, observed by listing). No local asset pack was found on
  disk. Whether a run without network succeeds from that cache was not
  tested (§9).
- **Variant sets** (observed, exp1):
  - `Mesh`: `Performance` (selected) or `Quality`
  - `Gripper`: `Default` (selected), `AlternateFinger`, `None` or `Robotiq_2F_85`

### 1.2 Prim paths (reference at `/World/franka`, observed exp1)

- **Articulation root:** `/World/franka`, with joint `/World/franka/rootJoint`.
- **Bodies (11):** `panda_link0` … `panda_link7`, `panda_hand`,
  `panda_leftfinger`, `panda_rightfinger`, all directly under
  `/World/franka`.
  - Masses: `panda_hand` 0.558 kg, each finger 0.0141 kg, `panda_link7`
    0.405 kg.
  - **There is no `panda_link8` prim.** Its 0.107 m offset lives in
    `panda_hand_joint`'s `localPos0`.
- **Arm joints:** `/World/franka/panda_link{N-1}/panda_joint{N}` (revolute,
  axis X).
- **`/World/franka/panda_link7/panda_hand_joint`** is a `PhysicsFixedJoint`:
  - body0 `panda_link7`, body1 `panda_hand`
  - `localPos0` (0, 0, 0.107), `localRot0` (0.7071, 0, 0.7071, 0)
  - `localRot1` (0.6533, −0.2706, 0.6533, 0.2706)
  - `excludeFromArticulation` false

  **It is a separate joint, not merged.** It appears in the articulation
  metadata as joint index 7, between `panda_joint7` (6) and the finger
  joints (8, 9).
- **Fingers:** `/World/franka/panda_hand/panda_finger_joint1` (→
  `panda_leftfinger`) and `…/panda_finger_joint2` (→ `panda_rightfinger`).
  Both are prismatic, axis X, limits 0-0.04 m, `localPos0` z = 0.0584.
- **DOF names (9):** `panda_joint1` … `panda_joint7`,
  `panda_finger_joint1`, `panda_finger_joint2`.

### 1.3 Drive defaults (observed exp1; USD values, then `get_gains`/`get_max_efforts`)

| Joint | Drive | Stiffness / damping | Max force | Max velocity |
|---|---|---|---|---|
| `panda_joint1`-`4` | angular, `force` | USD 400 / 80 per degree; API 22918.3 / 4583.7 per rad | 87 N·m | 2.175 rad/s |
| `panda_joint5`-`7` | same | same | 12 N·m | 2.61 rad/s |
| `panda_finger_joint1` | linear, `force` | 400 / 80 | **7.2 N** | 0.2 m/s |
| `panda_finger_joint2` | **none**: applied schemas `PhysxMimicJointAPI:rotX`, `PhysxJointAPI`, `IsaacJointAPI` | API 0 / 0 | API 0 | 10 |

Joint limits from `get_dof_limits` (rad) are the standard Panda limits,
for example joint 4 is −3.0718 … −0.0698 and joint 6 is −0.0175 … 3.7525.

Physics scene `/physicsScene` (observed exp1, exp1b):
- solver TGS, broadphase MBP, `enableGPUDynamics` false
- `PhysxArticulationAPI` position iterations 32, velocity iterations 1,
  self-collision off

The URDF has finger effort 20 and velocity 0.2, both fingers prismatic
with limits 0-0.04, and arm efforts 87/12
(`project/assets/robots/franka/lula_franka_gen.urdf:126-246,298-310`). The
repository copy is identical to Isaac's (`diff`, observed).

### 1.4 Kinematics vs `lula_franka_gen.urdf`

- **Fixed joints in the URDF:** `panda_joint8`, xyz 0 0 0.107
  (`lula_franka_gen.urdf:249-250`), and `panda_hand_joint`, rpy 0 0
  −0.785 (`:255-258`).
- **In the USD** these two are composed into the one `panda_hand_joint`
  (§1.2).
- **FK check** (observed exp1, Lula loaded as in
  `project/tests/e2e/isaac_lula_fk.py:13-31`): each configuration was
  driven and settled, then compared. The table gives position and angle
  differences between PhysX link transforms and Lula FK. The USD
  `XFormPrim` after `render()` gave the same positions and at most 0.0164°.

| q (7 arm joints, rad) | link1 | link4 | link7 | panda_hand |
|---|---|---|---|---|
| 0, −1.3, 0, −2.87, 0, 2.0, 0.75 | 0.0000 mm, 0° | 0.00004 mm, 0° | 0.00026 mm, 0° | 0.00032 mm, 0° |
| 0.5, −0.4, 0.3, −2.0, 0.2, 1.8, −0.6 | 0.00018 mm | 0.00029 mm | 0.00019 mm | 0.00021 mm, 0° |
| −1.0, 0.6, −0.8, −1.2, 1.5, 2.5, 1.2 | 0.00006 mm, 0.0048° | 0.00013 mm | 0.00027 mm | 0.00027 mm, 0° |

`panda_link8` cannot be compared directly because it is not a body. Since
`panda_hand` = link8 rotated −45° about z with no translation
(`lula_franka_gen.urdf:255-258`), the hand match implies link8's position
matches.

Isaac's own test compares Franka FK with a 5 mm tolerance and skips the
finger frames and frame 0, with the comment "There is a known bug with the
kinematics not matching on the Franka finger frames"
(`test_kinematics.py:128-144`). Finger frames were not checked here.

---

## 2. Stepping and articulation I/O (Q2)

### 2.1 API

- **Constructor and dt:**
  - `SimulationContext(physics_dt, rendering_dt, stage_units_in_meters, physics_prim_path, sim_params, set_defaults, backend, device, stage)` (`$SIMCTX:101-112`)
  - `set_simulation_dt` (`:407-470`)
  - In `step(render=True)`, Isaac runs rendering_dt / physics_dt substeps
    per call (`:407-470`). `run_scene.py` does not need this mode.
- **Physics only:** `step(render=False)` calls `simulate` + `fetch_results`
  (`$SIMCTX:672-714`; `physics_context.py:563-565`). It needs the timeline
  playing.
- **Rendering without physics:** `render()` runs `app.update()` with
  `/app/player/playSimulations` set to False (`$SIMCTX:716-747`).
- **Two clocks:**
  - `sim.current_time` sums physics step events and resets when the
    timeline stops (`$SIMCTX:1419-1426`).
  - Observed (exp1): 1000 × `step(render=False)` advanced
    `current_time` by 1.000 s while the timeline stayed at 0. 30 ×
    `render()` advanced the timeline by 1.000 s and physics time by 0.
  - Observed (exp3b): after `sim.stop(); app.update(); sim.play();
    app.update()`, `current_time` was 0.035. A plain `app.update()` while
    playing steps physics.

  **So `run_scene.py` must render with `render()`, not `app.update()`,
  and must use physics time (`current_time`) for stamps, not timeline
  time.**
- **Commands:**
  - `apply_action(ArticulationAction(joint_positions, joint_velocities,
    joint_indices))` (`$PRIMS/single_articulation.py:948`), used in
    exp1/exp3/exp6.
  - Gains: `set_gains`/`get_gains` (`$PRIMS/articulation.py:2974,3114`).
  - Limits: `get_max_efforts` (`:2790`), `get_dof_limits` (`:440`),
    `get_joint_max_velocities` (`:2924`).
- **Reads:**
  - `get_joint_positions`, `get_joint_velocities`
  - `get_measured_joint_efforts`: DOF-projected efforts
    (`$PRIMS/articulation.py:1285-1354`). At home q (observed exp1):
    joint2 9.26 N·m, joint4 20.26 N·m, joint6 1.04 N·m, the others about 0.
  - `get_measured_joint_forces` (`:1356-1467`, §2.4)

  All reads together took 0.027 ms (exp1b).

### 2.2 Cost (observed; CPU physics, Franka + ground)

| Measurement | Value | Source |
|---|---|---|
| `step(render=False)`, 1 kHz | 1.26 ms (raw `simulate`+`fetch` 0.98 ms) | exp1b |
| same, position iterations 8 instead of 32 | 1.21 ms | exp1b |
| same with `/physics/updateToUsd` and `/physics/updateVelocitiesToUsd` False | 0.38 ms (raw 0.33) | exp1c |
| `render()`, no camera | 21 ms | exp1 |

With USD writeback off, USD and `XFormPrim` poses stop following physics.
The measured discrepancy for `panda_hand` was 440 mm (exp2,
`noUsd_hand_phys_vs_xform_mm`). It returned to 0.002 mm after writeback was
turned back on and a step was run (`usd_hand_phys_vs_xform_mm`). Whether
the RTX renderer reads robot poses from USD or from Fabric with writeback
off was not determined (§9).

### 2.3 Position + velocity targets (observed exp1)

Joint 1 tracked a 0.3 rad, 0.5 Hz sine commanded at 250 Hz with 4
substeps:
- position only: max error 0.158 rad, RMS 0.101 rad
- position + velocity feed-forward: max error 0.0015 rad, RMS 0.0010 rad

The large default damping (4584 per rad/s) drags toward zero velocity
unless a velocity target is given.

### 2.4 Measured force at `panda_hand_joint`

- **API.** `get_measured_joint_forces()` returns (num_joints + 1) × 6 rows
  (force N, torque N·m), and a joint's row is `joint_index + 1`. Per the
  docstring, values are "reported in the local body reference frame (child
  joint frame of the link's incoming joint)"
  (`$PRIMS/articulation.py:1356-1467`). For `panda_hand` (body index 8 of
  11) that is row 8 (observed exp1).
- **Static value at home q** (observed exp1): F = (5.231, −1.633, 1.753) N,
  T = (0.000, 0.0049, 0.0046) N·m.
  - |F| = 5.75 N, which is the weight of hand + fingers (0.5864 kg × g).
  - The same weight in `panda_hand` axes is (−2.394, −0.085, 5.231) N.
  - The reported vector is therefore the supporting force in the joint's
    child frame, i.e. `panda_hand` axes rotated by `localRot1` (§1.2).
  - **To publish in frame `panda_hand`, rotate by `localRot1`.**
- **With a 0.5 kg board fixed to the hand:** ΔF = (4.42, −1.35, 1.48) N
  against an expected 4.905 N magnitude, i.e. 1.1 % low (observed exp1,
  `attach.dF`).

---

## 3. ROS 2 from `run_scene.py` (Q3)

### 3.1 Contexts

- Enabling `isaacsim.ros2.bridge` does **not** enable
  `isaacsim.ros2.sim_control`; it must be enabled explicitly (observed
  exp3).
- `sim_control` calls `rclpy.init()` and ignores the RuntimeError if
  already initialised (`$SC/simulation_control.py:130-135`). After it is
  enabled, the script's own `rclpy.init()` raises
  `RuntimeError('Context.init() must only be called once')`, while
  `rclpy.ok()` is True (observed exp3). The script then shares
  `sim_control`'s default context. Its node `isaac_sim_control` is spun by
  a `MultiThreadedExecutor` thread (`:85-148`).

### 3.2 Lockstep (250 Hz cycle, 4 substeps, peer echoes the state stamp)

Both runs used `peer.py` in the container and reliable, keep-last-1,
volatile QoS:

| Variant | RTF | Wall per cycle | Wait for command p50 / p99 / max | Misses | Source |
|---|---|---|---|---|---|
| Script `rclpy` (JointState + WrenchStamped out, JointState in) | 0.54 | 7.4 ms | 0.58 / 1.28 / 1.91 ms | 0 / 1000 | exp3, exp3b |
| Bridge OmniGraph only (§3.4) | 0.61 | 6.6 ms | 0.55 / 1.32 / 1.88 ms | 0 / 1000 | exp6 |
| Script `rclpy` + `render()` and 3-image publish every 8th cycle | 0.12 | 33 ms | 0.57 / 1.77 / 3.0 ms | 0 / 500 | exp3b |

The peer received all 1500 (exp3b) and 1000 (exp6) state and wrench
messages.

### 3.3 Large messages across Windows ↔ WSL (observed exp4, exp3b)

`std_msgs/UInt8MultiArray` was sent from Isaac's `rclpy` to the container,
20 messages per size, reliable and best effort:

| Profile | 1000-1300 B | 1400 B … 1.6 MB |
|---|---|---|
| `project/container/dds/fastdds.xml` (as is) | 20/20 | **0/20**, both QoS |
| same + `<maxMessageSize>1400</maxMessageSize>` in the UDP transport, both sides | 20/20 | reliable 20/20 (19/20 at 1.6 MB); best effort 17-20/20 |

- **Publish cost with the 1400 B profile** (median, blocking
  `publish()`): 16 KB 0.37 ms, 600 KB 6.5 ms, 1.2 MB 10.8 ms, 1.6 MB
  16.1 ms (reliable). Without it, the call costs under 1 ms but nothing
  arrives.
- **Real images** (exp3b, same profile; 63 frames each):
  - `rclpy` `Image` messages `rgb8` 848×480 (1 221 120 B) and `32FC1`
    848×480 (1 628 160 B) all arrived.
  - In the camera cycle, the annotator reads took 11.5 ms, message
    building 3.3 ms and publishing the three images 44 ms (mean).
- **Default profile** (exp3): the same topics delivered 0 images, while
  CameraInfo (small) and in-process subscribers on Windows did receive
  them.
- **Where the limit is.** Phase 0.5 measured RTT with small messages only
  (`docs/designs/phases/phase_0_5_notes.md`). The large-message risk was
  flagged but not tested (`phase_0_5_inventory.md:645-648`). WSL runs in
  `networkingMode=mirrored` (`C:\Users\Allwin\.wslconfig`). The cutoff
  between 1300 B and 1400 B payloads plus RTPS and UDP/IP headers matches
  a 1500 B MTU with IP fragments dropped. That is an inference, not
  traced with a packet capture.

### 3.4 OmniGraph nodes under `evaluate_sync` (observed exp6, exp3b)

- Graphs created with `pipeline_stage` `GRAPH_PIPELINE_STAGE_ONDEMAND`,
  with an `omni.graph.action.OnTick` entry node, evaluate on each
  `og.Controller.evaluate_sync(graph)` with the timeline playing. This
  held for Publish Joint State, generic Publisher, Subscribe Joint State
  and Service Server (exp6, exp7).
- Camera Helper and CameraInfo Helper need one `evaluate_sync`. It creates
  the writers, which then publish on every `render()` (exp3b; pattern
  from
  `$ISAAC/standalone_examples/api/isaacsim.ros2.bridge/camera_manual.py:70-180`).
- **Clock.** `isaacsim.core.nodes.IsaacReadSimulationTime.outputs:simulationTime`
  equalled `sim.current_time` exactly after physics-only steps, over 1000
  cycles (exp6, `stamp_minus_sim_time_max_abs` 0.0). Camera Helper stamps
  are physics time too, not timeline time (exp3b, §4.5).
- **Graph cost.** One evaluation of a graph with Read Simulation Time,
  Publish Joint State and the `WrenchStamped` publisher took 0.42 ms mean
  (exp6).
- **QoS.** Writing a JSON string such as
  `{"history":"keepLast","depth":1,"reliability":"reliable","durability":"volatile",…}`
  straight into `inputs:qosProfile`, without a QoS node, worked. The peer
  matched and received every message (exp6).
- **Generic publisher fields.** With `geometry_msgs`/`WrenchStamped` set,
  the generic Publisher's inputs are flat:
  - `inputs:header:frame_id`, `inputs:header:stamp:sec`,
    `inputs:header:stamp:nanosec`
  - `inputs:wrench:force:{x,y,z}`, `inputs:wrench:torque:{x,y,z}`

  So the script sets the stamp and wrench fields each cycle (exp6).
- **Subscriber outputs.** Subscribe Joint State
  `outputs:timeStamp` (seconds) matched the stamp the peer echoed. Its
  `outputs:positionCommand` and `outputs:velocityCommand` held the
  commanded values: the robot stayed at the commanded home pose (exp6,
  `q_end`).
- **Not checked:** a same-process `rclpy` subscriber to the graph's
  `/p2/joint_states` received nothing within 3 s, so the published values
  were not compared with physics in-process (exp6, `graph_js_check` null).

### 3.5 Message packages for the gripper command

`control_msgs` is absent from:
- Isaac's bundled Humble packages (listing of
  `$BR/humble/rclpy/`; also `phase_0_5_inventory.md:174-180`)
- the container image (`ls /opt/ros/humble/share`, observed)

Packages present in both Isaac's bundle and the container image
(observed listings):

| Package | Candidate for `/gripper_command` |
|---|---|
| `sensor_msgs` | `JointState`: `name` [`panda_finger_joint1`], `position` [width/2], `effort` [max effort]. The bridge's Subscribe Joint State outputs `positionCommand` and `effortCommand` (`phase_0_5_inventory.md:240-241`) |
| `std_msgs` | `Float64MultiArray` [width, max effort] |
| `trajectory_msgs` | `JointTrajectory` (heavier than needed) |
| `geometry_msgs`, `std_srvs`, `builtin_interfaces`, `rosgraph_msgs` | — |

Isaac's bundle also has `simulation_interfaces`, `isaac_ros2_messages`,
`example_interfaces`, `nav2_msgs` and `vision_msgs` (listing of
`$BR/humble/rclpy/`). `$BR/humble/lib` has typesupport DLLs for
`sensor_msgs`, `std_msgs`, `std_srvs`, `trajectory_msgs`,
`geometry_msgs`, `simulation_interfaces` and `isaac_ros2_messages`.

On the Bazel side, intrinsic-core's `MODULE.bazel` brings in
`ros2_common_interfaces` (`ic/MODULE.bazel:962`). That upstream repository
contains `sensor_msgs`, `std_msgs` and `trajectory_msgs` (external
knowledge; its BUILD targets were not inspected).

---

## 4. Camera: `SingleViewDepthSensorAsset` + `rsd455.usd` (Q4, Q7)

### 4.1 Asset structure (observed exp2a)

- **URL:** `<asset root>/Isaac/Sensors/Intel/RealSense/rsd455.usd`.
- **Rigid body.** Referenced at `/World/D455`, the prim
  `/World/D455/RSD455` is an Xform with `PhysicsRigidBodyAPI`. It falls
  under gravity unless `physics:kinematicEnabled` is set True. With that
  set, it stayed at (1.3, 0, 0.9) after 1500 steps (exp2,
  `camera_pos_after_1500_steps`).
- **Cameras under it:**
  - `Camera_Pseudo_Depth`: focal length 1.93, horizontal aperture 3.896,
    vertical aperture 2.453
  - `Camera_OmniVision_OV9782_Color`, `…_Left`, `…_Right`
  - an `Imu_Sensor`
- **Template render product.**
  `TemplateRenderProducts/Camera_Pseudo_Depth_render_product` is
  **2048×1080**. Its `omni:rtx:post:depthSensor:*` attributes:

  | Attribute | Value |
  |---|---|
  | `baselineMM` | 55 |
  | `confidenceThreshold` | 0.95 |
  | `enabled` | False |
  | `focalLengthPixel` | 897 |
  | `maxDisparityPixel` | 110 |
  | `maxDistance` | 1e7 |
  | `minDistance` | 0.5 |
  | `noiseDownscaleFactorPixel` | 1 |
  | `noiseMean` | 0.25 |
  | `noiseSigma` | 0.25 |
  | `outlierRemovalEnabled` | 3 |
  | `rgbDepthOutputMode` | 0 |
  | `sensorSizePixel` | 1280 |
- **Frame.** The D455 Xform looks along its +x with z up. Euler
  (0°, 45°, 180°) at (1.3, 0, 0.9) looks down at the robot's work area.

### 4.2 API that worked (exp2, exp3)

```python
asset = SingleViewDepthSensorAsset(prim_path="/World/D455", asset_path=root + "/Isaac/Sensors/Intel/RealSense/rsd455.usd",
                                   position=..., orientation=...)          # $CAMX/single_view_depth_sensor.py:240-330
# set /World/D455/RSD455 physics:kinematicEnabled = True
sim.reset()
asset.initialize(attach_rgb_annotator=True)                                # :355-397 (enables the depth sensor, copies the template attributes)
sensor = asset.get_child_depth_sensor("/World/D455/RSD455/Camera_Pseudo_Depth")   # :407
sensor.set_resolution((848, 480))                                          # camera.py:517; render product became 848x480
sensor.attach_annotator("DepthSensorDistance")                             # camera.py:712
sensor.add_distance_to_image_plane_to_frame()                              # camera.py:828
rgb   = sensor.get_rgba()                                                  # (480, 848, 4) uint8, camera.py:1015
noisy = sensor._custom_annotators["DepthSensorDistance"].get_data()        # (480, 848) float32
gt    = sensor.get_depth()                                                 # (480, 848) float32 m, distance_to_image_plane, camera.py:1050
K     = sensor.get_intrinsics_matrix()                                     # camera.py:1703
```

- The depth sensor also has per-parameter setters, such as
  `set_noise_sigma` and `set_confidence_threshold`
  (`$CAMX/single_view_depth_sensor.py:55-237`).
- **Intrinsics at 848×480:** fx = fy = 420.08, cx = 424, cy = 240 (exp2).
  This equals 848 × 1.93 / 3.896.
- **Saved frames** under `W/`: `rgb_home.png`, `depth_home.npy`/`.png`
  (noisy), `depth_gt_home.npy`/`.png`, `valid_mask_home.png`, plus
  `*_moved*` variants.

### 4.3 Depth sensor output (observed exp2, exp2c)

- **Q7.** The log shows `Graphics API: D3D12`, and all three outputs
  were produced at 2048×1080 and 848×480 (exp2, exp2c).
- **Units.** At 848×480 the median noisy/GT ratio of valid pixels is
  10 003.9; at 2048×1080 it is 9 999.1. The stage has
  `metersPerUnit` 1.0. The extension docs list `DepthSensorDistance` as
  "Distance in meters"
  (`$ISAAC/extscache/omni.sensors.nv.camera-1.0.0+69cbf6ad.wx64.r/docs/camera_extension.rst:551`).
  Invalid pixels are exactly 0.0.
- **Valid fraction** at 848×480 depends on `confidenceThreshold` in the
  opposite direction from what the name suggests:

  | `confidenceThreshold` | Valid pixels |
  |---|---|
  | 0.95 (default) | 13.2 % |
  | 0.5 | 1.6 % |
  | 0.99 | 16.7 % |

  At native resolution, 11.6 % are valid. The docs describe confidence
  from Sobel edges of the color image, so textureless regions are
  discarded (`camera_extension.rst:486-617`). `W/valid_mask_home.png`
  shows valid pixels only along intensity edges.
- **Accuracy** after ÷ 10⁴: relative RMS against GT over valid pixels is
  1.9 % (2048×1080) and 2.9 % (848×480) (exp2c).
- **Temporal noise.** Consecutive frames of a static scene differ in the
  noisy output, including pixels flipping between valid and 0. GT differs
  by at most 5e-6 m (exp2c/exp2d/exp2e, `static_frame_diff`).
- **Noise model** (`camera_extension.rst:486-617`): single-view disparity
  reprojection with Gaussian disparity noise. The parameters are listed
  there, and `rgbDepthOutputMode` takes 0-7.

### 4.4 Latency and cost (observed exp2d, exp2e)

- **Annotator latency.** After moving an object, the change appeared in
  RGB and GT at the 5th `render()` with default settings (exp2d). It
  appeared at the 2nd `render()` with `--/app/asyncRendering=0
  --/app/asyncRenderingLowLatency=0 --/omni/replicator/asyncRendering=0
  --/app/hydraEngine/waitIdle=1` (exp2e). Frame-time cost of those flags
  was not measured separately.
- **`render()` cost** with the depth sensor and annotators (GPU shared,
  see caveat):

  | Scene | Time per render |
  |---|---|
  | no lights | 52 ms |
  | lit, 848×480 | 115-210 ms |
  | lit, 2048×1080 | 317 ms |

  Annotator reads took 10-11.5 ms (exp2, exp2c, exp3b).
- **Warm-up.** With a near-empty stage, the first non-empty RGB frame
  needed about 5 s of wall-clock `app.update()`. 120 quick updates in
  2 s returned empty arrays (exp5b-exp5d).

### 4.5 Publishing to ROS 2

- **Camera Helper** (`ROS2CameraHelper`, types `rgb`, `depth`,
  `depth_pcl`, `instance_segmentation`, `semantic_segmentation`,
  `bbox_2d_tight`, `bbox_2d_loose`, `bbox_3d`; `$BR/isaacsim/ros2/bridge/ogn/python/nodes/OgnROS2CameraHelper.ogn`):
  - **`depth` is `DistanceToImagePlane`, i.e. ground truth**
    (`OgnROS2CameraHelper.py:127-139`). No Camera Helper type publishes
    `DepthSensorDistance`.
  - On the depth sensor's render product it published `32FC1` 848×480
    depth and `rgb8` RGB (exp3b). The graph was on-demand and evaluated
    once with `evaluate_sync`, the `camera_manual.py` pattern. That
    evaluation creates the writers, which then published on every
    `render()`. They kept publishing after `inputs:enabled` was set
    False without re-evaluating the graph.
  - With the 1400 B profile the container counted 30 CameraInfo, 28 RGB
    and 21 depth messages. The run had 15 renders in the camera section
    plus about 16 during the later service calls (exp3b, peer `/p2g/*`).
  - **Stamps** are physics time. After render *k*, the newest received
    message was stamped with the physics time of render *k − 1* (exp3b,
    `graph_camera.rows`). Whether the image content matches that stamp
    was not checked; the annotator lag above suggests it may not (§9).
- **CameraInfo Helper** published `K` = [420.08, 0, 424; 0, 420.08, 240;
  0, 0, 1], `plumb_bob`, `D` all zeros, 848×480 and the configured
  `frame_id` (exp3b).
- **Script `rclpy`** published RGB, noisy depth (÷ 10⁴, `32FC1`) and GT
  depth as `sensor_msgs/Image`. All 63/63 frames of each arrived with the
  1400 B profile (exp3b). Cost is in §3.3.

---

## 5. `isaacsim.ros2.sim_control` (Q5)

### 5.1 Services

- **Services** (`$SC/simulation_control.py:48-65`), with an empty name
  prefix (`:41`):
  - simulation state: `get_simulation_state`, `set_simulation_state`,
    `reset_simulation`, `step_simulation`, `get_simulator_features`
  - entities: `get_entities`, `get_entity_info`, `spawn_entity`,
    `delete_entity`, `get_entity_state`, `get_entities_states`,
    `set_entity_state`
  - worlds: `load_world`, `unload_world`, `get_current_world`,
    `get_available_worlds`
- **Action:** `simulate_steps` (`:67-69`).
- **Message package:** `simulation_interfaces` 1.1.0 is in Isaac's bundle.
  It is **not** in the container image (`ls /opt/ros/humble/share`,
  observed). Whether the apt package can be installed was not checked.

### 5.2 Handlers need `app.update()`

- **Mechanism.** Every handler is a coroutine submitted from the ROS
  executor thread with `asyncio.run_coroutine_threadsafe(…, self.loop)`
  and awaited with `future.result()` (`$SC/simulation_control.py:334-351`).
  `self.loop` is the main thread's event loop (`:146-148`), which runs
  inside `app.update()`. Several handlers also `await next_update_async()`.
- **Observed (exp3b).** Calls came from the script's own `rclpy` client
  in the same process. `get_entities`, `spawn_entity`,
  `get_entities_states` and `delete_entity` each did not complete during
  3 s of `step(render=False)` + `spin_once`. Each completed after 3 renders
  (`spawn_entity`: 4).
  - In `run_scene.py` such calls are therefore served only during camera
    renders, with a delay of several renders.
  - The design's 1 s free-running timeout is unaffected, but a scene
    call costs at least about 100 ms of simulation time at one render per
    33 ms.

### 5.3 What the services do (observed exp3b; source)

- **`SpawnEntity`** (`:757-944`):
  - Accepts only USD files (`is_valid_usd_file`, `:780`, checks the
    extension). `resolve_asset_path_async` falls back to the asset root
    (`:786`).
  - It defines the prim, adds a reference (`:886`) and sets the world
    pose (`:893,914`). It tags the prim with the custom attribute
    `simulationInterfacesSpawned = True` (`:920`).
  - `allow_renaming = false` with an existing name returns
    `NAME_NOT_UNIQUE` (`:849-851`).
  - Observed: a local `C:/isaac-sim-work/phase2/spawn_cube.usda` (rigid
    0.05 m cube, mass 0.1) spawned at `/World/spawned_cube` (0.4, −0.3,
    0.3) with result 1 (OK). The prim had `PhysicsRigidBodyAPI` and
    `PhysicsMassAPI` and the attribute.
  - After 800 physics-only steps it lay on the ground (z = 0.025). A body
    spawned while the script steps physics itself is simulated without a
    timeline restart.
- **`GetEntities`** (`:619-669`) with filter `""` returned 145 entities.
  These are every prim except `/Render`, including
  `/OmniverseKit_Persp`, `/physicsScene` and ground sub-prims. A regex
  filter `^/World/cube$` returned just that prim.
- **`GetEntitiesStates`** (`:1129-1194`; `entity_utils.py:66-204`)
  returned correct positions for `/World/cube` and `/World/spawned_cube`.
  The header stamp was 0 and `frame_id` was the prim name (`cube`,
  `spawned_cube`), not a world frame.
- **`DeleteEntity`** (`:671-717`) removed the spawned cube. The robot's
  articulation handle stayed valid and its joints were unchanged.

### 5.4 `ResetSimulation` and timeline stop/play

`ResetSimulation` (`:946-1012`) runs these steps:
1. `timeline.stop()`
2. `next_update_async()`
3. deletes every prim with `simulationInterfacesSpawned`
4. `next_update_async()`
5. `timeline.play()` (`:978,981,998,1001,1004`)

Observed (exp3b): `sim.stop(); app.update(); sim.play(); app.update()`
caused the following:
- The articulation handle became invalid and `get_joint_positions()`
  returned None.
- `sim.current_time` went from 22.28 s to 0.035 s.
- After `robot.initialize()`, the joints were at about 0, close to the USD
  default pose, instead of the commanded home pose:
  (0.0016, −0.0568, 0, −0.0875, 0, 0.0858, 0.0913, 0, 0).

`StepSimulation` requires the simulation to be paused (`:1014-1087`).

---

## 6. `/sim/reset`, `/sim/attach_board`, `/sim/detach_board` (Q6)

### 6.1 Serving `std_srvs/Trigger` (observed exp7, container client, 1400 B profile)

| Server in Isaac | Result |
|---|---|
| Script `rclpy` `create_service(Trigger, …)`, spun with `spin_once(timeout 0)` once per 4-step cycle | 5/5 calls succeeded, 2.7-12 ms round trip, callback ran between physics steps |
| Bridge `OgnROS2ServiceServerRequest` + `OgnROS2ServiceServerResponse` (`$BR/ogn/docs/OgnROS2ServiceServerRequest.rst:21`, `…Response.rst:21`), on-demand graph, `evaluate_sync` per cycle, an `omni.graph.action.Counter` on `onReceived` to detect requests | 5/5 calls answered, 2.7-5.4 ms. **The reply is sent during the evaluation that receives the request**, with the response inputs' values at that moment. The first call got `success=false, message=""`; each later call got the values the script wrote after the previous request |

Notes on the bridge nodes:
- **Wiring.** The node pattern follows Isaac's test: `onReceived` →
  `onReceived` and `serverHandle` → `serverHandle`
  (`$BR/isaacsim/ros2/bridge/tests/test_service.py:42-99`).
- **Field names.** The Trigger response fields appear as
  `inputs:Response:success` and `inputs:Response:message` only after the
  Response node has computed once. They must be addressed as
  `og.Controller.attribute("inputs:Response:success", node)`. A path
  string `/Graph/Resp.inputs:Response:success` fails to resolve (exp7).
- **Request output.** `Trigger`'s empty request appears as
  `outputs:Request:structure_needs_at_least_one_member`.

The container has `std_srvs` (observed listing), so these calls work Fast
DDS ↔ Fast DDS. Phase 0.5 found that services fail across vendors
(CycloneDDS ↔ Fast DDS, `phase_0_5_notes.md`).

### 6.2 Runtime fixed joint to `panda_hand` (observed exp1)

- **Attach.** While the timeline was playing, the script created:
  - a `UsdGeom.Cube` with `RigidBodyAPI` + `MassAPI` (0.5 kg)
  - a `UsdPhysics.FixedJoint` with body0 `/World/franka/panda_hand`,
    body1 the board, and a `localPos0` offset

  Result:
  - The articulation handle stayed valid.
  - After a 0.5 rad joint-1 move, the board followed the hand within
    0.0003 mm, both before and after a render.
  - The measured hand force rose by the board's weight (§2.4).
- **Detach.** Removing the joint and board prims kept the handle valid.
  The joints were unchanged, and the hand force returned to the static
  value (5.2307, −1.6326, 1.7526).
- **Resetting the robot** without a timeline stop:
  - `set_joint_positions` + `apply_action` of the same targets moved the
    robot to the commanded pose (exp1, exp3b home pose).
  - Joint velocities were not explicitly zeroed in these runs.
  - Removing spawned objects can use `DeleteEntity` or USD `RemovePrim`.
    Both kept the handle valid (§5.3, §6.2).

---

## 7. Catalog assets (Q7)

### 7.1 Shipped props (observed exp5, `omni.client.list` on the asset root)

| Need | Shipped candidates (`/Isaac/Props/…`) |
|---|---|
| Table | `Mounts/table.usd`, `Mounts/textured_table.usd`, `Mounts/thor_table.usd`, `Mounts/SeattleLabTable/`, `Mounts/ThorlabsTable/`, `PackingTable/packing_table.usd` |
| Cube | `Blocks/basic_block.usd`, `block.usd`, `block_instanceable.usd`, `red_block.usd`, `blue_block.usd`, `green_block.usd`, `yellow_block.usd`, `nvidia_cube.usd`, `Blocks/DexCube/`, `Blocks/MultiColorCube/`; `Shapes/cube.usd`; `YCB/Axis_Aligned/036_wood_block.usd`, `061_foam_brick.usd` |
| Plate | none named "plate". Closest: `Shapes/disk.usd`, `YCB/Axis_Aligned/024_bowl.usd` |
| Peg / hole block | none named "peg" or "hole". `Factory/` holds `factory_bolt_m{4,8,12,16,20}_{loose,tight}`, `factory_nut_m…_{loose,tight}` and `gear_assets` |
| Calibration target | `Camera/checkerboard_6x10.usd` (+ `.png`): a checkerboard, not ChArUco |

Other `Props` directories:
- `Beaker`, `Conveyors`, `DeformableTube`, `Dolly`, `Flip_Stack`
- `Food`, `Forklift`, `KLT_Bin`, `Mugs`, `Pallet`
- `Rubiks_Cube`, `Sektion_Cabinet`, `Sortbot_Housing`, `UIElements`
- `YCB/Axis_Aligned_Physics` (4 objects)

Isaac's code references `Blocks/basic_block.usd` most often (5 times),
then YCB objects (`rg` over `$ISAAC/standalone_examples` and
`$ISAAC/exts`).

The props' physics schemas, units and mesh quality were not inspected.
Directories outside `/Isaac/Props` and `/Isaac/Environments`, such as an
IsaacLab peg-in-hole set, were not listed (§9).

### 7.2 GLB → USD with `omni.kit.asset_converter` in a standalone script (observed exp5)

- **Setup.** `enable_extension("omni.kit.asset_converter")` worked headless.
  The API is `converter.get_instance().create_converter_task(import_path,
  output_path, progress_callback, asset_converter_context)`, then `await
  task.wait_until_finished()` (`$AC/omni/kit/asset_converter/impl/extension.py:72-129`;
  `$AC/docs/Usage.md:60-130`). The script drove the coroutine with
  `asyncio.ensure_future` and `app.update()` until it was done.
- **Test input.** A 0.05 × 0.03 × 0.02 m box written by `trimesh` 4.5.1
  as GLB (1292 B). `trimesh` came from
  `$ISAAC/exts/omni.pip.compute/pip_prebundle`, added to `sys.path`.

| `AssetConverterContext` | Result | `metersPerUnit` | `upAxis` | Extent |
|---|---|---|---|---|
| defaults | OK, 0.76 s | **0.01** | Y | ±2.5, ±1.5, ±1.0 (cm) |
| `use_meter_as_world_unit = True` | OK, 0.35 s | 1.0 | Y | ±0.025, ±0.015, ±0.01 (m) |

- **Output prims:** `/World` (default prim, Xform) →
  `/World/geometry_0` (Xform) → `/World/geometry_0/geometry_0` (Mesh).
- **Axes.** The vertex axes were kept as written; no up-axis rotation was
  applied. The stage is labelled Y-up, per glTF convention. The context
  has `convert_stage_up_z` (`Usage.md:90-130`), which was not tried.
- **No physics schemas** were applied, so collision, rigid body and mass
  must be added by `build_catalog.py`.

### 7.3 ChArUco texture (observed exp5e)

- **Board image.** `cv2` 4.11.0 (`opencv_python_headless`, from the same
  `omni.pip.compute` prebundle) generated it with
  `cv2.aruco.CharucoBoard((7, 5), 0.03, 0.022,
  getPredefinedDictionary(DICT_5X5_100)).generateImage((1400, 1000),
  marginSize=20)` → `W/charuco_7x5.png`.
- **Board in the scene.** The image was applied to a 0.21 × 0.15 m quad
  mesh with an `st` primvar, through `UsdPreviewSurface` ← `UsdUVTexture`
  ← `UsdPrimvarReader_float2`. Rendered at 848×480 from 0.35 m above,
  **`cv2.aruco.CharucoDetector` found all 17 markers and all 24 ChArUco
  corners** (`W/charuco_render_e.png`).
- **Scale.** The PNG includes a 20 px margin, so the printed squares on a
  0.21 m quad are 0.03 × 1360/1400 = 29.1 mm, not 30 mm. The quad must be
  sized to the margin, or the margin set to 0.
- **Near clipping.** `isaacsim.sensors.camera.Camera` created its prim
  with USD's default `clippingRange` (1.0, 1e6). In a meter stage nothing
  nearer than 1 m renders: the first renders were uniformly gray
  (exp5-exp5d). `set_clipping_range(0.01, 100)` fixed it (exp5e).
  `rsd455.usd`'s own cameras were not checked for this.

---

## 8. Facts that conflict with the design text

Facts only; each is cited above.

| Design text (`robot_platform.md`) | Fact |
|---|---|
| Gripper: "both finger joints to width/2; finger drive max force = max effort / 2" (`:334`) | Only `panda_finger_joint1` has a drive. `panda_finger_joint2` is a mimic joint with no drive and max effort 0. The drive max force defaults to 7.2 N, not 20 N (§1.3). Whether the mimic joint passes the full drive force to the second finger was not tested |
| `/gripper_command` is `control_msgs/msg/GripperCommand` (`:334`) | `control_msgs` is in neither Isaac's bundle nor the container image (§3.5) |
| Camera via `SingleViewDepthSensorAsset` + ROS2 Camera Helper, with `/camera/depth` "32FC1, meters, with sensor noise" (`:335`) | Camera Helper's `depth` is ground truth only. Noisy depth comes from the `DepthSensorDistance` annotator, in units about 10⁴ × meters, with 12-15 % valid pixels (§4.3, §4.5) |
| Camera 848×480 (`:311`) | The template render product is 2048×1080; `set_resolution((848, 480))` works (§4.1, §4.2) |
| D7: all I/O through bridge components (`:322-327`) | Bridge nodes cover state, wrench, command, camera RGB/GT/info and services. The generic service server replies before the script acts (§6.1). Noisy depth has no Camera Helper type (§4.5) |
| Camera topics reach `camera_sim` in the container (`:300-302`) | Not with `container/dds/fastdds.xml` as is; needs `maxMessageSize` ≤ 1400 B on both sides (§3.3) |
| `/wrist_wrench` "frame `panda_hand`" from the measured joint force (`:332`) | The API reports it in the joint's child frame. A rotation by `localRot1` is needed to get `panda_hand` axes (§2.4) |
| "Rendering for the camera runs every 33 ms of simulation time" (`:357`) | `render()` advances the timeline but not physics, and `app.update()` while playing advances physics (§2.1). Measured `render()` + 3-image publish brings RTF to 0.12 at one render per 8 cycles (§3.2), under GPU contention |
| `sim_control` used for scene calls during lockstep (`:336`) | Its handlers only run inside `render()`/`app.update()`, 3-4 renders per call (§5.2) |
| The RSD455 is placed as a fixed camera | Its root is a dynamic rigid body and falls unless kinematic (§4.1) |

---

## 9. Not verified

- **Render timings.** The GPU was shared with another application during
  all camera runs. Timings without it, and the cost of the synchronous
  rendering flags, were not measured. GPU memory use of Isaac was not
  measured.
- **Noisy depth scale.** The cause of the ≈ 10⁴ factor on
  `DepthSensorDistance` is unknown. Whether it is a fixed factor or
  depends on `metersPerUnit` or on another setting was not tested.
  Whether the noise model parameters (`focalLengthPixel` 897,
  `sensorSizePixel` 1280) should change with resolution was not checked.
- **Camera Helper stamp/content alignment.** Whether a Camera Helper
  message's content matches its physics-time stamp, given the 1-4 render
  annotator lag, was not checked.
- **Rendering with USD writeback off.** Whether the renderer shows the
  robot's physics pose with `/physics/updateToUsd` False was not checked.
- **Root cause of the image loss.** IP fragment loss on the mirrored
  loopback is inferred from the size cutoff, not from a packet capture.
  Behaviour with the 1400 B profile under sustained 30 Hz × 3 images
  (about 135 MB/s) was not tested; exp3b published at one frame per
  8 cycles.
- **Graph-published JointState content.** It was not compared with physics
  in-process (§3.4). The peer received all messages and echoed their
  stamps. `/clock` via ROS2 Publish Clock was not tested.
- **Offline asset use.** Running with no network from the omni client
  cache was not tested.
- **Finger force.** Whether the finger mimic joint respects the finger
  drive's max force, and the grasp force actually achieved, were not
  tested. Finger-frame FK was not compared with Lula.
- **Container `simulation_interfaces`.** Whether it is installable by apt
  in the container image was not checked (`apt-cache policy` printed
  nothing).
- **Bazel message packages.** The packages inside intrinsic-core's
  `ros2_common_interfaces` (external knowledge only).
- **Props.** Physics schemas, scale and suitability of the shipped props,
  and asset directories outside `/Isaac/Props` and `/Isaac/Environments`.
- **Asset converter axes.** `convert_stage_up_z`, and how a Y-up
  converted USD composes into the Z-up scene. The `omni.usd.metrics.assembler`
  extension is loaded (`W/scripts/exp5c.log`) but its effect was not
  checked.
- **Unexplained run.** One exp3 run reported `peer_matched False` while the
  peer counted 846 state messages. The rerun matched normally
  (`W/scripts/exp3.log` was overwritten by the rerun). Not investigated.
