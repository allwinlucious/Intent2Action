# Phase 2 implementation notes

Status: **stopped on four issues that need changes outside the plan.**
Six of ten checks pass: `lockstep`, `free_run`, `gripper`, `camera`,
`entities` and `standalone`. Four fail: `wrench`, `tf`, `camera_fresh` and
`reset`. Each failure comes from a wrong plan value or plan assumption, or
from a missing piece of the plan; none is a bug in the implementation. Each
fix is a design or configuration change, so the worker recorded them
without making them (see "Blocking issues"). Step 5 (`camera_request_renders`)
depends on `camera_fresh`, so it was not done; the value stays at the plan's
starting value of 3.

All plan files exist (steps 1–4). The catalog, the ChArUco board and the
image are built. No containers or Isaac processes from this work are left
running.

## Check output

`.venv/bin/python tests/e2e/phase2_check.py` (final run, exit 1; also in
`logs/phase2_check_final.out`):

```
isaac READY pid=25128
PASS lockstep: cycles 2000 missed 0 max tracking error 0.00157 rad rtf 0.2194138746532631
PASS free_run: sim advanced 0.27200001300000004 s during the pause; lockstep after 1 cycles
PASS gripper: width 0.0: finger 0.0019 m; width 0.08: finger 0.0381 m
FAIL wrench: home [0.006, 0.0, 5.753] N (off by 2.458); board rise 1.962 N (want 1.962 ±5%, attach attached); after detach off by 0.000 N (detached)
FAIL tf: missing []; d455_optical parent Camera_Pseudo_Depth; table 0.000 mm 0.000 deg, hole_block 0.000 mm 0.000 deg, Camera_Pseudo_Depth 0.000 mm 180.000 deg, red_cube 0.00 mm, plate 0.00 mm, peg 0.00 mm
PASS camera: rgb rgb8 848x480, depth 32FC1 848x480 median 2.057 m, K [420.08, 0.0, 424.0, 0.0, 420.08, 240.0, 0.0, 0.0, 1.0], frames ['d455_optical'], stamps [0.004, 0.004, 0.004] s after request
FAIL camera_fresh: mean abs diff 0.00; frame stamped +0.0040 s, received 0.04 s wall after the request
PASS entities: listed ['/World/objects/table', '/World/objects/red_cube', '/World/objects/plate', '/World/objects/peg', '/World/objects/hole_block']; spawn 1 ; on /tf True; delete 1; gone True
FAIL reset: spawn 1; reset epoch 1; max joint error 0.00090 rad; extra_cube gone True; epoch None -> None; table 0.000 mm 0.000 deg, hole_block 0.000 mm 0.000 deg, red_cube 0.00 mm, plate 0.00 mm, peg 0.00 mm
PASS standalone: 0 containers running; 2 progress lines in 30 s (last: cycles 778 (+451) rtf 0.17 free_run_cycles 778 renders 19 (+10, 534 ms each) epoch 0); 0 Error/Traceback lines []
elapsed 135.3
```

## Blocking issues

### 1. `wrench`: the expected force belongs to a different pose

At the plan's home pose (`ARM_HOME` = 0, −0.785, 0, −2.356, 0, 1.571,
0.785) the hand points straight down. The load on the sensor is therefore
(0.006, 0.000, 5.753) N, along +z of `panda_hand`, as decision 8 itself
says. The plan's expected value (−2.394, −0.085, +5.231) N has the same
magnitude (5.753 N) but was measured in exp1 at another pose (q2 = −1.3,
q4 = −2.87, q6 = 2.0; Inv §2.4). The other two parts pass: attaching the
board raises the magnitude by 1.962 N (0.2 kg × 9.81), and detaching
restores it exactly.

Needed: a new expected value in the plan, (0, 0, 5.753) N at home or the
exp1 pose for this check. That is a change to the plan's criteria.

### 2. `tf`: the bridge already publishes camera prims in ROS optical convention

Publish Transform Tree publishes `Camera_Pseudo_Depth` rotated 180° about
its x axis compared with the USD prim. Its +z points along the view
direction and its y points down, i.e. ROS optical convention. Evidence:

*   Published: position (1.3, 0, 0.9), rotation xyzw (0.6533, 0.6533,
    −0.2706, −0.2706). Its +z is (−0.707, 0, −0.707), toward the table.
    The USD prim's −z is the same vector.
*   The camera really looks that way: the depth at the image centre is
    1.271 m, and 0.9 · √2 = 1.273 m is where the 45°-down view ray meets
    the table top.
*   The bridge plugin (`isaacsim.ros2.bridge.plugin.dll`) references
    `UsdGeomCamera`, which fits special handling of camera prims.

Two consequences:

*   The `tf` check expects the USD orientation from the scene file, so it
    fails by exactly 180°. All other frames match to 0.000 mm / 0.000°.
*   Decision 10's Raw Transform Tree adds another 180° about x, so
    `d455_optical` ends up in USD convention (−z forward, y up), the
    opposite of its intent. Images are stamped `d455_optical`, so
    phase 3b would project with the wrong frame.

Needed: change decision 10. Either make `Camera_Pseudo_Depth` →
`d455_optical` the identity (or drop that node and name the images
`Camera_Pseudo_Depth`), and state the expected `Camera_Pseudo_Depth`
orientation in ROS optical convention. Both are design choices.

### 3. `camera_fresh`: the scene has no lights, so RGB is black

The plan's scene (`sim/scenes/default.yaml`, file 8 `scene.build`) adds no
light, so RGB frames are black. Mean channel values are [0, 0.03, 0.09]
out of 255, with a standard deviation of 2.6. Moving joint 1 by 0.5 rad
therefore changes nothing: the mean absolute difference is 0.00 against
the required > 2. Depth is fine because it does not depend on light.

Needed: lights in the scene (type, pose, intensity). That is a scene
design decision. Step 5 (the smallest N for which `camera_fresh` passes
10/10) depends on this check, so it was not done; `camera_request_renders`
stays at 3.

### 4. `reset`: Fast DDS late joiners don't get the latched `/sim/epoch` across Windows ↔ WSL

Isaac's epoch publisher is correct: reliable, transient-local,
lifespan and deadline infinite, as reported by discovery. Findings:

| Subscriber in the container | Joins after the publish | Subscribed during a reset |
|---|---|---|
| CycloneDDS, transient-local | gets the latched value | — |
| Fast DDS (`container/dds/fastdds.xml`), transient-local | **gets nothing** | gets it |
| Fast DDS with `<data_sharing><kind>OFF</kind>` for readers and writers | gets the latched value | — |

So Fast DDS data-sharing (on by default, AUTO) breaks late-joiner
delivery between the two hosts. A likely cause: under mirrored networking
both sides share 127.0.0.1 and look like the same host. The io probe must
use Fast DDS, because cross-vendor services don't work, so it never learns
the previous epoch (`None -> None`). The other reset criteria pass: joints
at home within 0.0009 rad, `extra_cube` gone, objects at their scene poses.

Needed: turn data-sharing off in `container/dds/fastdds.xml` (the
diagnostic profile was a temporary copy in `/tmp`). That changes a file
beyond decision 14, so it was not applied. Any Fast DDS node that starts
after Isaac (e.g. later `ros2_hwm`, `camera_sim`) has the same problem.

## Measurements

*   **RTF, lockstep, no camera requests:** 0.22–0.44 across `lockstep`
    runs (2000 cycles); 0.275 in a separate 3000-cycle run.
    **Free-run (standalone):** 0.17–0.33 per 10 s window.
*   **RTF, lockstep, one `/camera/request` per second wall**
    (`phase2_peer_hwm.py --request-period-s 1`, 3000 cycles): 0.094 over
    the run. Isaac's 10 s windows show 0.02–0.06 in steady state: 33 renders
    per 10 s (about 10 requests × 3 renders plus the periodic ones) at
    about 280 ms each leave almost no time for stepping.
*   **Render cost** (one `sim.render()`, D3D12, synchronous flags): mostly
    190–430 ms, about 300 ms typical. The range seen is 59–534 ms. That is
    above the plan's 115–210 ms (decision 3). Periodic renders alone
    (about 12–17 per 10 s) take 40–50 % of wall time without requests.
*   **`camera_request_renders`:** not measured (blocking issue 3); stays 3.
*   **`panda_hand_joint` localPos1:** (−3.8e-8, 3.8e-8, −6.0e-8) m,
    effectively zero. The torque shift of decision 8 is applied but has no
    effect.
*   **Camera intrinsics** (`Camera.get_intrinsics_matrix()`):
    [[420.0821, 0, 424], [0, 420.0822, 240], [0, 0, 1]]. The bridge's
    CameraInfo forces fy = fx. Isaac adjusts the vertical aperture
    0.2453 → 0.2205 for square pixels (warning at start-up).

## Implementation choices within the plan

These interpret the plan where it gives no detail; none changes a decision.

*   **`/tf` targets are applied by re-creating the Transform Tree node.**
    The node reads rigid bodies through a PhysX tensor view built on its
    first evaluation. It ignores targets added later (`extra_cube` never
    appeared). Deleting one of its bodies invalidates that view for good,
    which logs `Simulation view object is invalidated … getTransforms`
    every publish and returns one body's pose for all of them. So
    `Graphs.set_tf_targets` deletes and re-creates the `Tree` node whenever
    the target list changes, and always after `/sim/reset`. This is how
    decision 10's "re-read after every render and after a reset" takes
    effect.
*   **Hwm probe, sine time base:** the sine uses τ, which starts at 0 at the
    first state pair and advances by the stamp delta. Across the pause it
    advances by one cycle, so the command doesn't jump after the pause.
    The tracking error skips the first 50 cycles after the start and after
    the resume, while the arm catches up from rest.
*   **Hwm probe, resume detection:** "lockstep resumes" = number of peer
    replies before a state whose joint-1 velocity steps by > 0.05 rad/s.
*   **Hwm probe extras:**
    *   `--request-period-s` publishes `/camera/request` (for the RTF
        measurement).
    *   RTF is over the pre-pause section.
    *   Containers get `--name phase2_<probe>`.
*   **Io probe, `/tf` subscription depth 10:** two bridge nodes publish
    `/tf` back to back, and keep-last 1 drops one of them. All other probe
    subscriptions use the plan's QoS.
*   **`extra_cube` spawn pose:** (0.30, −0.30, 0.0). The plan gives none;
    this spot is free on the table.
*   **Expected camera pose** (`phase2_check.expected_scene`): the D455 pose
    from `default.yaml`, composed with the asset-internal rotations logged
    by exp2a (RSD455 wxyz (0, 0, 0, 1), Camera_Pseudo_Depth wxyz (0.5,
    0.5, 0.5, 0.5); Inv §4.1).
*   **Wrench tolerances:** the 0.3 N and 0.1 N tolerances are on the
    Euclidean distance between force vectors.
*   **`camera_fresh`:** each trial moves joint 1 back by −0.5 rad
    afterwards. Periodic 2 Hz renders also refresh the image during the
    1.5 s move, so this check measures N only weakly; step 5 will need
    that in mind.
*   **`reset` pose criterion:** checks only the objects under
    `/World/objects`, not the camera.
*   **Run-scene log line:** also reports render time (`M ms each`).
*   **Helpers added:** `scene.object_paths()` (used for the `/tf` targets)
    and `sync_isaac.sh --pull-catalog` with `cp --no-preserve=mode` (files
    copied back from `/mnt/c` came back executable).
*   **`sim/tools/build_catalog.py`** starts a headless `SimulationApp`
    because bare `pxr` does not import in Isaac's Python.

## Files

Created:

*   `config/timing.yaml`
*   `tools/sync_isaac.sh`, `tools/build_catalog_meshes.py`,
    `tools/make_charuco.py`
*   `requirements-dev.txt`
*   `sim/tools/build_catalog.py`, `sim/scenes/default.yaml`, `sim/scene.py`,
    `sim/graphs.py`, `sim/run_scene.py`
*   `tests/e2e/phase2_peer_hwm.py`, `tests/e2e/phase2_peer_io.py`,
    `tests/e2e/phase2_check.py`
*   `assets/objects/{table,red_cube,plate,peg,hole_block}/{mesh.glb,object.yaml,model.usd}`
*   `assets/calib/charuco_7x5.png`
*   this file

Modified:

*   `container/dds/fastdds.xml` (`maxMessageSize` 1400)
*   `container/Dockerfile` (`python3-yaml`, `simulation_interfaces` 1.1.0 into
    `/opt/ros_ext`, `COPY config`)

Logs (git-ignored) are in `logs/phase2_*`.
