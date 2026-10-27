# Phase 2 implementation notes

Status: **done.** All ten checks pass in three consecutive runs after
Addendum A (see "Addendum A" at the end). The sections before it describe
the first implementation run and are kept as found; where Addendum A
showed them wrong, they say so.

First run: six of ten checks pass: `lockstep`, `free_run`, `gripper`, `camera`,
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

*Wrong, see Addendum A:* the epoch publisher had not published yet; data-sharing
plays no part.

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

*   `container/dds/fastdds.xml` (`maxMessageSize` 1400; the line was in
    fact missing until Addendum A)
*   `container/Dockerfile` (`python3-yaml`, `simulation_interfaces` 1.1.0 into
    `/opt/ros_ext`, `COPY config`)

Logs (git-ignored) are in `logs/phase2_*`.

## Addendum A

All ten checks pass in three consecutive runs (below). There were three
unstable checks, and each had its own cause.

### Check output

`.venv/bin/python tests/e2e/phase2_check.py`, three runs in a row, all exit 0
(`logs/phase2_check_addA2_run{1,2,3}.out`; Isaac and probe logs next to
them). The GPU was shared with other work during these runs, so renders took
220–340 ms and RTF was lower than when unloaded (see "Measurements").

```
isaac READY pid=23404
PASS lockstep: cycles 2000 missed 0 max tracking error 0.00263 rad rtf 0.4026846840692575; started after 1 replies; latched epoch 0
PASS free_run: sim advanced 0.556000026 s during the pause; lockstep after 2 cycles
PASS gripper: width 0.0: finger 0.0019 m; width 0.08: finger 0.0381 m
PASS wrench: home [0.006, -0.0, 5.753] N (off by 0.007); board rise 1.963 N (want 1.962 ±5%, attach attached); after detach off by 0.000 N (detached)
PASS tf: missing []; d455_optical parent Camera_Pseudo_Depth; table 0.000 mm 0.000 deg, hole_block 0.000 mm 0.000 deg, Camera_Pseudo_Depth 0.000 mm 0.000 deg, red_cube 0.00 mm, plate 0.00 mm, peg 0.00 mm
PASS camera: rgb rgb8 848x480, depth 32FC1 848x480 median 2.057 m, K [420.08, 0.0, 424.0, 0.0, 420.08, 240.0, 0.0, 0.0, 1.0], frames ['d455_optical'], stamps [0.004, 0.004, 0.004] s after request
PASS camera_fresh: mean abs diff 1.08; frame stamped +0.0040 s, received 0.24 s wall after the request
PASS entities: listed ['/World/objects/table', '/World/objects/red_cube', '/World/objects/plate', '/World/objects/peg', '/World/objects/hole_block']; spawn 1 ; on /tf True; delete 1; gone True
PASS reset: spawn 1; reset epoch 1; max joint error 0.00090 rad; extra_cube gone True; epoch 0 -> 1; table 0.000 mm 0.000 deg, hole_block 0.000 mm 0.000 deg, red_cube 0.00 mm, plate 0.00 mm, peg 0.00 mm
PASS standalone: 0 containers running; 2 progress lines in 30 s (last: cycles 1483 (+785) rtf 0.31 free_run_cycles 1483 renders 3 (+2, 266 ms each) epoch 0); 0 Error/Traceback lines []
elapsed 143.4
```

Runs 2 and 3 are identical except for these numbers:

*   `lockstep` rtf: 0.387 and 0.402.
*   `free_run`: 0.544 s and 0.512 s of sim time during the pause; lockstep
    after 1 cycle in both.
*   `camera_fresh` diff: 1.08 and 1.17, received 0.53 s and 0.52 s after
    the request.
*   `standalone` render time: 343 ms and 222 ms.

An earlier set of three runs, before the last hwm probe fix, failed once:
`lockstep` with `missed 1` in run 2 (`logs/phase2_check_addA_run{1,2,3}.out`).
See section 3 below.

### 1. `camera_fresh`

There were three causes.

*   **`maxMessageSize` was missing from `container/dds/fastdds.xml`.**
    The notes listed it as done, but the line wasn't there, so image-sized
    datagrams were IP-fragmented on the WSL loopback. With the line restored
    and the image rebuilt, frames arrive at N = 6 too. The "no frame at all"
    case wasn't seen again.
*   **The threshold of 2 can't be reached.** The 0.5 rad joint-1 move
    changes the image by a mean absolute difference of 1.03–1.17 (white arm
    over a light floor, in a small part of the image). The arm visibly moves
    in the saved frames. Two requested frames of an unchanged scene differ
    by 0.1–0.3, which is RTX temporal noise that never settles. A known
    stale frame differs by 0.04. So the probe now passes on
    `diff > FRESH_MIN_DIFF` = 0.6.
*   **Every render published images, and a closed gate takes effect one
    render late.** The Camera Helper's writers publish from each render,
    and the probe takes the first frame stamped at or after the request.
    The first render after physics steps can still show the previous
    state: with N = 1 the frame was stale in 4 of 10 trials (diff
    0.08–0.31). Gating the writers per render showed a second effect. The
    per-render-product `IsaacSimulationGate` nodes
    (`LdrColorSDIsaacSimulationGate`,
    `DistanceToImagePlaneSDIsaacSimulationGate`, `inputs:step` 0/1) let the
    render after an open one publish as well, with the old pixels and the
    new stamp. That gave 2 frames per request, the first one stale.

Fix (`sim/graphs.py` `Graphs.set_image_publishing`, `sim/run_scene.py`
`render`):

*   Images are published only for a request. The gate is open only on
    the request's last render.
*   One more render follows with the gate closed, at the same sim time.
*   At start-up, one render creates the gates (they don't exist before the
    first render) and a second render with the gate closed absorbs the
    lagging publish.

Each request now yields exactly one frame, and 12 s of idle running yields
none. The synchronous-rendering settings were confirmed applied at run time,
so asynchronous rendering is not a factor.

**Step 5, `camera_request_renders`:** the io probe's `camera_fresh` alone,
10 trials each:

| N | passes | diff range |
|---|---|---|
| 1 | 6/10 | stale ones 0.08–0.31 |
| 2 | 10/10 | 0.91–0.99 |
| 3 | 10/10 | 0.98–1.03 |

N = 2 is set in `config/timing.yaml`.

### 2. `/sim/epoch`

The bridge's `ROS2Publisher` node creates its DDS writer on its second
evaluation and publishes from its third. This was measured on the epoch
graph: no writer on the ROS graph after the start-up evaluation, a writer
with no sample after one reset, and data only from the second reset on. The
epoch graph is evaluated only at start-up and on reset, so the start-up
epoch 0 was never published and the first reset only created the writer.
Every subscriber (Fast DDS and CycloneDDS, late or early) therefore got
nothing until the second reset. Graphs evaluated every cycle (state, wrench,
`/tf`) lose only their first two cycles, which no check sees.

Data-sharing plays no part. Once the writer had published, Fast DDS and
CycloneDDS late joiners got the latched value with the default
`fastdds.xml`. Why data-sharing OFF seemed to help in blocking issue 4 was
not investigated. `fastdds.xml` keeps the default.

Fix: `Graphs.__init__` evaluates the epoch graph
`PUBLISHER_WARMUP_EVALUATIONS` = 2 times. Its input is 0, so a warm-up
publish would also be correct. The start-up `publish_epoch(0)` is then the
first real publish. Result: the Fast DDS io probe, subscribed before the
reset, sees `epoch 0 -> 1`. `phase2_peer_hwm.py` (CycloneDDS, started after
Isaac) subscribes `/sim/epoch` with reliable, keep-last 1, transient-local
QoS and reports the latched value as `epoch_seen` (0 in every run).

### 3. `lockstep`

*   **0.29 rad tracking error: a discovery race at the probe's start.** The
    probe started τ at its first state pair. But Isaac's `/joint_command`
    reader may match the probe's writer later than the probe's readers
    match Isaac's writers. Until then Isaac free-runs and holds the arm at
    home while τ advances. After the 50 settle cycles, the error is then
    the sine itself. In one check run, joint 1 started following only after
    102 replies (0.408 s), and 0.3 · sin(π · 0.408) = 0.287 rad. That
    matches the reported 0.29.
*   **`missed 1`: the same race on the subscriber side.** It was seen
    twice in full checks and reproduced once with a fresh Isaac and a
    logging probe. A state stamped 5.410 s arrived before `/wrist_wrench`
    had matched, so it never paired and was pruned 1 s later. It
    happens only when the probe is the first peer of a fresh Isaac: 0
    misses in 20 runs against an Isaac that had already seen peers.
*   **Ruled out on the Isaac side:**
    *   A command applied at the wrong cycle after a render: the tracking
        error stays at 0.0015–0.0028 rad in all runs, including the
        periodic renders during lockstep.
    *   A held command: same evidence.
    *   A race at the resume after the pause: 100 pause/resume transitions
        (a probe pausing 1.5 s every 100 cycles) gave 0 misses.

Fix (`tests/e2e/phase2_peer_hwm.py`):

*   τ stays 0 until joint 1 follows the velocity feed-forward of the τ = 0
    command (|v1| > 0.05 rad/s).
*   Cycles, RTF, misses and tracking error count from that start. A state
    that arrives before it doesn't count as missed.
*   The JSON reports `start_pairs`, the replies sent before the start.

### Measurements (Addendum A)

*   **Render cost** (one `sim.render()`): 30–33 ms with the GPU to itself;
    160–340 ms while other GPU work ran. A request costs N + 1 = 3 renders.
*   **RTF, lockstep, no requests:** 0.62–0.63 with the GPU to itself, 0.39–0.40
    with it shared. **Free-run (standalone):** 0.44 and 0.31–0.32.
*   **RTF with one request per second:** not re-measured after these changes.

### Deviations from the plan

*   `camera_fresh` passes on a mean absolute difference > 0.6 instead of > 2
    (section 1).
*   Images are published only for `/camera/request`. The 5 s periodic
    renders and the extra renders publish no images. Decision 3 says images
    are also published without requests.
*   A request runs `camera_request_renders` + 1 renders, and start-up runs
    two renders.
*   The `lockstep` check also requires `epoch_seen == 0`.
*   The hwm probe starts τ, and counts cycles and misses, only once joint 1
    follows (section 3).

Files changed in Addendum A: `container/dds/fastdds.xml`,
`config/timing.yaml`, `sim/graphs.py`, `sim/run_scene.py`,
`tests/e2e/phase2_peer_hwm.py`, `tests/e2e/phase2_peer_io.py`,
`tests/e2e/phase2_check.py`, this file.
