# Phase 0.5: ROS 2 between Isaac (Fast DDS, Windows) and the container (WSL)

Part of [robot_platform.md](../robot_platform.md). Facts with citations:
[phase_0_5_inventory.md](phase_0_5_inventory.md) ("Inv §n").

## Goal and done criteria

`tests/e2e/phase0_5_check.py` runs the probes below and prints `PASS`/`FAIL`
per check. All must pass, except `service_cyclone`, whose result (either
way) is recorded because it decides which RMW the container's Python nodes
use.

| Name | Condition |
|---|---|
| `topic_cyclone` | Isaac (Fast DDS) publishes `/probe/ping` (`sensor_msgs/JointState`) at 250 Hz for 20 s; a container node on **CycloneDDS** echoes each message to `/probe/pong`; Isaac receives ≥ 99% of them back |
| `topic_fastdds` | the same with the container node on **Fast DDS** (Linux) |
| `latency` | from `topic_cyclone`: round-trip p50, p99, max printed; pass if p99 < 4 ms (one 250 Hz cycle) |
| `service_fastdds` | a container client on Fast DDS calls Isaac's `/probe/trigger` (`std_srvs/srv/Trigger`) 10 times; all succeed with `message == "isaac"` |
| `service_cyclone` | the same from a CycloneDDS client (recorded, not required) |
| `no_7400` | while all probes run, neither Windows (`netstat -ano -p udp`) nor WSL (`ss -uln`) has a socket on port 7400 or 7401 |
| `firewall` | `Get-NetFirewallHyperVVMSetting` for WSL reports `DefaultInboundAction: Block` (already observed, Inv §4.5; re-checked by the script) |

Then `docs/designs/phases/phase_0_5_notes.md` records the numbers and the
RMW decision below.

## Decisions made in this plan

*   **Discovery is unicast on loopback on both sides; multicast off.**
    Multicast does not cross Windows ↔ WSL in both directions, and the two
    share one UDP port space (Inv §4.4). Peers = `127.0.0.1`, participant
    indices auto-assigned, peer range 20 on both sides (Isaac uses several
    participants; the container will run about five processes).
*   **RMW of the container's Python nodes depends on `service_cyclone`.**
    `ros2_hwm` is CycloneDDS (D1) and uses topics only. If `service_cyclone`
    fails, the Python nodes (`scene_sync`, `robot_service`'s sim calls,
    `camera_sim`) use Fast DDS (`rmw_fastrtps_cpp`, the image's default),
    because they call Isaac's services; the design doc's "Every ROS 2 node
    in the container uses CycloneDDS" is changed accordingly. If it
    passes, all container nodes stay on CycloneDDS.
*   QoS for the probes: reliable, keep-last depth 1 (the design's control
    QoS).

## Constraints

*   No Bazel; no `git commit`; Docker via `sg docker -c`; no `pkill -f`.
*   Isaac's Python: `cat <script> | cmd.exe /c "cd /d C:\isaac-sim && python.bat -"`.
    Environment variables from WSL do not reach it; the script sets its
    own (Inv §1.5).
*   Windows-side files live in `C:\isaac-sim-work\` (`/mnt/c/isaac-sim-work/`
    from WSL); copy them there from the repository before running.

## Target files

### 1. `sim/dds/fastdds_isaac.xml` (Create)

Inv §3.3's profile with `maxInitialPeersRange` 20. Used by Isaac and by
the container's Fast DDS nodes (same file; the container mounts it).

### 2. `container/dds/cyclonedds.xml` (Create)

Inv §4.2's configuration, unchanged (`MaxAutoParticipantIndex` 20).

### 3. `sim/ros_probe.py` (Create; runs in Isaac's Python)

1.  Before anything else: `os.environ` `ROS_DISTRO=humble`,
    `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `PATH += ;C:\isaac-sim\exts\isaacsim.ros2.bridge\humble\lib`,
    `FASTRTPS_DEFAULT_PROFILES_FILE=C:\isaac-sim-work\fastdds_isaac.xml`,
    `ROS_DOMAIN_ID=0` (Inv §1.5, §3.1).
2.  `SimulationApp({"headless": True, "extra_args": ["--/app/vulkan=false"]})`,
    `enable_extension("isaacsim.ros2.bridge")`, `simulation_app.update()`,
    then `import rclpy` (Inv §1.4).
3.  Node `isaac_probe`: service `/probe/trigger` (`std_srvs/srv/Trigger`,
    returns `success=True, message="isaac"`); subscriber `/probe/pong`;
    publisher `/probe/ping`. QoS: reliable, keep-last 1.
4.  Wait up to 30 s for `/probe/pong`'s publisher count ≥ 1 and
    `/probe/ping`'s subscription count ≥ 1 (spinning). Then for 20 s at
    250 Hz: publish `JointState` with `position=[seq]` and
    `header.stamp` = `time.perf_counter_ns()` split into sec/nanosec;
    spin with `timeout_sec=0` between sends. On each pong, RTT =
    now − stamp. Keep spinning 30 s after the last ping so the container
    clients can finish their service calls.
5.  Write `C:\isaac-sim-work\probe_result.json`: `sent`, `received`,
    RTT list in ms. Exit.

The phase runs this script twice (once per container RMW).

### 4. `tests/e2e/ros_probe_peer.py` (Create; runs in the container)

`rclpy` node: subscribe `/probe/ping`, republish each message unchanged on
`/probe/pong` (same QoS); after the first ping arrives, call
`/probe/trigger` 10 times (5 s timeout each) and print
`SERVICE <ok_count>/10`; exit after 10 s without pings.

### 5. `container/Dockerfile` (Modify)

Add `ros-humble-rmw-cyclonedds-cpp` to the `apt-get install` line. Add
`COPY container/dds /opt/app/container/dds` and
`COPY sim/dds /opt/app/sim/dds`.

### 6. `tests/e2e/phase0_5_check.py` (Create; runs on WSL)

Orchestrates, for `rmw` in (`rmw_cyclonedds_cpp`, `rmw_fastrtps_cpp`):
1.  Copy `sim/ros_probe.py` and `sim/dds/fastdds_isaac.xml` to
    `/mnt/c/isaac-sim-work/`; delete any old `probe_result.json`.
2.  Start the peer:
    `sg docker -c "docker run --rm --name probe --network host -e RMW_IMPLEMENTATION=<rmw> -e CYCLONEDDS_URI=file:///opt/app/container/dds/cyclonedds.xml -e FASTRTPS_DEFAULT_PROFILES_FILE=/opt/app/sim/dds/fastdds_isaac.xml -v $PWD/tests/e2e:/probe --entrypoint bash intrinsic-core:dev -c 'source /opt/ros/humble/setup.bash && python3 /probe/ros_probe_peer.py'"`
    in the background, capturing its output.
3.  Start Isaac's probe (`cat /mnt/c/isaac-sim-work/ros_probe.py | cmd.exe /c "cd /d C:\isaac-sim && python.bat -"`),
    timeout 180 s.
4.  While both run (about 10 s into the ping phase): `no_7400` (Windows
    via `cmd.exe /c netstat -ano -p udp`, WSL via `ss -uln`).
5.  Read `probe_result.json` and the peer's `SERVICE n/10` line; evaluate
    the checks for this RMW.
Finally `firewall` (PowerShell, as in Inv §4.5). Print `PASS`/`FAIL` lines
and the latency numbers; exit 0 if all required checks pass.

## Steps

1.  Files 1–6; rebuild the image
    (`sg docker -c "docker build -f container/Dockerfile -t intrinsic-core:dev ."`).
2.  `.venv/bin/python tests/e2e/phase0_5_check.py`.
3.  If discovery fails (no pongs), try in order and record each result:
    (a) explicit peer ports in Cyclone (`127.0.0.1:7410` … `:7420`, even
    numbers) and in Fast DDS (`initialPeersList` locators with ports
    7410–7420); (b) DDS on the LAN address 192.168.0.161 instead of
    127.0.0.1 in both XML files. Do not change anything else; if none
    works, stop and record.
4.  Write `docs/designs/phases/phase_0_5_notes.md` (Create): the script
    output, the configuration that worked, the latency numbers, the
    `service_cyclone` result and the resulting RMW decision.
