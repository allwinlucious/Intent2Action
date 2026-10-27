# Phase 0.5: ROS 2 between Isaac (Fast DDS, Windows) and the container (WSL)

Part of [robot_platform.md](../robot_platform.md). Facts, with source
citations, are in [phase_0_5_inventory.md](phase_0_5_inventory.md)
("Inv §n"). Terms: RMW = the ROS 2 middleware layer that picks the DDS
vendor (`rmw_cyclonedds_cpp` or `rmw_fastrtps_cpp`); D1 = the design's
owner decision that `ros2_hwm` uses Intrinsic's Bazel ROS 2 (CycloneDDS).

Preconditions (true now): WSL mirrored networking active; `container/stage/`
populated (phase 0); image `intrinsic-core:dev` exists.

## Goal and done criteria

`tests/e2e/phase0_5_check.py` runs the probe twice (container side on
CycloneDDS, then on Fast DDS) and prints `PASS`/`FAIL` per check:

| Name | Required | Condition |
|---|---|---|
| `topic_cyclone` | yes | Isaac (Fast DDS) sends `/probe/ping` (`sensor_msgs/JointState`) at 250 Hz for 20 s; the container peer on CycloneDDS echoes each to `/probe/pong`; Isaac gets ≥ 99% back |
| `topic_fastdds` | yes | the same with the peer on Fast DDS (Linux) |
| `service_fastdds` | yes | the Fast DDS peer calls Isaac's `/probe/trigger` (`std_srvs/srv/Trigger`) 10 times; ok = `success == True and message == "isaac"`; all 10 ok |
| `service_cyclone` | no (recorded) | the same from the CycloneDDS peer |
| `no_7400` | yes | 10 s after the peer prints `FIRST_PING`: no UDP socket on port 7400 or 7401 on Windows (`netstat -ano -p udp`) or WSL (`ss -uln`) |
| `firewall` | yes | `Get-NetFirewallHyperVVMSetting` for WSL: `DefaultInboundAction : Block` |
| `latency` | no (recorded) | round-trip p50 / p99 / max (ms) for each run; input to phase 2's control-rate choice (design: "Timing") |

Only Isaac-side servers exist in the design, so services are tested in that
direction only. The probe uses the script's own `rclpy`, not the OmniGraph
bridge nodes (same DDS libraries; the bridge path is tested in phase 2).

## Decisions made in this plan

*   **DDS per process** is now fixed in the design doc ("Process model",
    "DDS per process"): Isaac-facing service calls and camera topics on
    Fast DDS (same vendor as Isaac), `ros2_hwm` on CycloneDDS (topics only).
    This phase proves the two required paths (`topic_cyclone`,
    `service_fastdds` / `topic_fastdds`). `service_cyclone` is recorded
    to confirm or refute the incompatibility (Inv §5.3).
*   **Discovery:** unicast to 127.0.0.1 on both sides, multicast off.
    Windows → WSL multicast works only on the LAN interface, WSL → Windows
    multicast fails on every interface, and loopback multicast fails both
    ways; Windows and WSL also share one UDP port space (Inv §4.4).
*   **Peer range:** participant IDs 0–19 on both sides: Fast DDS
    `maxInitialPeersRange` 20, Cyclone `MaxAutoParticipantIndex` 19.
    Expected participants: Isaac 2 (bridge + script `rclpy`), container 5
    (`ros2_hwm`, `scene_sync`, `camera_sim`, `robot_service`, grasp
    planner), plus headroom for probes and tools.
*   **QoS** for ping/pong: reliable, keep-last depth 1, volatile, the
    design's control-loop QoS (a dropped command would stall lockstep).

## Constraints

*   No Bazel; no `git commit`; Docker via `sg docker -c "…"`; no `pkill -f`
    or `killall` (stop the probe container with `docker rm -f probe`).
*   Isaac's Python: pipe the script on stdin:
    `cmd.exe /c "cd /d C:\isaac-sim && python.bat -" < <repo>/sim/ros_probe.py`.
    WSL environment variables do not reach it (Inv §1.5).
*   Windows-side files: only `fastdds.xml`, copied by the orchestrator to
    `C:\isaac-sim-work\fastdds.xml` (`/mnt/c/isaac-sim-work/fastdds.xml`).
*   `<repo>` = `/home/mew/untitled-project` (absolute, never `$PWD`).

## Target files

### 1. `container/dds/fastdds.xml` (Create)

Inv §3.3's profile with `maxInitialPeersRange` 20. Used by Isaac (copied to
Windows) and by the container's Fast DDS nodes (copied into the image).

### 2. `container/dds/cyclonedds.xml` (Create)

Inv §4.2's configuration with `MaxAutoParticipantIndex` 19.

### 3. `container/Dockerfile` (Modify)

Add `ros-humble-rmw-cyclonedds-cpp` to the `apt-get install` line. The
existing `COPY container/config …` pattern: add
`COPY container/dds /opt/app/container/dds`.

### 4. `sim/ros_probe.py` (Create; Isaac's Python 3.11)

1.  Before any other import of Isaac modules:
    ```python
    os.environ["ROS_DISTRO"] = "humble"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ["PATH"] += r";C:\isaac-sim\exts\isaacsim.ros2.bridge\humble\lib"
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = r"C:\isaac-sim-work\fastdds.xml"
    os.environ["ROS_DOMAIN_ID"] = "0"
    ```
2.  `SimulationApp({"headless": True, "extra_args": ["--/app/vulkan=false"]})`;
    `from isaacsim.core.utils.extensions import enable_extension`;
    `enable_extension("isaacsim.ros2.bridge")`; `simulation_app.update()`;
    `import rclpy`; `rclpy.init()` (Inv §1.4, `clock.py`).
3.  Node `isaac_probe` with: service `/probe/trigger` (returns
    `success=True, message="isaac"`); publisher `/probe/ping` and
    subscriber `/probe/pong`, both QoS reliable / keep-last 1 / volatile.
4.  Wait (spinning with `spin_once(timeout_sec=0.05)`) until
    `/probe/ping` has ≥ 1 subscriber and `/probe/pong` has ≥ 1 publisher,
    at most 120 s. On timeout: write the result file with `sent=0`, close,
    exit.
5.  Print `PING_START` (flush). Send loop, absolute schedule: for
    `k in range(5000)` (20 s at 250 Hz): `t_k = t0 + k * 4 ms`
    (`time.perf_counter_ns()`); publish `JointState(position=[float(k)])`
    with `header.stamp` = send time split into sec/nanosec; then
    `while perf_counter_ns() < t_{k+1}: spin_once(timeout_sec=remaining)`.
    The pong callback records `perf_counter_ns() - stamp` (RTT) keyed by
    `k`.
6.  After the last ping, keep spinning for 30 s (late pongs, service
    calls).
7.  Write `C:\isaac-sim-work\probe_result.json`:
    `{"sent": n, "received": m, "rtt_ms": [...]}`; print `PROBE_DONE`;
    `simulation_app.close()`.

### 5. `tests/e2e/ros_probe_peer.py` (Create; container's Python 3.10)

Single-threaded `rclpy` executor; nothing blocks inside callbacks:
*   Subscriber `/probe/ping` → republish the same message on `/probe/pong`
    (same QoS as file 4). On the first ping print `FIRST_PING` (flush).
*   Service client `/probe/trigger`: after the first ping, run calls one at
    a time with `call_async`; start the next when the previous future is
    done or 5 s have passed (that call counts as failed). ok =
    `result.success and result.message == "isaac"`. After 10 calls print
    `SERVICE <ok>/10`.
*   Exit: wait up to 180 s for the first ping (else print
    `NO_PING` and exit 1); after that, exit 10 s after the last ping.

### 6. `tests/e2e/phase0_5_check.py` (Create; WSL, `.venv` Python)

For each `rmw` in `rmw_cyclonedds_cpp`, `rmw_fastrtps_cpp`:
1.  Copy `container/dds/fastdds.xml` to `/mnt/c/isaac-sim-work/`; delete
    `probe_result.json` there; `sg docker -c "docker rm -f probe"`.
2.  Start the peer with `subprocess.Popen` (stdout piped, read by a
    thread):
    `sg docker -c "docker run --rm --name probe --network host -e RMW_IMPLEMENTATION=<rmw> -e CYCLONEDDS_URI=file:///opt/app/container/dds/cyclonedds.xml -e FASTRTPS_DEFAULT_PROFILES_FILE=/opt/app/container/dds/fastdds.xml -v /home/mew/untitled-project/tests/e2e:/probe:ro --entrypoint bash intrinsic-core:dev -c 'source /opt/ros/humble/setup.bash && python3 /probe/ros_probe_peer.py'"`.
3.  Start Isaac's probe with `subprocess.Popen` (stdin = the script file),
    stdout read by a thread.
4.  When the peer prints `FIRST_PING`: wait 10 s, then run `no_7400`. If
    no `FIRST_PING` within 180 s: mark the run's topic and service checks
    `FAIL` (cause: no discovery).
5.  Wait for Isaac to exit (timeout 300 s; on timeout terminate it) and
    for the peer to exit (timeout 60 s; on timeout `docker rm -f probe`).
6.  Evaluate from `probe_result.json` and the peer's `SERVICE n/10` line
    (missing line = 0/10). Latency: p50, p99 and max of `rtt_ms`.

Then `firewall` via `powershell.exe -NoProfile -Command
"Get-NetFirewallHyperVVMSetting -PolicyStore ActiveStore -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}'"`.
Print all lines; exit 0 if every required check passes.

## Steps

1.  Files 1–6. Rebuild:
    `sg docker -c "docker build -f container/Dockerfile -t intrinsic-core:dev ."`.
2.  `.venv/bin/python tests/e2e/phase0_5_check.py`.
3.  If a required topic check fails in either run, apply fallback (a),
    rerun both runs; if still failing, apply (b) on top, rerun; record
    every result.
    *   (a) Explicit ports, indices 0–19 (even ports 7410–7448).
        `cyclonedds.xml`: replace `<Peer Address="127.0.0.1"/>` with
        twenty `<Peer Address="127.0.0.1:74NN"/>` for NN = 10, 12, …, 48.
        `fastdds.xml`: replace the single `initialPeersList` locator with
        twenty `<locator><udpv4><address>127.0.0.1</address><port>74NN</port></udpv4></locator>`
        for the same ports.
    *   (b) LAN address instead of loopback: let `L` = the IPv4 address of
        the interface carrying the default route (`ip -4 route get 1.1.1.1`
        → `src`). Replace every `127.0.0.1` in both files with `L`
        (`interfaceWhiteList`, `metatrafficUnicastLocatorList`,
        `initialPeersList` in Fast DDS; `NetworkInterface@address` and
        `Peer@Address` in Cyclone).
    If (b) also fails: stop and record.
4.  Write `docs/designs/phases/phase_0_5_notes.md` (Create): full script
    output, the configuration that worked, latency numbers per run, the
    `service_cyclone` result.
