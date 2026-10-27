# Phase 0: container skeleton

Part of [robot_platform.md](../robot_platform.md). Facts (with source
citations) are in [phase_0_inventory.md](phase_0_inventory.md); this plan
cites its sections as "Inv §n".

## Goal and done criteria

The Docker image `intrinsic-core:dev` starts these processes, and
`tests/e2e/phase0_smoke.py` passes against the running container:

| Process | Check in `phase0_smoke.py` |
|---|---|
| zenohd | TCP connect to 127.0.0.1:7447 succeeds |
| localcas | gRPC channel to 127.0.0.1:18020 becomes ready within 5 s |
| platform_stubs | `ResourceRegistry.ListResourceInstances` on 127.0.0.1:18010 returns OK |
| geometry | gRPC channel to 127.0.0.1:18001 becomes ready within 5 s |
| world | `ObjectWorldService.ListObjects(world_id="world")` on 127.0.0.1:18000 returns an object named `root` |
| planner | `MotionPlannerService.ClearCache` on 127.0.0.1:18002 returns OK |
| loopback HWM | process alive (checked through ICON below) |
| ICON | `IconApi.GetStatus` on 127.0.0.1:18003 returns OK (any content), and `IconApi.ListParts` returns OK |

Plus: `container/README.md` lists every listening port of every process.

Not in this phase: Franka model, ICON parts (phase 1), ROS 2 (phase 0.5).

## Constraints

*   Do not modify anything under `/home/mew/intrinsic-core`.
*   Docker commands must run as `sg docker -c "<command>"` (the shell's
    group list predates the `docker` group membership).
*   Never use `pkill -f` or `killall`; stop processes by PID or with
    `docker stop`.
*   Bazel builds are slow (minutes to hours); every needed binary is
    already built in `/home/mew/intrinsic-core/bazel-bin`. Do not run
    `bazel clean`.
*   Python for project scripts uses `/home/mew/untitled-project/.venv/bin/python`.

## Target files

### 1. `container/config/icon.txtpb` (Create)

Plain `intrinsic_proto.icon.IconMainConfig` (not `Any`-wrapped; Inv §3.1).
Start-up only, no parts (Inv §3.2, first ICON block):

```textproto
# proto-file: intrinsic_apis/intrinsic/icon/server/config/icon_main_config.proto
# proto-message: intrinsic_proto.icon.IconMainConfig
control_frequency_hz: 250
hardware_module_read_write_timeout_seconds: 10
hardware_module_names: "robot_hwm"
hardware_module_that_drives_clock: "robot_hwm"
```

### 2. `container/config/loopback_hwm.txtpb` (Create)

Plain `intrinsic_proto.icon.HardwareModuleConfig` (Inv §3.2):

```textproto
# proto-file: intrinsic_apis/intrinsic/icon/hal/proto/hardware_module_config.proto
# proto-message: intrinsic_proto.icon.HardwareModuleConfig
name: "robot_hwm"
module_config {
  [type.googleapis.com/intrinsic_proto.icon.LoopbackConfig] { num_dof: 7 }
}
drives_realtime_clock: true
control_frequency_hz: 250
```

### 3. `container/entrypoint.sh` (Modify)

Keep the existing structure (`run` helper, `wait -n`, exit 1). Changes:

*   Add `export ALLOWED_PUBSUB_IPv4=127.0.0.1` before starting anything, so
    Zenoh peer listeners bind loopback instead of `0.0.0.0` (Inv §2 notes).
*   `localcas`: unchanged flags (port 18020, dirs under
    `/var/lib/intrinsic/`).
*   `world`: add `--conductor_port=18009` (otherwise it takes 8082, Inv §2)
    and `--worlds_path=/var/lib/intrinsic/worlds` (create the directory
    with `mkdir -p` first).
*   After the planner line, start:
    *   `run hwm "$BIN/loopback_hardware_module_main" --module_config_file="$CFG/loopback_hwm.txtpb" --logger_connection_timeout=0s`
    *   `run icon "$BIN/generic_icon_machine_server" --config_pbtxt_file="$CFG/icon.txtpb" --non_realtime --icon_server_port=18003 --use_runtime_asset_fallback=false --server_name=robot --zenoh_router="$ZENOH"`
    (ICON command line from Inv §3.2.)

### 4. `container/run.sh` (Create)

Starts the container. Exact command:

```bash
#!/usr/bin/env bash
# Runs the Intrinsic container on the host network (see container/README.md for ports).
set -euo pipefail
exec sg docker -c "docker run --rm --name intrinsic --network host \
  --cap-add IPC_LOCK --cap-add SYS_NICE --ulimit memlock=-1:-1 \
  -v intrinsic-data:/var/lib/intrinsic intrinsic-core:dev"
```

`IPC_LOCK` and unlimited memlock: ICON and the HWM call `mlockall` and
exit if it fails (Inv §5). `SYS_NICE`: requested by ICON's own manifest
(Inv §5).

### 5. `container/Dockerfile` (Modify)

No functional change needed beyond what exists, except: the `COPY
container/config` line already copies the two new config files. Verify
`zenohd` path inside the image: the existing `COPY --from=zenoh /zenohd`
assumes the binary is at `/zenohd` in the zenohd image; if `docker build`
fails there, find the path with
`sg docker -c "docker create --name z <image>" && sg docker -c "docker export z" | tar -t | grep -i zenohd`
then `docker rm z`, and correct the `COPY` source path.

### 6. `container/README.md` (Create)

Sections: "Build" (the three commands from the design doc's Verification
Plan), "Run" (`container/run.sh`), "Ports" (table below), "Smoke test"
(`.venv/bin/python tests/e2e/phase0_smoke.py`).

Ports table (from Inv §2 with this phase's flags):

| Process | Port | Bind |
|---|---|---|
| zenohd | 7447 | 127.0.0.1 |
| Zenoh peers in world, planner, ICON, HWM | random | 127.0.0.1 (`ALLOWED_PUBSUB_IPv4`) |
| world | 18000, conductor 18009 | all |
| geometry | 18001 | all |
| planner | 18002 | all |
| ICON | 18003 | all |
| platform_stubs | 18010 | all |
| localcas | 18020 | all |
| HWM | none (Unix socket `/tmp/intrinsic_icon/robot_hwm.sock`) | — |
| metrics exporters | off (no `--opencensus_metrics_port`) | — |

Add one sentence: exposure beyond the machine is controlled by WSL's
Hyper-V firewall (checked in phase 0.5).

### 7. `container/gen_protos.sh` (Run, no change)

Generate ICON's API stubs: `container/gen_protos.sh intrinsic/icon/proto/v1/service.proto`
(the script resolves include roots; the proto lives under
`intrinsic_apis/`). Commit the generated files under `gen/`.

### 8. `tests/e2e/phase0_smoke.py` (Create)

Python, uses stubs from `gen/` (add `gen/` to `sys.path` the way
`container/services/platform_stubs.py` does). One function per row of the
done-criteria table, each with a 5 s deadline; print `PASS <name>` or
`FAIL <name>: <error>` per check; exit code 0 only if all pass. Before the
checks, wait up to 240 s for 127.0.0.1:18003 to accept TCP connections
(ICON opens its port only after main-loop creation, up to ~300 s; Inv
§3.4).

## Steps

1.  Create files 1, 2, 4, 6; modify 3 and 5 as above.
2.  Run step 7 (proto generation).
3.  `container/stage_intrinsic.sh` (must end without "unresolved
    libraries").
4.  `sg docker -c "docker build -f container/Dockerfile -t intrinsic-core:dev ."`
    from `/home/mew/untitled-project`.
5.  Start `container/run.sh` in the background, wait, run
    `phase0_smoke.py`. Logs: `sg docker -c "docker logs intrinsic"`.
6.  If ICON fails main-loop creation with zero parts (Inv §5 "Not checked":
    whether `MainLoop::Create` accepts no parts), record the exact error
    in `docs/designs/phases/phase_0_notes.md` and leave ICON's check
    failing: phase 1 adds the arm part. Every other check must pass.
7.  Stop the container with `sg docker -c "docker stop intrinsic"`.
8.  Write `docs/designs/phases/phase_0_notes.md`: what passed, image size,
    start-up time until all checks pass, any deviation from this plan.

## Tests

*   `tests/e2e/phase0_smoke.py` is the test (above).
*   Nothing else is unit-testable in this phase (configuration only).
