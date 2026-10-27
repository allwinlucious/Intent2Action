# Phase 0: container skeleton

Part of [robot_platform.md](../robot_platform.md). Facts (with source
citations) are in [phase_0_inventory.md](phase_0_inventory.md); this plan
cites its sections as "Inv §n".

## Goal and done criteria

The Docker image `intrinsic-core:dev` starts these processes, and
`tests/e2e/phase0_smoke.py` passes against the running container:

| Process | Check in `phase0_smoke.py` |
|---|---|
| `zenohd` | TCP connect to 127.0.0.1:7447 succeeds |
| `zenoh_sessions` | Q6: `ss -Htnp state established '( dport = :7447 )'` (run on the host; host networking) shows at least one connection from each of the processes `world_binary`, `motion_planner_` and `generic_icon_ma` (`ss` truncates names to 15 characters). Zenoh rejects a protocol-version mismatch during the handshake, so an established session shows compatibility. zenohd in the image reports v1.7.2, the same as the linked zenoh-c (Inv §4) |
| localcas | gRPC channel to 127.0.0.1:18020 becomes ready within 5 s |
| platform_stubs | `ResourceRegistry.ListResourceInstances` on 127.0.0.1:18010 returns OK |
| geometry | gRPC channel to 127.0.0.1:18001 becomes ready within 5 s |
| world | `ObjectWorldService.ListObjects(world_id="world")` on 127.0.0.1:18000 returns an object named `root` |
| planner | `MotionPlannerService.ClearCache` on 127.0.0.1:18002 returns OK |
| `hwm` | `sg docker -c "docker exec intrinsic test -S /tmp/intrinsic_icon/robot_hwm.sock"` exits 0, and `docker logs intrinsic` contains `Hardware Module Runtime started.` |
| `icon_status` | `IconApiStub.GetStatus(GetStatusRequest())` on 127.0.0.1:18003 returns OK |
| `icon_parts` | `IconApiStub.ListParts(ListPartsRequest())` returns OK |

Check names (the `<name>` printed by the smoke test): `zenohd`,
`zenoh_sessions`, `localcas`, `stubs`, `geometry`, `world`, `planner`,
`hwm`, `icon_status`, `icon_parts`. Phase 0 is done only when all ten pass.

Plus: `container/README.md` lists every listening port and every address
flag of every process.

Not in this phase: Franka model, ICON parts (phase 1), ROS 2 (phase 0.5).

## Constraints

*   Do not modify anything under `/home/mew/intrinsic-core`.
*   Docker commands must run as `sg docker -c "<command>"` (the shell's
    group list predates the `docker` group membership).
*   Never use `pkill -f` or `killall`; stop processes by PID or with
    `docker stop`.
*   Do not run `container/build_intrinsic.sh` or any Bazel command: all
    six binaries already exist in `/home/mew/intrinsic-core/bazel-bin`.
*   Do not run `git commit`; the lead commits after review.
*   Python style for new files: `pyink --line-length=80 --indent-spaces=2`
    and `isort --profile=google` (install both into the venv with pip if
    missing). Do not reformat existing files.
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

Plain (not `Any`-wrapped) `intrinsic_proto.icon.HardwareModuleConfig` (Inv §3.2):

```textproto
# proto-file: intrinsic_apis/intrinsic/icon/hal/proto/hardware_module_config.proto
# proto-message: intrinsic_proto.icon.HardwareModuleConfig
name: "robot_hwm"
module_config {
  [type.googleapis.com/intrinsic_proto.icon.LoopbackConfig] { num_dof: 7 }  # Franka arm: 7 joints (Inv uses 6)
}
drives_realtime_clock: true
control_frequency_hz: 250
```

### 3. `container/entrypoint.sh` (Modify)

Keep the existing structure (`run` helper, `wait -n`, exit 1). Changes:

*   Add `export ALLOWED_PUBSUB_IPv4=127.0.0.1` before starting anything, so
    Zenoh peer listeners bind loopback instead of `0.0.0.0` (Inv §2 notes).
*   `world`: add `--conductor_port=18009` (otherwise it takes 8082, Inv §2)
    and `--worlds_path=/var/lib/intrinsic/worlds` (create the directory
    with `mkdir -p` first).
*   After the planner line, start:
    *   `run hwm "$BIN/loopback_hardware_module_main" --module_config_file="$CFG/loopback_hwm.txtpb" --logger_connection_timeout=0s`
    *   `run icon "$BIN/generic_icon_machine_server" --config_pbtxt_file="$CFG/icon.txtpb" --non_realtime --icon_server_port=18003 --use_runtime_asset_fallback=false --server_name=robot --zenoh_router="$ZENOH" --resource_registry_service=localhost:18010`
    (ICON command line from Inv §3.2.)

### 4. `container/run.sh` (Create)

Starts the container. Exact command:

```bash
#!/usr/bin/env bash
# Runs the Intrinsic container on the host network (see container/README.md for ports).
set -euo pipefail
sg docker -c "docker rm -f intrinsic" >/dev/null 2>&1 || true
exec sg docker -c "docker run --name intrinsic --network host \
  --cap-add IPC_LOCK --cap-add SYS_NICE --ulimit memlock=-1:-1 \
  -v intrinsic-data:/var/lib/intrinsic intrinsic-core:dev"
```

`IPC_LOCK` and unlimited memlock: ICON and the HWM call `mlockall` and
exit if it fails (Inv §5). `SYS_NICE`: ICON's manifest requests it for sim
too (Inv §5); kept so the process matches its supported configuration. No
`--rm`: the stopped container keeps its logs for `docker logs`; `run.sh`
removes the previous one first.

### 5. `container/Dockerfile` (No change)

It already copies `container/config` (including the new files) and
`/zenohd` from the zenohd image (verified: the image's entrypoint is
`/dumb-init -- /zenohd`; it needs glibc ≤ 2.34, and `ros:humble` has
2.35).

### 6. `container/README.md` (Create)

Sections: "Build" (`container/build_intrinsic.sh`,
`container/stage_intrinsic.sh`,
`sg docker -c "docker build -f container/Dockerfile -t intrinsic-core:dev ."`),
"Run" (`container/run.sh`), "Ports" (table below), "Address flags" (table
below), "Smoke test" (`.venv/bin/python tests/e2e/phase0_smoke.py`).

Ports table (from Inv §2 with this phase's flags):

| Process | Port | Bind |
|---|---|---|
| zenohd | 7447 | 127.0.0.1 |
| Zenoh peers in world, planner, ICON | random | 127.0.0.1 (`ALLOWED_PUBSUB_IPv4`) |
| world | 18000, conductor 18009 | all (`[::]`) |
| geometry | 18001 | all (`[::]`) |
| planner | 18002 | all (`[::]`) |
| ICON | 18003 | `0.0.0.0` |
| platform_stubs | 18010 | all |
| localcas | 18020 | all |
| HWM | none (Unix socket `/tmp/intrinsic_icon/robot_hwm.sock`) | — |
| metrics exporters | off (no `--opencensus_metrics_port`) | — |

Add one sentence: exposure beyond the machine is controlled by WSL's
Hyper-V firewall (checked in phase 0.5).

Address flags table (every address flag from Inv §1; "off" = left empty,
feature disabled):

| Binary | Flag | Value | Target |
|---|---|---|---|
| world | `--resource_registry_address` | `localhost:18010` | stand-in |
| world | `--cas_service_address` | `localhost:18020` | localcas |
| world | `--zenoh_router` | `tcp/127.0.0.1:7447` | zenohd |
| world | `--data_logger_grpc_service_address` | empty | off |
| world | `--hss_service_address`, `--sim_service_address`, `--workcell_cluster_service`, `--asset_deployment_service`, `--asset_instances_service`, `--runtime_db_service_address` | empty | off (conductor features) |
| world | `--logger_address` | empty | unused by the binary |
| geometry | `--cas_service_address` | `localhost:18020` | localcas |
| planner | `--world_service_address` | `localhost:18000` | world |
| planner | `--geometry_service_address` | `localhost:18001` | geometry |
| planner | `--cas_grpc_service_address` | empty | off (non-volatile cache) |
| planner | `--zenoh_router` | `tcp/127.0.0.1:7447` | zenohd |
| planner | runtime context `InstalledAssetsReader` | `localhost:18010` | stand-in |
| ICON | `--resource_registry_service` | `localhost:18010` | stand-in |
| ICON | `--zenoh_router` | `tcp/127.0.0.1:7447` | zenohd |
| HWM | — | — | none (Unix socket only) |

### 7. `container/gen_protos.sh` (Run, no change)

Generate ICON's API stubs: `container/gen_protos.sh intrinsic/icon/proto/v1/service.proto`
(the script resolves include roots; the proto lives under
`intrinsic_apis/`). Leave the generated files in `gen/` (the lead
commits them).

### 8. `tests/e2e/phase0_smoke.py` (Create; new directory `tests/e2e/`)

Python, uses stubs from `gen/` (add `gen/` to `sys.path` the way
`container/services/platform_stubs.py` does). One function per check name
above, each with a 5 s deadline; print `PASS <name>` or
`FAIL <name>: <error>`; exit code 0 only if all pass; finally print
`elapsed <s>` = seconds from the script's start to its end. Stubs and
requests: `ResourceRegistryStub.ListResourceInstances(ListResourceInstanceRequest())`
(singular `Request`), `ObjectWorldServiceStub.ListObjects(ListObjectsRequest(world_id="world"))`
then match any `response.objects[i].name == "root"`,
`MotionPlannerServiceStub.ClearCache(empty_pb2.Empty())`,
`IconApiStub.GetStatus(GetStatusRequest())`,
`IconApiStub.ListParts(ListPartsRequest())`. Before the checks, wait up
to 180 s for 127.0.0.1:18003 to accept TCP connections (ICON opens its
port after main-loop creation; with no services configured the bound is
the 120 s HWM connection wait, Inv §3.3).

## Steps

1.  Create files 1, 2, 4, 6; modify 3 and 5 as above.
2.  Run step 7 (proto generation).
3.  `container/stage_intrinsic.sh` (must exit 0; it fails on "unresolved
    libraries").
4.  `sg docker -c "docker build -f container/Dockerfile -t intrinsic-core:dev ."`
    from `/home/mew/untitled-project`.
5.  `mkdir -p logs && container/run.sh > logs/phase0_run.log 2>&1 &`,
    then immediately run `.venv/bin/python tests/e2e/phase0_smoke.py`
    (its own ICON wait is the only wait).
6.  If a check fails, read `sg docker -c "docker logs intrinsic"`, fix
    the cause within the target files of this plan, rebuild, and repeat.
    If the fix would need a file outside this plan or a change under
    `/home/mew/intrinsic-core` (for example, ICON rejecting a config with
    zero parts), stop and write the exact error into
    `docs/designs/phases/phase_0_notes.md` instead.
7.  Stop the container with `sg docker -c "docker stop intrinsic"`.
8.  Write `docs/designs/phases/phase_0_notes.md` (Create): the smoke
    test output, image size (`docker images`), the `elapsed` value, and
    any deviation from this plan.

## Tests

*   `tests/e2e/phase0_smoke.py` is the test (above).
*   Nothing else is unit-testable in this phase (configuration only).
