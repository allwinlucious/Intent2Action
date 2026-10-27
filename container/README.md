# Intrinsic container

One Docker image (`intrinsic-core:dev`) that runs the Intrinsic Core
binaries as plain processes, started by `container/entrypoint.sh`.

## Build

```bash
container/build_intrinsic.sh
container/stage_intrinsic.sh
sg docker -c "docker build -f container/Dockerfile -t intrinsic-core:dev ."
```

## Run

```bash
container/run.sh
```

## Franka loader

`robot_model/` builds the SceneObjects `franka` (arm) and `franka_hand`
from `assets/robots/franka/lula_franka_gen.urdf` and uploads the meshes
with `GeometryService.CreateGeometry`. `entrypoint.sh` runs
`python3 -m robot_model.load_franka` in the foreground after the planner
starts and before the HWM and ICON, so ICON's main loop sees the robot.
Loading is delete-then-create, so restarts with the persisted world volume
work. If the loader fails, the container exits.

## Ports

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

Exposure beyond the machine is controlled by WSL's Hyper-V firewall
(checked in phase 0.5).

## Address flags

"off" = left empty, feature disabled.

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

## Smoke test

```bash
.venv/bin/python tests/e2e/phase0_smoke.py
```
