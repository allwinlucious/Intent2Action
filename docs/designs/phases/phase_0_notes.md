# Phase 0 implementation notes

Status: **8 of 10 checks pass; `icon_status` and `icon_parts` carried to phase 1** (accepted by the lead: they need the robot in the world).

## Smoke test (2026-10-27)

```
PASS zenohd
PASS zenoh_sessions
PASS localcas
PASS stubs
PASS geometry
PASS world
PASS planner
PASS hwm
FAIL icon_status: FAILED_PRECONDITION: Please configure the following required, but currently unused interfaces: robot_hwm: [joint_position_command]
FAIL icon_parts: FAILED_PRECONDITION: Please configure the following required, but currently unused interfaces: robot_hwm: [joint_position_command]
elapsed 14.1
```

*   Image `intrinsic-core:dev`: 2.11 GB; `container/stage/`: 644 MB.
*   All services are up 14 s after `docker run`.
*   **Q6 answered:** world, planner and ICON hold established Zenoh
    sessions with zenohd (both v1.7.2).
*   **ICON:** the gRPC server runs and the HWM connection works: ICON
    reads the loopback module's interfaces and refuses to build its main
    loop because no part claims `joint_position_command`. A
    position-commanded arm part needs the robot's kinematic object in the
    world (Inv §3.5), which phase 1 creates. Phase 1's done criteria
    include `icon_status` and `icon_parts` passing.

## Deviations from the plan

*   `container/stage_intrinsic.sh` exited 1 on success (the `grep` in the
    final loop returned 1 under `pipefail`). Fixed with `|| true` on that
    `grep` (found by the worker, applied by the lead).
*   `container/run.sh` was created without the executable bit; fixed.
*   The `zenoh_sessions` check runs `ss` inside the container
    (`docker exec`), because a host-side `ss -p` as a normal user cannot
    name the container's root processes. `iproute2` added to the image
    for this.
*   `pyink` 26.5.1 has no `--indent-spaces`; `--pyink-indentation=2` was
    used.
*   `container/stage/lib` is empty: the only Bazel-built libraries
    (`librosidl_typesupport_introspection_{c,cpp}_identifier.so`) resolve
    through each binary's RUNPATH into the staged runfiles.

## Carried into phase 1 (from the code review)

*   ICON builds its main loop once at start-up and captures kinematics
    then. After the loader creates the Franka in `world`, phase 1 must
    call `IconApi.ClearFaults` (which rebuilds the main loop) or start
    ICON after the robot exists.
*   `--worlds_path` is in the persistent volume, so worlds survive
    container restarts (needed later for saved calibration). Phase 1's
    loader must be idempotent (skip or update an existing robot).
