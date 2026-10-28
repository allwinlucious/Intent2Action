# Phase 6 implementation notes

Status: **done.** All four done criteria pass: `tools`, `calls`, `cuj1`
5/5, `unit`. The `phase3_cuj_check.py` rerun passes 8/8 on its second run.
No container or Isaac process is left running. No image rebuild was needed:
the container does not use `gen/robot_service.desc`.

| Check | Result |
|---|---|
| `tools` (`tests/e2e/phase6_check.py`) | PASS. 30 tools, each with a description and an input schema |
| `calls` (same) | PASS. `get_state` 7 joints; `move_to_pose` done 0.49 s; `get_image` PNG 1280×720; unreachable → error `NOT_FOUND: operations/MoveToPose-3: planning failed: IK solver couldn't find any …`; `wait=false` move → `wait_operation` done; 8 calls, 8 log lines |
| `cuj1` (`tests/e2e/phase6_cuj1.py`) | PASS, **5/5** (below) |
| `unit` | PASS. `pytest tests/test_mcp_server.py` 8 passed; `pytest tests/` 89 passed |
| `phase3_cuj_check.py` | First run 7/8: `obstacle` failed with "1/20 path samples in collision" at [-0.317, -0.093, -0.063, -2.207, 0.175, 2.335, 0.673]. This is the known flake at the same spot ([phase_4_notes.md](phase_4_notes.md), "Open: obstacle check flakiness"). The second run passed 8/8 |

The outputs are in `logs/phase6_check.out`, `logs/phase6_cuj1.out`,
`logs/phase6_phase3_cuj_check.out` and `…_check2.out`. The transcripts are
`logs/cuj1_trial_<n>.jsonl`.

## CUJ 1 trials

Each trial used `a frontier hosted model` with only the `robot` MCP server and no
built-in tools.

| Trial | Time | Tool calls | Cube from the plate centre | Cube z − plate z | Cost |
|---|---|---|---|---|---|
| 1 | 73 s | 15 | 0.6 mm | 12.0 mm | $0.21 |
| 2 | 71 s | 15 | 0.6 mm | 12.0 mm | $0.11 |
| 3 | 64 s | 15 | 0.8 mm | 12.0 mm | $0.11 |
| 4 | 63 s | 15 | 0.7 mm | 12.0 mm | $0.10 |
| 5 | 72 s | 16 | 0.7 mm | 12.0 mm | $0.11 |

Every transcript tool call has a line in `logs/mcp_server.log`. The
sequence was the same in all five trials, apart from where `get_state`
came:

`get_state`, `list_objects`, `plan_grasp`, `move_to_pose` (pregrasp),
`move_to_pose` (grasp), `grasp`, `attach_object`, `move_to_pose` (lift),
`move_to_pose` (above the plate), `move_to_contact` (rejected),
`move_to_contact`, `release`, `detach_object`, `move_to_pose` (up),
`list_objects`.

Trial 5 also tried `detect_objects` at the end to verify, got
`UNIMPLEMENTED`, and fell back to `list_objects`.

Observations for the lead. Nothing here blocks G10.

- **`move_to_contact` speed range.** the frontier model lowered the cube onto the
  plate with `move_to_contact` instead of a computed `move_to_pose`. Its
  first call was rejected every time with `INVALID_ARGUMENT: speed must
  be between 0.0007 and 0.0101 m/s for 5 N, got 0.02` (0.02 or 0.03).
  It recovered from the message on the next call. Stating the speed limit
  in the `MoveToContactRequest.speed` comment would save that call.
- **No `get_image` call.** the frontier model never called `get_image`. CUJ 1's
  "Then" lists `GetImage` and `DetectObjects`, but `list_objects` gives
  ground-truth poses, so no image is needed. The done criteria do not
  require an image.

## Implementation

- **Decision 1 (process).** `mcp_server/server.py` uses the low-level
  `Server(on_list_tools=…, on_call_tool=…)` of mcp 2.3.0 over stdio. gRPC
  calls go through the generated `RobotServiceStub` inside
  `anyio.to_thread.run_sync`. `gen/` is put on `sys.path` relative to the
  file.
- **Decision 2 (generated tools).** `gen_protos.sh` writes
  `gen/robot_service.desc` when `robot_service/robot_service.proto` is
  among its arguments. Rerunning it left the generated `.py` files
  byte-identical.
  - The server loads the descriptor set into its own `DescriptorPool`,
    used for walking and for comments; comments are looked up from
    `source_code_info` by path. Requests and responses use the generated
    classes (the default pool).
  - Descriptions: the RPC comment, then `` `field` (type): `` with the
    field's comments, enum values with their comments (`*_UNSPECIFIED`
    left out, as in the schema), and the oneof members.
  - Schemas also carry field comments and nested message comments (for
    example Pose's "Position (m) and orientation (unit quaternion)") as
    property `description`s.
  - `wait` gets `"default": true` in the schema, and the server sets it
    when it is absent.
- **Decision 3 (tool set).** 30 tools: every `RobotService` RPC except
  `GetPointCloud`.
- **Decision 4 (results).** As specified. Operation `error.code` is given
  by its name (for example `ABORTED`).
  - An `UNAVAILABLE` whose message contains "failed to connect" (gRPC's
    own connection error) becomes `UNAVAILABLE: robot service not
    reachable at 127.0.0.1:18100`. robot_service's own `UNAVAILABLE`
    messages are kept.
  - In WSL, with no service listening, that error takes about 20 s (the
    connect attempt does not fail fast).
- **Decision 5 (instructions).** `INSTRUCTIONS` in `server.py`, repeated
  in `mcp_server/README.md`. Besides the specified content it says that
  object poses are bottom centres (`sim/scene.py`, catalog origins).
- **Decision 6 (observability).** A dedicated `mcp_server` logger writes
  to `logs/mcp_server.log`, not the root logger, so the MCP library's own
  log lines stay out. One line per call: `tool=… args=… duration=…
  status=OK|<gRPC code>|INVALID_ARGUMENTS|UNKNOWN_TOOL`, plus one start
  line.
- **Decision 8 (CUJ 1).** The command is exactly as specified. A trial
  passes when `llm-cli` exits 0, its `result` event is not an error, the
  cube is on the plate, and every transcript tool call (by name, counted)
  is in the log lines written during the trial.

## Files changed

- `container/gen_protos.sh` (Modify): writes `gen/robot_service.desc`
- `gen/robot_service.desc` (generated)
- `mcp_server/server.py` (Create)
- `mcp_server/README.md` (Create)
- `.mcp.json` (Create)
- `requirements-dev.txt` (Modify): `mcp==2.3.0`
- `tests/test_mcp_server.py` (Create)
- `tests/e2e/phase6_check.py` (Create)
- `tests/e2e/phase6_cuj1.py` (Create)
- `docs/designs/robot_platform.md` (Modify): G10, `mcp_server` section
- `docs/designs/phases/phase_6_notes.md` (Create)
