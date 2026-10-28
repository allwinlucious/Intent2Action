# Phase 6: MCP server (the VLM's interface)

Part of [robot_platform.md](../robot_platform.md) (G10, CUJ 1). The API
is `robot_service/robot_service.proto` (`RobotService`); its RPC comments
become tool descriptions. Requires phases 3, 4, 5c and 5b done (phase 5's
`DetectObjects` may still be `UNIMPLEMENTED`; the VLM can use
`ListObjects`, which carries ground-truth poses until perception exists).

## Goal and done criteria

| Name | Condition |
|---|---|
| `tools` (`tests/e2e/phase6_check.py`) | an MCP client (Python SDK) on the server lists exactly the tools of decision 3, each with a non-empty description and an input schema |
| `calls` (same) | `get_state` returns joints; `move_to_pose` (`wait` true) to a reachable pose returns `done` and `state` `DONE`; `get_image` (RGB) returns an MCP image content block (PNG 1280×720); `move_to_pose` to an unreachable pose returns an MCP error result whose text contains the gRPC code and the server's message; `wait_operation` on a `wait=false` move returns it done |
| `cuj1` (`tests/e2e/phase6_cuj1.py`) | 5 trials (`SimService.ResetScene` before each): an LLM CLI with only this MCP server and the prompt "Put the red cube on the plate." finishes; success = Isaac's true `red_cube` centre is within the plate's radius (0.09 m) in x/y and between 0 and 0.06 m above the plate top. ≥ 4/5 (G10). Every tool call appears in `logs/mcp_server.log` |
| `unit` | `pytest tests/test_mcp_server.py` passes |

## Decisions

1.  **Process:** `mcp_server/server.py` (Create), Python in the WSL venv,
    started by the MCP client over **stdio**; connects to
    `127.0.0.1:18100` (gRPC, `RobotService` only). Dependencies: `mcp`
    (2.3.0, pinned in `requirements-dev.txt` → rename to
    `requirements.txt`, Modify), `grpcio`, `protobuf`; stubs from `gen/`.
2.  **Tools are generated from the proto at start-up**, not hand-written:
    `container/gen_protos.sh` (Modify) also writes
    `gen/robot_service.desc` (`--include_source_info --include_imports`
    descriptor set). `server.py` reads it, walks `robot.v1.RobotService`,
    and registers one tool per RPC with the low-level MCP server API
    (`Server.list_tools` / `call_tool`):
    *   name = RPC name in snake_case (`MoveToPose` → `move_to_pose`);
    *   description = the RPC's leading comment + one line per request
        field from its comment ("`pose`: target pose of `tool`, in the
        robot base frame"); enum fields list their values with comments;
    *   input schema = JSON Schema from the request message: double/float →
        `number`, int → `integer`, bool → `boolean`, string → `string`,
        enum → `string` with `enum` of value names (`*_UNSPECIFIED`
        omitted), message → nested `object` (recursive), repeated →
        `array`, `oneof` → all members optional with the oneof named in
        the description; `google.protobuf.Empty` → `{}`; every field
        optional (proto3 defaults apply). For `wait` fields the default is
        documented as true and the server sets it true when absent.
    *   arguments → request via `google.protobuf.json_format.ParseDict`
        (field names as in the proto, `ignore_unknown_fields=False`; a
        bad argument → MCP error "invalid arguments: …").
3.  **Tool set:** every `RobotService` RPC **except** `GetPointCloud`
    (≈ 57 600 points at the default stride is not useful to a language
    model and is large; it stays a gRPC API for programs). Includes
    `GetOperation` and `WaitOperation`. `SimService` is not exposed.
4.  **Results:**
    *   Normal replies → one text content block with the response as
        JSON (`json_format.MessageToDict`, `preserving_proto_field_name=True`).
    *   `google.longrunning.Operation` → JSON `{name, done, state, detail,
        planned_duration_s, error?: {code, message}, response?}` (metadata
        `OperationInfo` and response unpacked to their message types by
        the `kind`→type table built from the descriptor).
    *   `GetImage` → an MCP **image content block** (`image/png`, the
        response's `png` base64) plus a text block
        `{kind, width, height, sim_time}`; DEPTH is returned the same way
        (16-bit PNG, mm), and its text block says so.
    *   gRPC errors → MCP tool result with `isError: true` and text
        `"<CODE>: <message>"` (e.g. `FAILED_PRECONDITION: robot faulted:
        simulation reset`). Connection failure → `UNAVAILABLE: robot
        service not reachable at 127.0.0.1:18100`.
    *   Deadlines: 70 s per call (`wait` calls block ≤ 60 s server-side),
        `WaitOperation` uses the requested timeout + 10 s.
5.  **Server instructions** (MCP `instructions` field, also in
    `mcp_server/README.md`): units (m, rad, N), frames (robot base =
    world, TCP between the fingertips), `wait=false` + `wait_operation`
    for long calls, typical pick sequence (`list_objects`/`detect_objects`
    → `plan_grasp` → `move_to_pose` pregrasp → `move_to_pose` grasp
    LINEAR → `grasp` → `attach_object` → lift → place → `release` →
    `detach_object`), and "call `clear_faults` after a fault".
6.  **Observability:** one log line per tool call to
    `logs/mcp_server.log` (Python `logging`, the project format): tool,
    arguments ≤ 200 chars, duration, result status/code. stdout is the
    MCP channel, so nothing else is printed to stdout.
7.  **Registration:** `.mcp.json` (Create, project root) with server
    `robot`: command `.venv/bin/python`, args `mcp_server/server.py`,
    so an MCP client in this repo sees it; `mcp_server/README.md` (Create)
    shows how to add it to other MCP clients.
8.  **CUJ 1 run:** `tests/e2e/phase6_cuj1.py` starts Isaac and the
    container (harness from `tests/e2e/harness.py`), then per trial runs
    an LLM CLI with a 10-minute timeout,
    stores each transcript in `logs/cuj1_trial_<n>.json`, and checks the
    cube with `SimService.GetTruePose`.
9.  **Design doc** (`robot_platform.md`, Modify): G10 and the
    `mcp_server` section per decisions 1–4 (stdio, generated tools, image
    content, `GetPointCloud` excluded).

## Steps

1.  Decisions 1–2 and 4 with `tests/test_mcp_server.py` (Create; no
    gRPC server needed): schema for `MoveToPoseRequest` (nested `Pose`,
    enum `Motion` values `ANY`, `LINEAR`), description contains the field
    comments, snake_case names, tool set excludes `get_point_cloud`,
    Operation → JSON with an unpacked `MotionResult`, gRPC error → error
    text, `GetImage` → image block.
2.  Decisions 3, 5–7; `tests/e2e/phase6_check.py` (Create) until it
    passes.
3.  `tests/e2e/phase6_cuj1.py` (Create) until ≥ 4/5. If below, record
    each failed transcript's last tool calls and the cause in
    `phase_6_notes.md` and stop (the lead decides: tool descriptions,
    instructions, or API changes).
4.  Decision 9; earlier checks unaffected (no container change); write
    `phase_6_notes.md` (Create).

## Constraints

No `git commit`; Docker via `sg docker -c`; no `pkill -f`; Windows
processes via `taskkill.exe /PID`. The MCP server never calls
`SimService` (tests may). Observability applies.
