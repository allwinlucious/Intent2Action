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
| `cuj1` (`tests/e2e/phase6_cuj1.py`) | 5 trials (`SimService.ResetScene` before each): an LLM CLI with only this MCP server's tools and the prompt "Put the red cube on the plate." finishes; success (from `SimService.GetTruePose`, whose poses are the objects' bottom-centre origins): horizontal distance cube↔plate ≤ 0.065 m (the 0.05 m cube lies fully on the 0.09 m-radius plate) and cube z − plate z ∈ [0.007, 0.022] (plate is 0.012 m thick). ≥ 4/5 (G10). Every tool call in the transcript appears in `logs/mcp_server.log` |
| `unit` | `pytest tests/test_mcp_server.py` passes |

## Decisions

1.  **Process:** `mcp_server/server.py` (Create), Python in the WSL venv
    (`/home/mew/untitled-project/.venv`), started by the MCP client over
    **stdio**; it puts the repo's `gen/` on `sys.path` relative to its own
    file and connects to `127.0.0.1:18100` (gRPC, `RobotService` only).
    `requirements-dev.txt` (Modify): add `mcp==2.3.0` (already installed in
    the venv). MCP API (verified in 2.3.0):
    `mcp.server.lowlevel.Server(name="robot", version=…, instructions=…,
    on_list_tools=list_tools, on_call_tool=call_tool)`, handlers
    `async def list_tools(ctx, params) -> mcp.types.ListToolsResult` and
    `async def call_tool(ctx, params: mcp.types.CallToolRequestParams) ->
    mcp.types.CallToolResult`; run with
    `async with mcp.server.stdio.stdio_server() as (r, w): await
    server.run(r, w, server.create_initialization_options())`. gRPC calls
    use the synchronous stub inside `anyio.to_thread.run_sync`.
2.  **Tools are generated from the proto at start-up**, not hand-written:
    `container/gen_protos.sh` (Modify): when `robot_service/robot_service.proto`
    is among its arguments it also writes `gen/robot_service.desc`
    (`--include_source_info --include_imports` descriptor set of that
    file). `server.py` reads it into a `descriptor_pool`, walks
    `robot.v1.RobotService`, and builds one tool per RPC:
    *   name = RPC name in snake_case (`MoveToPose` → `move_to_pose`);
    *   description = the RPC's leading comment, then one line per request
        field: `` `name` (type): `` + the field's leading and trailing
        comments joined (or nothing after the colon if it has none); enum
        fields add their value names with the values' comments;
    *   input schema = JSON Schema from the request message: double/float →
        `number`, int → `integer`, bool → `boolean`, string → `string`,
        enum → `string` with `enum` of value names (`*_UNSPECIFIED`
        omitted), message → `{"type": "object", "properties": {…}}`
        (recursive; every object schema has `"type": "object"`), repeated
        → `array`, `oneof` → all members optional with the oneof named in
        the description; `google.protobuf.Empty` →
        `{"type": "object", "properties": {}}`; `google.protobuf.Duration`
        → `{"type": "string", "description": "duration, e.g. \"30s\""}`
        (`ParseDict` accepts that form). Every field optional (proto3
        defaults apply). For `wait` fields the default is documented as
        true and the server sets it true when absent.
    *   arguments → request via `google.protobuf.json_format.ParseDict`
        (field names as in the proto, `ignore_unknown_fields=False`; a
        bad argument → MCP error "invalid arguments: …").
3.  **Tool set:** every `RobotService` RPC **except** `GetPointCloud`
    (57 600 points at the default stride 4 on the 1280×720 camera is not
    useful to a language model; it stays a gRPC API for programs). Includes
    `GetOperation` and `WaitOperation`. `SimService` is not exposed.
4.  **Results:**
    *   Normal replies → one text content block with the response as
        JSON (`json_format.MessageToDict(…, preserving_proto_field_name=True,
        always_print_fields_with_no_presence=True)`).
    *   `google.longrunning.Operation` → JSON `{name, done, state, detail,
        planned_duration_s, error?: {code, message}, response?}`: metadata
        and response are `Any`s unpacked by their `type_url` through the
        descriptor pool (`message_factory.GetMessageClass`).
    *   `GetImage` → an MCP **image content block** (`image/png`, the
        response's `png` base64) plus a text block
        `{kind, width, height, sim_time}`; DEPTH is returned the same way
        (16-bit PNG, mm), and its text block says so.
    *   gRPC errors → MCP tool result with `isError: true` and text
        `"<CODE>: <message>"` (e.g. `FAILED_PRECONDITION: robot faulted:
        simulation reset`). Connection failure → `UNAVAILABLE: robot
        service not reachable at 127.0.0.1:18100`.
    *   Deadlines: 70 s per call (`wait` calls block ≤ 60 s server-side),
        except `WaitOperation` (requested timeout + 10 s) and `Jog`
        (`duration_s` + 10 s).
5.  **Server instructions** (MCP `instructions` field, also in
    `mcp_server/README.md`): units (m, rad, N), frames (robot base =
    world, TCP between the fingertips), `wait=false` + `wait_operation`
    for long calls, typical pick sequence (`list_objects`/`detect_objects`
    → `plan_grasp` → `move_to_pose` pregrasp → `move_to_pose` grasp
    LINEAR → `grasp` → `attach_object` → lift → place → `release` →
    `detach_object`), and "call `clear_faults` after a fault".
6.  **Observability:** one log line per tool call to
    `logs/mcp_server.log` (path relative to the repo root; Python
    `logging`, format `%(asctime)s %(levelname)s mcp_server %(message)s`): tool,
    arguments ≤ 200 chars, duration, result status/code. stdout is the
    MCP channel, so nothing else is printed to stdout.
7.  **Registration:** `.mcp.json` (Create, project root) with server
    `robot`: command `.venv/bin/python`, args `mcp_server/server.py`,
    so an MCP client in this repo sees it; `mcp_server/README.md` (Create)
    shows how to add it to other MCP clients.
8.  **CUJ 1 run:** `tests/e2e/phase6_cuj1.py` starts Isaac and the
    container the way `tests/e2e/phase3_cuj_check.py:445-458` does
    (`hwm._Run().start_isaac/start_container`, `p1._wait_for_ports()`,
    `hwm._require_state("ENABLED", …)`, `cuj._wait_ready()`), then per
    trial runs an LLM CLI (no
    built-in tools, so the model cannot read the repo) with a 10-minute
    timeout, stores each stream in `logs/cuj1_trial_<n>.jsonl`, and checks
    the cube with `SimService.GetTruePose`.
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
4.  Decision 9; rerun `phase3_cuj_check.py` (the proto descriptor change
    must not affect the service); write `phase_6_notes.md` (Create).

## Constraints

No `git commit`; Docker via `sg docker -c`; no `pkill -f`; Windows
processes via `taskkill.exe /PID`. The MCP server never calls
`SimService` (tests may). Observability applies.
