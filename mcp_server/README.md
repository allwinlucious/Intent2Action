# mcp_server

MCP server for the robot (phase 6): one tool per `RobotService` RPC except
`GetPointCloud`, generated at start-up from `gen/robot_service.desc` (written
by `container/gen_protos.sh robot_service/robot_service.proto`). Tool names
are the RPC names in snake_case, descriptions are the RPC and field comments
of `robot_service/robot_service.proto`, input schemas are the request
messages. It talks gRPC to `RobotService` on `127.0.0.1:18100` (the
container must be up) and never to `SimService`.

The MCP client starts it over stdio. Each tool call is logged to
`logs/mcp_server.log`.

## Registering it

an MCP client in this repository picks it up from `.mcp.json`. For an MCP client
elsewhere:

```bash
llm-cli mcp add robot -- /home/mew/untitled-project/.venv/bin/python /home/mew/untitled-project/mcp_server/server.py
```

Any other MCP client (...): add a stdio server with
absolute paths:

```json
{
  "mcpServers": {
    "robot": {
      "command": "/home/mew/untitled-project/.venv/bin/python",
      "args": ["/home/mew/untitled-project/mcp_server/server.py"]
    }
  }
}
```

## Results

*   Normal replies: the response message as JSON.
*   Long-running calls (`wait`, default true, blocks up to 60 s; with
    `wait` false poll with `get_operation` / `wait_operation`):
    `{name, done, state, detail, planned_duration_s, error?: {code, message}, response?}`.
*   `get_image`: a PNG image block (RGB 8-bit; DEPTH 16-bit in mm) and
    `{kind, width, height, sim_time}`.
*   Errors: an error result `"<CODE>: <message>"`, e.g.
    `FAILED_PRECONDITION: robot faulted: simulation reset`.

## Instructions sent to the client

The server's MCP `instructions` (`INSTRUCTIONS` in `server.py`):

> Controls a simulated Franka Panda arm with a parallel gripper (opening up
> to 0.075 m) through Intrinsic's robot stack. Units: metres, radians,
> seconds, newtons, kilograms. Poses are a position (x, y, z) and a unit
> quaternion (qx, qy, qz, qw). Frames: the robot base frame is the world
> frame (z up); every pose is in it. The TCP (tool centre point, the default
> `tool`) is between the fingertips; FLANGE is the robot flange. Object poses
> from list_objects / detect_objects are the objects' bottom centres.
>
> Long-running calls (motions, grasp, release, calibrate_camera,
> detect_objects) take `wait`: true (the default) blocks up to 60 s and
> returns the operation, false returns at once; then poll with get_operation
> or block with wait_operation. A returned operation has `done`, `state` and,
> on failure, `error`.
>
> Typical pick and place: list_objects (or detect_objects) -> plan_grasp(object)
> -> move_to_pose to the pregrasp -> move_to_pose to the grasp with motion
> LINEAR -> grasp -> attach_object -> move_to_pose up (LINEAR) -> move_to_pose
> above the place position -> lower (LINEAR) -> release -> detach_object ->
> move up.
>
> After a fault (get_state shows faulted), call clear_faults before moving
> again.
