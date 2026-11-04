# Phase L−1: perception-style object IDs (owner decision D10)

Part of [robot_platform.md](../robot_platform.md) (D10) and
[model_learning.md](../model_learning.md). Objects are no longer known by
scene-file names (`red_cube`, `plate`). They get neutral IDs with
attributes, as a perception stack would give them. The model lists the
objects, decides which ID matches what it was told, and commands with IDs.
There is no `find_objects`; grounding is the model's job.

## Done criteria

*   `list_objects` returns only IDs `obj_<k>` (k a random two-digit
    number), each with `category`, `color`, `size`, `pose`, `source`,
    `attached_to`; no scene-file names appear anywhere in its reply.
*   After `SimService.ResetScene` every object has a new random ID (no ID
    repeats across 5 consecutive resets for the same object).
*   Every call that takes an object takes its ID; an unknown ID returns
    `NOT_FOUND` "no object with id X; list_objects shows the current ids".
*   All existing checks pass with IDs: `phase3_hwm_check.py`,
    `phase3_scene_check.py`, `phase3_cuj_check.py`, `phase4_check.py`,
    `phase5c_check.py`, `phase5b_check.py`, `phase6_check.py`,
    the phase-6 CUJ 1 check (since removed) (≥ 4/5, prompt unchanged: "Put the red cube on the
    plate."); `pytest tests/` passes.
*   New check `tests/e2e/ids_check.py` (Create) covers the first three
    bullets.

## Decisions

1.  **API** (`robot_service/robot_service.proto`, Modify; field numbers
    reused where the meaning is the same):
    *   `WorldObject { string id = 1; string source = 2; Pose pose = 3;
        string attached_to = 4; string category = 5; string color = 6;
        double size_x = 7; double size_y = 8; double size_z = 9; }`
        (sizes in m, axis-aligned box in the object's frame; comments say
        so). `name` is gone.
    *   `ObjectRequest { string id = 1; }` (used by `AttachObject`,
        `DetachObject`, `RemoveObject`).
    *   `PlanGraspRequest { string object_id = 1; }`.
    *   `DetectObjectsRequest { repeated string categories = 1; bool wait = 2; }`;
        `Detection { string object_id = 1; Pose pose = 2; double score = 3; string category = 4; }`.
    *   `SimService`: `AddObject(AddObjectRequest{catalog, pose})` returns
        `AddObjectResponse{id}`; `RemoveObject(ObjectRequest)`;
        `GetTruePose(GetTruePoseRequest{frame})` where `frame` is an object
        ID, `panda_hand` or `Camera_Pseudo_Depth`; new
        `ListTrueObjects(google.protobuf.Empty) returns (TrueObjects)` with
        `TrueObject{id, catalog, pose}` (tests only; never in MCP).
    *   RPC comments (they become MCP descriptions): `ListObjects` says
        "objects are identified only by `id`; match the user's words to
        `category`, `color`, `size` and `pose`; ids change after a scene
        reset"; every object argument says "an id from `list_objects`".
2.  **SimLink** (`robot_service/simlink.proto`, Modify): `Spawn(SpawnRequest{catalog, pose})
    → SpawnResponse{id}`; `Delete(NameRequest{name})` takes an ID;
    `GetTruePose(frame)` as above; new `ListTrue(Empty) → TrueObjects`
    (import from `robot_service.proto`); `SetAttached(name)` takes an ID.
3.  **Catalog attributes** (`tools/build_catalog_meshes.py`, Modify; writes
    `assets/objects/*/object.yaml`): new keys `category` and
    `color_name`:

    | catalog | category | color_name |
    |---|---|---|
    | `table` | `table` | `brown` |
    | `red_cube` | `cube` | `red` |
    | `plate` | `plate` | `white` |
    | `peg` | `peg` | `blue` |
    | `hole_block` | `block_with_hole` | `grey` |
    | `obstacle` | `box` | `grey` |
    | `calib_board` | `calibration_board` | `white` |
4.  **ID assignment in `scene_sync`** (`container/services/scene_sync.py`,
    Modify):
    *   It keeps a map Isaac prim name ↔ ID. IDs are `obj_<k>`, k drawn
        without replacement from 10–99 with `random.SystemRandom`.
    *   A fresh map is drawn at start-up and after every
        `SimLink.Reset`. Before re-creating, it deletes every mirrored
        object from `world` and `sim_world`.
    *   `Spawn` gives the prim a new internal name `spawn_<n>` (counter)
        and a fresh ID, and returns the ID.
    *   World objects are created with `name` = ID (`name_is_global_alias`
        as now). `user_data` holds `catalog`, `category`, `color`
        (`StringValue`) and `source`.
    *   `calib_board` keeps its internal name and is never given an ID.
5.  **`robot_service`** (`container/services/robot_service.py` and
    `container/services/robot/*.py`, Modify):
    *   `ListObjects` returns every object in `world` except `franka`,
        `franka_hand`, `d455` and `calib_board`. Attributes come from
        `user_data`; size comes from the catalog `dims`.
    *   Every object argument is looked up by ID; an unknown ID gives the
        `NOT_FOUND` message above. `PlanGrasp`, `Attach`, `Detach` and the
        `SimService` calls use IDs. `PlanGrasp`'s MoveIt collision
        objects use IDs as their `id`.
    *   `ResetScene` returns once `scene_sync` has recreated the objects
        with their new IDs.
6.  **MCP** (`mcp_server/server.py`, Modify only if needed; regenerate
    `gen/robot_service.desc`): the server instructions' pick sequence
    starts with "`list_objects`, choose the id whose category, color,
    size and pose match the request".
7.  **Tests:** each e2e check that used scene names now resolves IDs with
    `SimService.ListTrueObjects` (catalog → ID) after each reset, through
    one helper `harness.ids()` (`tests/e2e/harness.py`, Modify). Unit
    tests are updated for the renamed fields.
8.  **Design doc** (`robot_platform.md`, Modify): the RobotService table,
    "Ground truth and belief" and the D10 row (no `find_objects`; the
    model grounds descriptions itself).

## Steps

1.  Decisions 1–3; regenerate the stubs.
2.  Decisions 4–6; rebuild the image.
3.  Decision 7, then `tests/e2e/ids_check.py` and every check listed
    under the done criteria (the known `obstacle` flake may need one
    rerun).
4.  Decision 8; write `phase_ids_notes.md` (Create).

## Constraints

No `git commit`; Docker via `sg docker -c`; no `pkill -f`; Windows
processes via `taskkill.exe /PID`; `~/intrinsic-core` untouched except
the `ext` symlink; Bazel only via `container/build_intrinsic.sh`.
Observability applies. If a decision is impossible, stop and record it.

## Addendum A (stage-9 review; these override the sections above where they differ)

1.  **ID draw:** a pure class `IdAllocator` in `container/services/scene_ids.py`
    (Create): `draw(prims) -> dict[prim, id]` takes k from 10–99 without
    replacement, **excluding every k used in the previous map**, so no
    object keeps its ID across a reset. Unit tests in
    `tests/test_scene_ids.py` (Create): uniqueness within a map,
    disjointness from the previous map over 50 draws, format `obj_\d{2}`.
2.  **Done criteria, exact:** every `WorldObject.id` matches
    `^obj_\d{2}$` and none equals a key of `sim/scenes/default.yaml`;
    `category` may (e.g. `plate`). IDs are unique within a scene, and
    after each of 5 resets no object keeps its previous ID.
3.  **Key spaces in `scene_sync`:** every internal structure (`known`,
    `last_seen`, `attached`, `catalogs`, `ros.poses`) stays keyed by
    **Isaac prim name** (= `/tf` frame). The ID is used only at two
    boundaries: (a) the world service: objects are created, updated and
    deleted under name = ID, with `user_data["prim"]` = prim name;
    `_delete_stale` compares `user_data["prim"]`, not the world name;
    (b) SimLink: request fields carrying an object are IDs, translated to
    the prim via the map (unknown → `NOT_FOUND` with the standard
    message); `GetTruePose` also accepts the pass-through frames
    `panda_hand` and `Camera_Pseudo_Depth`; `calib_board` is accepted by
    name in `SetAttached`, `Delete` and `GetTruePose` (internal, never
    listed).
4.  **Reset:** `Sync` has a lock held by the sync thread for each `/tf`
    processing pass. `SimLink.Reset` (also when called directly, e.g.
    CUJ 5) takes the lock, calls `/sim/reset`, waits for the new epoch,
    deletes every object with `user_data["source"] == "ground_truth"` from
    both worlds, clears the prim structures, draws a new map, releases the
    lock and returns. The sync thread re-creates objects from the next
    `/tf`. `robot_service.ResetScene` then waits (timeout 10 s) until
    every ID in `SimLink.ListTrue` exists in `world` (`GROUND_TRUTH`) or in
    `sim_world` (`PERCEPTION`).
5.  **Start-up with persisted worlds** (D10: IDs persist while an object
    stays put): at start-up `scene_sync` reads existing ground-truth
    objects in `world` and adopts their ID for their `user_data["prim"]`;
    only prims without an ID get a fresh draw. A container restart
    therefore keeps IDs; only resets change them.
6.  **Errors:** one helper `unknown_object(id)` in
    `container/services/robot/__init__.py` builds `NOT_FOUND` "no object
    with id X; list_objects shows the current ids", used by `World.get`
    and `grasp.py`. `franka`, `franka_hand`, `d455` and `calib_board`
    passed as an object argument give the same `NOT_FOUND`.
7.  **Protos in full** (old fields `reserved`):
    *   `WorldObject { string id = 1; string source = 2; Pose pose = 3; string attached_to = 4; string category = 5; string color = 6; double size_x = 7; double size_y = 8; double size_z = 9; }`
        — comment: "`pose` is the bottom centre of the object; its
        bounding box (`size_*`, m) extends upward from it in the object's
        frame".
    *   `ObjectRequest { string id = 1; }`; `PlanGraspRequest { string object_id = 1; }`.
    *   `DetectObjectsRequest { reserved 1; repeated string categories = 3; bool wait = 2; }`;
        `Detection { string object_id = 1; Pose pose = 2; double score = 3; string category = 4; }`.
    *   `AddObjectRequest { string catalog = 1; reserved 2; Pose pose = 3; }`,
        `AddObjectResponse { string id = 1; }`; `GetTruePoseRequest { string frame = 1; }`;
        `TrueObject { string id = 1; string catalog = 2; string prim = 3; Pose pose = 4; }`,
        `TrueObjects { repeated TrueObject objects = 1; }`.
    *   SimLink: `SpawnRequest { string catalog = 1; reserved 2; robot.v1.Pose pose = 3; }`,
        `SpawnResponse { string id = 1; }`, `IdRequest { string id = 1; }`
        (replaces `NameRequest` for `Delete`), `SetAttachedRequest { string id = 1; bool attached = 2; }`,
        `ListTrue(google.protobuf.Empty) returns (robot.v1.TrueObjects)`.
8.  **Terms:** `category` and `color` (a word) in the API and docs;
    catalog `object.yaml` keeps `color` (RGB) and gains `category` and
    `color_name`. Add the two keys by editing the 7 `object.yaml` files
    and `tools/build_catalog_meshes.py` so a rebuild writes them; **do not
    regenerate meshes**. `scene_sync` reads `object.yaml` for them.
9.  **Tests:** the helper is `ids(sim) -> dict[str, str]` (prim name →
    ID, from `ListTrueObjects`) in `tests/e2e/phase3_cuj_check.py` (next
    to `_sim`); every check calls it after each reset and container
    restart. Files to modify: `phase3_cuj_check.py` (incl. `check_misc`:
    every listed object has an `obj_\d{2}` ID and the default prims map
    to listed IDs), `phase3_scene_check.py` (compare world objects with
    `/tf` through `user_data["prim"]`; the spawned second cube is found
    by the ID `Spawn` returns), `phase4_check.py`, `phase5b_check.py`,
    `phase5c_check.py` (incl. its `GRASP_PLANNER=off` restart),
    `phase6_check.py`, the phase-6 CUJ 1 check (since removed). `tests/e2e/ids_check.py`
    (Create) calls `PlanGrasp`, `AttachObject`, `DetachObject`,
    `RemoveObject` and `GetTruePose` with `obj_00` (never drawn: k ≥ 10)
    and expects the `NOT_FOUND` message; runs 5 resets for criterion 2;
    checks `AddObject` returns an ID that appears in `list_objects`.
10. **Docs:** `robot_platform.md`: RobotService table rows for objects,
    the SimService table, "Ground truth and belief", and D10's wording
    (`category`, `color`, `obj_<k>`); `model_learning.md`: remove
    `find_objects` (lines on object identity and phase L−1).
    `mcp_server/server.py` (Modify): the instructions' pick sequence
    becomes "`list_objects`, choose the id whose category, color, size and
    pose match the request, then `plan_grasp(object_id)` → …".
