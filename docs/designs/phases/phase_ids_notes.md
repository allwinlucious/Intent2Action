# Object IDs: implementation notes

Status: **done.** `list_objects` returns only `obj_<k>` IDs, each with
`category`, `color`, `size_*`, `pose`, `source` and `attached_to`. No ID equals
a key of `sim/scenes/default.yaml`. IDs are unique, and after each of 5 resets
no object kept its ID. An unknown ID returns `NOT_FOUND` "no object with id X;
list_objects shows the current ids". Every listed check passes.
`phase5b_check.py` needed one rerun: its `insert` criterion failed 7/10 on the
first run, from the search stall documented in phase 5b (below). No container
or Isaac process is left running. Nothing was committed.

| Check | Result | Output |
|---|---|---|
| `tests/e2e/ids_check.py` | PASS 4/4: `list`, `unknown`, `add`, `resets` (5 resets, all IDs listed again 4.2 s after ResetScene returned) | `logs/ids_check.out` |
| `phase3_hwm_check.py` | PASS 7/7 | `logs/ids_run_phase3_hwm_check.out` |
| `phase3_scene_check.py` | PASS 5/5; `restart` also asserts the IDs are kept across `docker restart` | `logs/ids_run_phase3_scene_check.out` |
| `phase3_cuj_check.py` | PASS 8/8 on the first run (`obstacle` passed) | `logs/ids_run_phase3_cuj_check.out` |
| `phase4_check.py` | PASS 6/6 | `logs/ids_run_phase4_check.out` |
| `phase5c_check.py` | PASS 6/6: cube 10/10, peg 10/10 | `logs/ids_run_phase5c_check.out` |
| `phase5b_check.py` | Run 1: 5/6, `insert` FAIL 7/10. Run 2: PASS 6/6, `insert` 9/10 | `logs/ids_run_phase5b_check.out`, `…_check_2.out` |
| `phase6_check.py` | PASS 2/2 (`tools`, `calls`; `calls` now includes `list_objects`) | `logs/ids_run_phase6_check.out` |
| `phase6_cuj1.py` | PASS **5/5** (need 4), prompt unchanged, 14–15 tool calls, cube 0.2–0.7 mm from the plate centre | `logs/ids_run_phase6_cuj1.out` |
| `pytest tests/` | 92 passed (89 before, plus 3 in `tests/test_scene_ids.py`) | — |

The image was rebuilt with `docker build` (`logs/phase_ids_build.log`). No
C++ changed, so Bazel was not run. Isaac and container logs are in
`logs/ids_isaac.log` and `logs/ids_container.log`.

## phase5b `insert` on run 1

The three failed trials (2, 8, 9) all made contact normally, 2.8–3.1 mm off
the hole axis. Then the search `MoveCompliant` aborted with "settled before
reaching the target (position error 0.0018 m)" at 3.6–3.8 N, 0.8–0.9 mm deep.
This is the search stall in [phase_5b_notes.md](phase_5b_notes.md) finding 20,
where run 8 passed 9/10 with one such stall. The ID change does not touch the
insert path: force control, `GetTruePose` of the hole, or Isaac physics. The
pre-contact placement (+2.5 to +3.1 mm off axis) matches run 8. Run 2 passed
9/10; its one failure was a contact force of 10.94 N, outside (12, 18) N. The
`insert` margin at 9/10 remains thin. It is open from phase 5b and was not
changed here.

## Rulings made during implementation

1. **Worked in place.** No worktree and no commits, per the task's rules.
2. **`IdAllocator` API.** Besides `draw(prims)`, it has `adopt(ids)`,
   `add(prim)`, `forget(prim)` and `prim(id)`. These cover start-up adoption,
   spawn and delete. `add` also excludes the previous map's k, so a spawned
   object cannot take an ID that was just retired by a reset.
3. **Un-adopted objects are deleted at start-up.** The first `ids_check` run
   failed `list`: the persisted world still held prim-named objects (`plate`,
   `peg`, …) from before this change, because `_delete_stale` only runs 5 s
   after the first `/tf`. `assign_ids()` now deletes ground-truth objects it
   did not adopt from both worlds (logged with `reason=not_adopted_at_start`).
4. **Adoption reads the mirrored worlds.** It reads `sim_world`, plus `world`
   in GROUND_TRUTH mode, rather than `world` only. In PERCEPTION mode, `world`
   holds no ground-truth objects to adopt.
5. **Delete is immediate.** `SimLink.Delete` removes the world objects, the
   prim structures and the ID at once, instead of waiting for `/tf` staleness.
   `calib_board` Delete keeps the old path.
6. **Spawn prim names.** `spawn_<n>` skips names already on `/tf` or in the
   catalogs.
7. **NOT_FOUND message prefix.** SimLink errors reach SimService callers as
   "scene_sync SimLink.X: no object with id …". `ids_check` therefore checks
   that the details contain the standard message rather than equal it.
8. **Non-scene frames.** `SimService.GetTruePose` passes `calib_board`
   through, as well as the robot and camera frames. `RemoveObject`,
   `AttachObject`, `DetachObject` and `PlanGrasp` reject the robot, camera and
   board with the same NOT_FOUND message.
9. **`ListTrue` contents.** It lists every mapped prim, even before the prim
   appears on `/tf` (pose unset). Map entries are dropped only by Delete and
   Reset.
10. **ID helper location.** Decision 7's `harness.ids()` was superseded by
    Addendum A's `ids(sim)` in `phase3_cuj_check.py`, which the other checks
    import. `tests/e2e/harness.py` is unchanged.
11. **Checks without scene names.** `phase4_check.py` and `phase6_check.py`
    used no scene names. `phase4`'s `_restart` now asserts the IDs are kept,
    and `phase6`'s `calls` adds a `list_objects` MCP call that asserts the ID
    format. `phase3_hwm_check.py` is unchanged.
12. **Unit tests.** No existing unit test used the renamed fields. Only
    `tests/test_scene_ids.py` was added.
13. **MCP instructions.** Besides the plan's pick-sequence wording, they gained
    one sentence ("Objects are identified only by an `id` … ids change after a
    scene reset") and now pass `(id)` to `attach_object` and `detach_object`.
14. **Object catalog.** `category` and `color_name` were added to each
    `object.yaml` and to `tools/build_catalog_meshes.py`. The yaml files match
    its `safe_dump` output byte for byte. The meshes were not regenerated.
