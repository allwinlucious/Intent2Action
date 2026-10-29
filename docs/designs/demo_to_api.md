# Show it once: a fine-tuned agent programs the robot from a demonstration

*   **Author:** mew
*   **Status:** Draft for owner review
*   **Date:** 2026-10-29

--------------------------------------------------------------------------------

## Overview

Robots are cheap now; telling them what to do is not. Every new task in a small factory, lab or warehouse
means an expert writing and testing robot code. This project removes that step:

1.  A person shows the task once, on a phone video. Examples: put all the cubes in a box, sort parts into
    boxes by shape or size, put one item in a box.
2.  An agent works out the **intent**, the rule behind what the person did, not their hand motion.
3.  The agent **writes a robot program** for that intent. The program is short, readable and parametric,
    and calls only the robot API. The person can read it and approve it.
4.  A **runtime** executes the program in any new scene (other positions, numbers of objects and object
    ids) with no model in the loop.
5.  When a step fails ("grasp blocked by obj_31", "box full"), the runtime **calls the agent back** with
    the program, the failing line, the error and the world state. The agent recovers with tool calls,
    and the program resumes.
6.  The program ends by verifying the result against the intent.

The agent is fine-tuned to write better robot programs and to recover better, and both are measured. The
choice of fine-tuning method comes from a selection study (raw model vs. fine-tuning vs. an action head; see
"Evidence"), which is part of the result.

The final model is **Muse Glimmer** (owner decision D13). It has a vision encoder, so one model can both watch
the video and call the tools. Approaches are chosen first with small open models on the local RTX 3080 and
on Kaggle, then the chosen approach is trained on Glimmer in the cloud, as in [model_learning.md](model_learning.md).

This builds on the robot platform ([robot_platform.md](robot_platform.md)) and on the results of the local
learning run (`docs/designs/model_learning_results.md` on branch `throwaway/learning`).

## The Problem

The learning run showed two things:

*   **Fine-tuning fixes single calls.** After SFT on the stack's own replies, wrong calls fell from 14 % to
    4 % and 95 % of stack errors were corrected.
*   **Whole tasks still fail in the steps between calls.** The model has to compute a place height from
    object poses and sizes, keep going after a collision error, and check before claiming success.

A demonstration-driven task adds two harder problems on top:

1.  **Seeing what happened:** which object moved where in a video of a hand.
2.  **Generalising from one example:** the person put the red, blue and green cubes in the box and left the
    cylinders; was the rule "cubes", "these three colours" or "everything left of the plate"?

The result must be *reliable*:

*   valid calls;
*   recovery from errors;
*   a check of the world before reporting success;
*   saying so when the intent is unclear or impossible in the new scene.

## Glossary

*   **Program:** a short Python program written by the agent, using only the robot API, loops and
    conditions (see "Program format"). It is parametric: it finds objects by attributes at run time.
*   **Runtime:** sandboxed code that checks and executes a program and hands failures to the agent.
*   **Recovery handoff:** the runtime calling the agent with the program, the failing line, the error and
    the world state; the agent returns resume, skip, retry or stop after fixing what it can with tool calls.

*   **Demonstration (demo):** a video of a person doing one task with real or simulated objects, seen from
    roughly the robot camera's viewpoint.
*   **Event:** one object movement seen in a demo: object (by attributes) → destination (box or object, by
    attributes).
*   **Intent:** the rule behind the events, in a typed form (see "Intent schema"), for example "every
    object with category cube → into the box".
*   **Transfer:** carrying out an intent in a new scene. The task is scored on the new scene only.
*   **Rule checker:** code that evaluates an intent against Isaac's true object poses (success or failure,
    per object).

## Goals

*   **G1:** Infer the intent from one demo video for these task families:
    *   put all objects of one kind in a box;
    *   sort by shape, by size or by colour into boxes;
    *   put one named item in a box;
    *   put objects on another object.
*   **G2:** The agent writes a program for the intent that passes the runtime's checks, is readable to the
    user, and carries the intent out in new scenes, scored by the rule checker; the program reports per object
    what was done.
*   **G2b:** Running a program needs no model calls; the agent is invoked only to write the program and for
    recovery.
*   **G3:** Reliability:
    *   programs that are valid and correct on the first try;
    *   successful recovery handoffs (injected failures: blocked grasp, full box, timeout, moved object);
    *   few wrong calls;
    *   recovery from collision, IK and timeout errors;
    *   a check (`list_objects`) before the final answer;
    *   a clear refusal or question when the intent cannot be determined, or no matching object or box
        exists.
*   **G4:** The intent is shown to the user before the robot moves (CUJ 2) and can be corrected in words.
*   **G5:** Measured on held-out synthetic demos and on the owner's real filmed clips (sim-to-real gap
    reported separately).
*   **G6:** The same data, code and metrics run with a small local model and with Glimmer; only the model
    and its chat template change.
*   **G7:** Fine-tuning is shown to help: raw vs. fine-tuned Glimmer on program validity, program correctness,
    recovery success and model calls per task, next to the selection study's results.

## Non-Goals

*   **Imitating the hand trajectory:** the robot uses its own planner and grasps.
*   **Executing on a real robot:** execution stays in the sim. Real videos are only an *input*. Real
    execution needs the deferred perception stack (D9).
*   **Long-horizon assembly, deformable objects, tool use.**
*   **An executive or behaviour trees** (as before). The agent's program is plain Python in a sandbox, not a
    behaviour tree; `pick` and `place` (D12) are two fixed, planner-backed calls in `robot_service`, not a
    skill framework.

## Owner Decisions

*   **D11 (owner, 2026-10-29, revised the same day):** one agent (one VLM) has three roles:
    1.  **Understand:** it captures the intent from the video as events plus a typed rule, shown to the user.
    2.  **Program:** it writes a program for the intent, shown to the user and checked by the runtime.
    3.  **Recover:** it is called back by the runtime when a program step fails, fixes the situation with
        tool calls, and tells the runtime to resume, skip, retry or stop.

    The program runs without the model; verification is the program's last step. (Earlier version of D11:
    the agent drove every call live; see Alternative E.)
*   **D12 (owner, 2026-10-29):** object-level calls `Pick(object_id)` and `Place(object_id, target_id)` in
    the API, backed by the grasp planner, the motion planner and a new place planner. The model says *what*
    and *where* ("pick obj_12", "place obj_12 in box obj_07"); the planners compute *how*. This removes the
    pose arithmetic that caused most whole-task failures in the learning run.
    *   The low-level calls stay for fine control and special cases: contact moves, insertion, jogging.
    *   `pick` and `place` make one planned attempt and never retry or improvise. On failure they return a
        clear reason. The runtime hands every failure to the agent (recovery handoff), which can only learn
        recovery if it sees the failures.
    *   This **changes D4** ("no executive, skills or behaviour trees"). There is still no executive and
        there are no behaviour trees, and the model still sequences the calls. `pick` and `place` are the
        only composite calls, implemented in plain code in `robot_service`.
*   **D13 (owner, 2026-10-29):** Muse Glimmer is the final model.
*   **D14 (owner, 2026-10-29):** the owner films the real demo clips: putting all cubes in a box, sorting by
    shape, sorting by size, putting one item in a box, and similar.

## Open Questions

1.  **Real objects:** which objects will appear in the videos (shapes, colours, sizes, boxes)? The sim
    catalog will copy them, so that sim and real demos show the same kinds of objects.
2.  **Ambiguity:** when one demo fits more than one rule, should the model ask (default), take the
    simplest rule, or ask for a second demo?
3.  **Confirmation:** always show the intent and wait for "yes" (default), or only when the model is
    unsure?
4.  **Something-Something v2:** real hand videos with templated labels ("Putting [something] into
    [something]") could help with real video. Its licence must allow this use; to be checked.

## Critical User Journeys

### CUJ 1: Show, then let the robot do it

1.  The user films themselves putting all the cubes from a mixed pile into a box.
2.  The model reports what it saw and the rule:
    *   events: red cube → box, small blue cube → box, green cube → box (the cylinders stayed);
    *   intent: every cube → into the box.
3.  The agent writes the program and shows it:

    ```python
    box = find(category="box")
    for obj in find_all(category="cube"):
        pick(obj)
        place(obj, box)
    verify(all(inside(o, box) for o in find_all(category="cube")))
    ```

4.  The user confirms. The sim scene has 2 cubes and 3 cylinders in other places.
5.  The runtime runs the program with no model calls. The robot puts both cubes in the box, and the
    program's `verify` passes. Report: "obj_12 (red cube) and obj_40 (yellow cube) are in the box obj_07;
    the cylinders were left as in the demo."

### CUJ 2: Correct the intent before execution

The model says "every red object → box". The user replies "no, every cube". The model updates the intent and
runs.

### CUJ 3: Unclear or impossible

*   **Unclear demo:** the person put only the red cube in the box, and the scene has no other red objects.
    The model asks: "only the red cube, or all cubes?"
*   **Impossible:** the new scene has no box. The model says so and does not move.

### CUJ 4: Recovery during execution

Line 4 of the program, `pick(obj_31)`, fails: "grasp blocked by obj_44". The runtime pauses and calls the
agent with the program, the line, the error and the world state. The agent moves obj_44 aside with `pick`
and `place` (to a free spot on the table), then answers "retry". The program resumes and finishes. Other
cases: "box full" → the agent uses another box that matches the rule, or stops and explains. The runtime
logs each handoff, and the UI highlights it.

## Technical Plan

### Intent schema

The intent is a list of rules. Each rule maps a selector to a destination:

```json
{"rules": [
  {"select": {"category": "cube"}, "to": {"relation": "in", "target": {"category": "box"}}}
 ],
 "keep": "everything else stays",
 "events": [
  {"object": {"category": "cube", "color": "red", "size": "large"}, "to": {"relation": "in", "target": {"category": "box"}}}
 ]}
```

*   **Selectors** use the attributes `list_objects` already reports (category, colour) plus a size class
    (small / medium / large, from the catalog sizes), or "the one object" for single-item tasks.
*   **Sorting** is several rules, one per group, for example `{"size": "small"} → box A` and
    `{"size": "large"} → box B`. Boxes are named by attributes (e.g. the red box), not ids, because ids
    differ between scenes.
*   **Events** are what the model saw. **Rules** are what it concluded. A built-in consistency check
    requires the rules to explain every event and to select no object that the person left in place. A
    rule that fails the check is not executed.

### Object catalog additions

Built to mirror the owner's real objects (Open Question 1). The likely set:

*   **Open boxes:** two or three colours, about 20 × 15 × 8 cm.
*   **Cubes:** 3.5 / 5 / 6.5 cm.
*   **Cylinders:** two sizes.
*   **A cuboid "bar".**

Each gets `object.yaml` (category, colour, size class) and is built with the existing catalog tools. Spheres
are avoided because they roll.

### API additions

Both new calls are long-running operations like the motions (`wait`, `get_operation`, status and reason
on failure, logged as today):

*   **`Pick(object_id)`:**
    1.  `PlanGrasp` (Intrinsic's MoveIt grasp planner).
    2.  Move to the pre-grasp pose (any path), then to the grasp pose (linear).
    3.  Close the gripper and check that it is holding something.
    4.  Attach the object in the world model.
    5.  Lift (linear).

    It fails with a clear reason, for example: no grasp found, object too wide, grasp blocked by obj_31, the
    gripper closed on nothing, or a fault.
*   **`Place(object_id, target_id)`:**
    1.  A new place planner chooses a collision-free spot: inside a box (free floor area, away from the
        walls and from objects already there) or on top of a plate or object. It computes the TCP pose from
        the held object's grasp offset and the target's top.
    2.  Move above that spot, then lower (linear).
    3.  Open the gripper and detach the object.
    4.  Retreat (linear).

    It fails with a reason: target full, no reachable spot, not holding `object_id`, or a fault.
*   **`PlanGrasp`, `move_to_pose`, `grasp`, `release`, `attach_object` and the force calls stay unchanged**
    for fine control. Insertion (CUJ 6 of the platform) still uses them.
*   **Episode call limit:** 60. Sorting 6 objects takes about 15–20 calls with `pick`/`place`.

### Program format and runtime

**Program format:** a Python subset, checked before running (AST whitelist):

*   **Allowed:** assignments, `for`, `if`, `while` with a bounded loop count, comparisons, list
    comprehensions, `sorted`/`len`/`min`/`max`, and these API functions:
    *   `find`, `find_all` (by attributes; return object handles);
    *   `pick`, `place`;
    *   `inside`, `on_top`, `size_of`;
    *   `verify`, `report`.

    They map onto `list_objects`, `pick`, `place` and the rule checker's geometry.
*   **Not allowed:** imports, attribute access other than an object's fields, file and network access, and
    the low-level motion calls (those stay for the agent's recovery tool calls).

**Runtime (`program_runtime`, plain Python next to the MCP server):**

1.  Parse and check the program. A rejected program goes back to the agent with the reason; this counts
    against "valid on the first try".
2.  Dry run against the current `list_objects` with stubbed motions: every `find` must match, and the loop
    counts must be finite.
3.  Execute step by step and log each call (as `agent.py` does).
4.  **On a failed call:** recovery handoff. The agent gets the program text, the failing line, the error
    message, `get_state` and `list_objects`, and the robot tools. It returns `{"action": "retry" | "skip" |
    "resume_at": <line> | "stop", "note": "…"}`. At most 3 handoffs per run, then stop and report.
5.  `verify` failing at the end is also a handoff; its only actions are fix + re-verify, or stop.

**Training (the "B" approach from the selection study, the recipe's SFT):**

*   **Program writing:** pairs of (intent + scene summary → program), from a rule-to-program expert that
    covers every task family, with variations in style (loop order, helper variables) so the model learns
    the pattern rather than one template; plus programs from Muse Spark Open kept only when they run
    correctly.
*   **Recovery:** pairs of (program + failing line + error + state → tool calls + decision), from injected
    failures in the sim (blocked grasps, full boxes, moved objects, timeouts). Recovery is scored by whether
    the task then succeeds.
*   **Kept from the study:** stage-1 single-call data with 30 % replay (call validity), and a short reasoning
    line before each decision.

### Reading a demo video (SAM 3.1 + VLM)

*   **SAM 3.1** (through its model API) segments and tracks every object across the clip. Prompting it with
    the catalog's object words ("cube", "box", "cylinder") gives a mask track per object. Each object's
    start and end position, and whether it ended inside a box's mask, give candidate **events** directly
    from pixels.
*   **The VLM (Glimmer)** gets selected frames with the masks drawn and numbered, plus the track summary.
    It writes the events (with attributes: colour, shape, size) and the intent rule in the schema above.
    SAM tells it *what moved where*; the VLM explains *why* (the rule).
*   **Muse Spark Open** (hosted, through its API) is the comparison model for reading demos, and the
    teacher that produces whole-task executor episodes (kept only when the rule checker passes).

### Synthetic demo videos (Isaac; fallback)

Built only if the real-clip results of the SAM + VLM path are too weak to train on (decided after V0).

*   A **kinematic human hand** (a rigged hand mesh) does the demo in a random scene: reach, close, carry the
    object along a natural path, open, leave the frame. The object is attached to the hand while carried;
    no physics grasp is needed.
*   **Rendered from the robot's D455 camera** (rendering is on here; physics-only mode stays for executor
    runs). The video is downsampled to the model's frame rate, about 1–2 frames per second plus the first
    and last frame.
*   **Randomised:** layouts, object sets and counts, distractors, lighting, table texture, hand skin tone
    and size, speed, order, camera pose (±5 cm, ±5°).
*   **Labelled automatically** from the sim: the events, the intent, and the executor's program for the new
    scene.
*   **Size:** a few thousand clips. Generation runs unattended in the sim windows (`learn/supervisor.py`).

### Real demo clips (filmed by the owner)

The filming protocol below goes to the owner. About 50 clips in total:

*   **Camera:** phone or webcam on a fixed stand, about 1 m from the table centre, looking down at about
    45°, the whole table in view. Keep the same place for all clips.
*   **Each clip:** one task, 10–30 s. Start and end with the hands out of view, so that the first and last
    frames show the before and after.
*   **Tasks:**
    *   all cubes in a box;
    *   sort by shape;
    *   sort by size;
    *   sort by colour;
    *   one item in a box;
    *   an item on another object;
    *   a few unclear ones (only one object moved).

    About 6–8 clips each, with different starting layouts and object sets.
*   **Labels:** a one-line description per clip in a CSV (file name, what you did). I turn that into the
    intent label.

They are used only for evaluation, plus a small fine-tuning split if needed.

### Models and training

*   **Local selection, step 1 (video → events + intent):** a small open vision-language model that fits
    the 3080 or Kaggle's 2× T4 at 4-bit (for example a Qwen VL model of 4–8B).
    *   First zero-shot on the real clips (the baseline).
    *   Then SFT on synthetic demos.
*   **Executor:** the text model from the learning run, retrained with the new data.
*   **Final:** Glimmer for both steps in the cloud with the recipe's settings (LoRA/QLoRA SFT, then
    episode-level GRPO with the rule checker as reward), on H100s.

### Evidence: the selection study (done, on Qwen3 8B)

The fine-tuning method was chosen by measurement, not assumption (`docs/designs/model_learning_results.md`):

| Version | Single calls right on first try | Recovery after an error | Sim tasks (30) | Wrong calls |
|---|---|---|---|---|
| Raw model | 70 % | 53 % | 9/30 | 14 % |
| Fine-tuned (recipe SFT), single calls | 89 % | 95 % | 4/30 | 4 % |
| Fine-tuned, + whole tasks | 88 % | 95 % | 10/30 | 6 % |
| Frozen model + action head | 31 % | 25 % | 4/30 | 14–20 % |

What it showed, and how this design uses it:

*   **SFT on the robot stack's own replies makes calls reliable:** kept.
*   **Single-call data alone teaches giving up:** whole-task and recovery data are always included.
*   **Pose arithmetic was the main whole-task failure:** hence `pick`/`place`, so programs never compute
    poses.
*   **The action head cannot copy exact ids and numbers from context**, and loops: dropped.
*   **Driving every call live costs a model call per step:** hence programs, written once.

### Evaluation

| Metric | On |
|---|---|
| Event accuracy (object and destination per event) | held-out synthetic demos, real clips |
| Intent accuracy (the rule selects the same objects as the true rule in the new scene) | same |
| Asks when the demo is ambiguous / refuses when impossible | marked clips |
| Program valid on the first try (passes the runtime's checks) | held-out intents × scenes |
| Program correct (task success in the new scene, rule checker, per object and whole task) | sim, held-out layouts |
| Recovery success (task succeeds after injected failures; handoffs used) | sim, injected failures |
| Model calls and wall time per task (program + recoveries) vs. driving every call live | sim |
| Raw vs. fine-tuned Glimmer on all of the above | same sets |
| End to end: demo → intent → execution | synthetic demos; real clips → sim scene with the same object kinds |

## Phases

| # | What | Needs | Done when |
|---|---|---|---|
| V0 | Owner films the clips; labels CSV; zero-shot baseline of a small VLM and of Glimmer (Glimmer 4-bit on Kaggle 2× T4 for inference) on events and intent | the clips | baseline scores for both models |
| V1 | `Pick` and `Place` (place planner included) in `robot_service`, the proto and MCP; open boxes in the catalog ([phases/phase_v1.md](phases/phase_v1.md)) | — | pick → place into a box ≥ 9/10, a box filled 4/4, stacking ≥ 4/5, error checks |
| V1b | The owner's real object kinds (shapes, sizes) in the catalog, size classes, the rule checker | V1, Open Question 1 | rule checker verified on every task family |
| V2 | Program runtime (format, checks, dry run, recovery handoff); rule-to-program expert; failure injection; program-writing and recovery data; SFT on the local model; sim eval of program validity, correctness and recovery | V1, V1b | the program and recovery metrics in the sim |
| V3 | Demo reading: SAM 3.1 tracking → events; VLM → events + intent (schema, prompts); synthetic hand videos only if V0 shows they are needed | V0 | event and intent accuracy on the owner's clips |
| V4 | Video → events + intent: SFT of the small VLM on synthetic clips (+ Something-Something if licensed); eval on held-out synthetic and real clips | V0, V3 | intent accuracy, sim-to-real gap |
| V5 | End to end, plus the confirmation and ask-when-unclear flow (CUJ 2, 3) | V2, V4 | end-to-end scores |
| V6 | Glimmer in the cloud: intent reading, program writing and recovery; recipe SFT, then GRPO with program success as the reward | V5 | raw vs. fine-tuned Glimmer on all metrics |
| V7 | Demo UI: one screen with the demo clip + SAM masks, events and rule, **the generated program** with a confirm step, the live Isaac viewer, the program's current line and call log (failures and recovery handoffs highlighted), the final verified checklist, and the study and results charts; recording of the 3-minute video | V5 | a full run recorded end to end from the UI |

V0 and V1 run in parallel; V2 needs V1; V3 needs the clips from V0; V7 starts with V2 and is finished last.

## Risks and Mitigations

*   **The gap between simulated hands and real video.** Mitigations:
    *   heavy randomisation;
    *   measuring the gap on real clips from the start (V0, V4);
    *   a small real fine-tuning split;
    *   Something-Something v2 if licensed.

    Glimmer's pretraining on real video should help.
*   **One demo can be ambiguous:** asking (CUJ 3) is a goal, not a failure; marked unclear clips measure it.
*   **The place planner is new code:** choosing a free spot in a partly filled box and the right release
    height. Its own checks (V1) cover full boxes, small and large objects, and stacking onto objects before
    any model training.
*   **Long executions** grow the context: with `pick`/`place`, about 15–20 calls for 6 objects.
*   **Hand animation effort:** start with a simple rigid hand with an open and a closed pose. Articulated
    fingers only if V4 shows the gap comes from the hand.
*   **Compute:** synthetic rendering uses the local GPU in the owner's off hours. VLM training on video
    frames costs more than text; Kaggle quota is 30 GPU-h per week.

## Alternatives Considered

### Alternative A: One model from video straight to tool calls

Simpler to describe, but a wrong reading of the video is only visible after the robot has moved. It cannot be
shown to the user or checked against the events, and its errors cannot be told apart from execution errors.
The typed intent costs one schema and gives a check, a confirmation step and separate metrics.

### Alternative B: Rule induction by code from the events

Given correct events, the simplest rule could be searched for by code (the attribute that separates moved
from unmoved objects). It is reliable, but it limits rules to what the search knows, and the project's aim
is a model that understands intent. Kept as the consistency check, not as the solver.

### Alternative C: Learning the hand trajectory (imitation learning)

That transfers the motion, not the intent. Our robot has its own planner and grasps. Copying a human hand
path does not carry over to new layouts or object counts.

### Alternative E: The agent drives every call live (the first version of D11)

The agent decides each call after seeing the previous reply. It adapts at every step, but it costs a model
call per step (about 20 per task), the behaviour is different on every run, and nothing can be reviewed
before the robot moves. With a program, the robot's behaviour is readable and repeatable, runs cost no model
calls, and the agent's flexibility is used only where it matters: on failures (recovery handoff).

### Alternative D: Demonstrations by teleoperating the robot

Robot data of the right shape, but people would have to drive the robot, and that is not the "show it
naturally" experience. A possible later extra source.
