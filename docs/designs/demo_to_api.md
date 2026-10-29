# From a human demonstration to reliable robot API calls

*   **Author:** mew
*   **Status:** Draft for owner review
*   **Date:** 2026-10-29

--------------------------------------------------------------------------------

## Overview

A person shows the task once, on video: they put all the cubes in a box, sort objects into boxes by shape or
by size, or put one item in a box. A model watches the video, works out the *intent* (the rule behind what the
person did, not their hand motion) and then operates the robot through the existing API so that the same
intent is carried out in a **different** scene: other positions, other numbers of objects, new object ids.

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

*   **Demonstration (demo):** a video of a person doing one task with real or simulated objects, seen from
    roughly the robot camera's viewpoint.
*   **Event:** one object movement seen in a demo: object (by attributes) → destination (box or object, by
    attributes).
*   **Intent:** the rule behind the events, in a typed form (see "Intent schema"), for example "every
    object with category cube → into the box".
*   **Executor:** the model role that turns an intent plus the robot's current scene into API calls.
*   **Transfer:** carrying out an intent in a new scene. The task is scored on the new scene only.
*   **Rule checker:** code that evaluates an intent against Isaac's true object poses (success or failure,
    per object).

## Goals

*   **G1:** Infer the intent from one demo video for these task families:
    *   put all objects of one kind in a box;
    *   sort by shape, by size or by colour into boxes;
    *   put one named item in a box;
    *   put objects on another object.
*   **G2:** Carry the intent out in a new scene through the existing API, scored by the rule checker; report
    per object what was done.
*   **G3:** Reliability:
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

## Non-Goals

*   **Imitating the hand trajectory:** the robot uses its own planner and grasps.
*   **Executing on a real robot:** execution stays in the sim. Real videos are only an *input*. Real
    execution needs the deferred perception stack (D9).
*   **Long-horizon assembly, deformable objects, tool use.**
*   **An executive or behaviour trees** (as before): the model sequences the calls itself. `pick` and
    `place` (D12) are two fixed, planner-backed calls in `robot_service`, not a skill framework.

## Owner Decisions

*   **D11 (owner, 2026-10-29):** one VLM does the whole job, in order:
    1.  It captures the intent from the video as events plus a typed rule, shown to the user.
    2.  It turns the intent into API calls in the new scene.
    3.  It watches every reply and recovers from failures.
    4.  It verifies the result before reporting.

    The typed intent sits between steps 1 and 2 (see Alternative A for the one-step variant).
*   **D12 (owner, 2026-10-29):** object-level calls `Pick(object_id)` and `Place(object_id, target_id)` in
    the API, backed by the grasp planner, the motion planner and a new place planner. The model says *what*
    and *where* ("pick obj_12", "place obj_12 in box obj_07"); the planners compute *how*. This removes the
    pose arithmetic that caused most whole-task failures in the learning run.
    *   The low-level calls stay for fine control and special cases: contact moves, insertion, jogging.
    *   `pick` and `place` make one planned attempt and never retry or improvise. On failure they return a
        clear reason, and every recovery decision belongs to the VLM, which can only learn recovery if it
        sees the failures.
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
3.  The user confirms. The sim scene has 2 cubes and 3 cylinders in other places.
4.  The robot puts both cubes in the box and checks with `list_objects`. It reports: "obj_12 (red cube) and
    obj_40 (yellow cube) are in the box obj_07; the cylinders were left as in the demo."

### CUJ 2: Correct the intent before execution

The model says "every red object → box". The user replies "no, every cube". The model updates the intent and
runs.

### CUJ 3: Unclear or impossible

*   **Unclear demo:** the person put only the red cube in the box, and the scene has no other red objects.
    The model asks: "only the red cube, or all cubes?"
*   **Impossible:** the new scene has no box. The model says so and does not move.

### CUJ 4: Recovery during execution

`place` fails because the box has no free collision-free spot left, or a `pick` fails because the grasp is
blocked by a neighbouring object. The executor reacts to the error message: it moves the blocking object
first, uses another box that matches the rule, or reports that the box is full. It does not give up after
the first error.

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

### Executor (intent → API calls)

The "B" approach from the learning run (SFT with the published recipe), with its three gaps closed:

1.  `pick` and `place` instead of computing poses by hand.
2.  Training sequences with a short reasoning line before each decision (which object next, which box, why),
    and recoveries: after a failed `pick` or `place`, act on its reason (clear the blocking object, use
    another matching box, retry once after a timeout, or report).
3.  Every sequence ends with `list_objects` and a per-object report checked against the intent.

Stage-1 single-call data and its 30 % replay are kept.

The whole-task data comes from two sources:

*   **A rule-following scripted expert:** it expands a rule into per-object `pick` + `place`, with
    randomly injected failures and their recoveries.
*   **A strong hosted model as teacher**, keeping only episodes the rule checker passes.

### Synthetic demo videos (Isaac)

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

### Evaluation

| Metric | On |
|---|---|
| Event accuracy (object and destination per event) | held-out synthetic demos, real clips |
| Intent accuracy (the rule selects the same objects as the true rule in the new scene) | same |
| Asks when the demo is ambiguous / refuses when impossible | marked clips |
| Task success in the new scene (rule checker, per object and whole task) | sim, held-out layouts |
| Wrong-call rate, recovery rate, verified final answer | sim executions |
| End to end: demo → intent → execution | synthetic demos; real clips → sim scene with the same object kinds |

## Phases

| # | What | Needs | Done when |
|---|---|---|---|
| V0 | Owner films the clips; labels CSV; zero-shot baseline of a small VLM and of Glimmer (Glimmer 4-bit on Kaggle 2× T4 for inference) on events and intent | the clips | baseline scores for both models |
| V1 | Catalog (boxes, shapes, sizes), size classes, rule checker; `Pick` and `Place` (place planner included) in `robot_service`, the proto and MCP; checks updated | — | `pick`/`place` checks pass (e.g. ≥ 9/10 per object kind into a box), rule checker verified |
| V2 | Executor: intent schema, rule-following expert with failure injection and recovery, reasoning lines, final verify; SFT; sim eval on the task families | V1 | executor success and reliability metrics in the sim |
| V3 | Synthetic demo videos: hand asset and animation, renderer, randomisation, labels | V1 | a few thousand labelled clips, spot-checked |
| V4 | Video → events + intent: SFT of the small VLM on synthetic clips (+ Something-Something if licensed); eval on held-out synthetic and real clips | V0, V3 | intent accuracy, sim-to-real gap |
| V5 | End to end, plus the confirmation and ask-when-unclear flow (CUJ 2, 3) | V2, V4 | end-to-end scores |
| V6 | Glimmer in the cloud, both steps, recipe SFT, then GRPO on whole episodes | V5 | Glimmer scores on all metrics |

V0, V1 and V3 can run in parallel; V2 needs V1.

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

### Alternative D: Demonstrations by teleoperating the robot

Robot data of the right shape, but people would have to drive the robot, and that is not the "show it
naturally" experience. A possible later extra source.
