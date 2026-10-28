# Teaching a small model to use the robot: raw vs. fine-tuned vs. action head

*   **Author:** mew
*   **Status:** Draft for owner review
*   **Date:** 2026-10-28

## Goal

Measure whether a small local model (Qwen3 8B, on the RTX 3080) can learn to
call the robot API with fewer mistakes, and compare three versions of it
on the same tasks:

| Version | What changes |
|---|---|
| **A. Raw** | Qwen3 8B as is, calling the MCP tools |
| **B. Fine-tuned** | QLoRA adapters trained on what the controller accepts and rejects (stage 1), then on whole task sequences from the sim (stage 2), without forgetting stage 1 |
| **C. Action head** | Qwen3 8B frozen; a small trained head picks the tool and regresses its parameters (stage 1 + 2 data). Built in a separate git worktree |

Reference point: a frontier hosted model on the same tasks (5/5 on CUJ 1, about 1
rejected call per run).

## Two stages: choose locally, train the target in the cloud

1.  **Choose the approach locally** with Qwen3 8B on the RTX 3080 (this
    document's phases L0–L6): raw vs. fine-tuned vs. action head.
2.  **Train the target model in the cloud** with the chosen approach:
    **Muse Glimmer** (30B, open weights, Apache 2.0; owner requirement),
    on a rented GPU (≥ 24 GB for 4-bit inference, more for training).

So everything built here must be model-agnostic and portable:
*   The agent loop, recorder, task suite, checkers and datasets do not
    depend on the model. Swapping in Glimmer changes only the model
    loader and its chat or tool-call template.
*   Training scripts take the base model as a parameter, and run the same
    on the local GPU and on a cloud GPU (containerised, no paths tied to
    this machine).
*   What does not transfer: the LoRA adapters and the action head's
    weights (Glimmer's hidden size differs). The data, code and
    hyperparameters found on Qwen transfer; the weights are retrained on
    Glimmer.
*   The sim (Isaac + container) stays on this machine. Cloud training
    uses recorded data; cloud evaluation connects to the robot stack here
    over the network, or the stack is deployed next to it (decided when
    we get there).
*   Glimmer has a vision encoder, so images can come back into the loop
    after the text-only comparison.

## What "a mistake" means (the metrics)

Every run is scored automatically:

| Metric | Definition |
|---|---|
| **Task success** | ground-truth check in Isaac (e.g. cube on plate), as `phase6_cuj1.py` does |
| **Error-call rate** | calls that return a non-OK status / all calls |
| **Invalid-argument rate** | calls rejected as `INVALID_ARGUMENT`, `NOT_FOUND`, or schema errors / all calls |
| **Calls per success** | efficiency |
| **Recovery rate** | after an error, does the next call fix it? |
| **Stage-1 call validity** | offline benchmark (no sim): given a situation and an intent, does the produced call succeed? Used to detect forgetting |

## Object identity (owner decision D10)

All tasks and training data use perception-style objects: neutral IDs
(`obj_<k>`, random per episode) with attributes (`category`, `color`, size,
pose). Prompts describe objects ("the red cube"); the model must ground
them to IDs itself: it calls `list_objects` and picks the ID whose
attributes match (there is no `find_objects`). This needs the API change
below (phase L−1, [phases/phase_ids.md](phases/phase_ids.md)).

## The building blocks

### 1. Physics-only sim (no rendering, no vision)

*   `run_scene.py --physics-only`: renderer disabled at start-up, no camera
    graphs. Physics already runs on the CPU, so Isaac should need little or no GPU
    memory and the model gets about 9 GB.
*   The MCP server gets a flag that hides `get_image`, `detect_objects`,
    `calibrate_camera`. The model knows the world only through
    `list_objects` and `get_state` (ground truth from the sim).
*   To verify: Isaac's scene services (spawn, reset) still work without
    rendering, and the actual GPU use.

### 2. One agent loop for every model

A small Python agent (`agent/`) that connects a model to the MCP server:
system prompt = the MCP server's instructions, tools = the MCP tool list,
loop: model → tool call → result → model, until the model stops or 40 calls.
The same loop runs A, B, C and the frontier model, so the only difference between
versions is the model. Everything is recorded per episode (JSONL): prompt,
every call, arguments, result, status, timing, final success. This record
format is also the training data format.

*   Inference for A and B: Hugging Face `transformers` + `bitsandbytes` 4-bit
    (same numerics as training; the LoRA adapter loads on top). Ollama is
    faster for A alone but cannot run C or B's adapter the same way, so it
    is only used for a quick first look.
*   Qwen3 "thinking" mode: on for A (its best behaviour), and the same
    setting kept for B and C. **Owner question 2.**

### 3. Task suite

Generated from templates over randomized scenes (positions and yaw of the
catalog objects within the table area, fixed seeds):

| Family | Example prompt | Success check |
|---|---|---|
| Pick and place | "Put the red cube on the plate." | cube on plate |
| Move | "Move the gripper 10 cm above the peg." | TCP within 1 cm |
| Grasp and lift | "Lift the peg 5 cm." | peg z rose ≥ 4 cm |
| Insert | "Put the peg in the hole of the grey block." | peg depth ≥ 25 mm |
| Recover | the scene starts with a fault | fault cleared, task done |
| Infeasible | "Pick up the plate." (too wide) | model reports it cannot, no crash |

*   **Train set**: 300 tasks; **held-out eval set**: 60 tasks (different
    seeds and some unseen phrasings); each eval task run 3 times.

### 4. Stage-1 data: what the controller accepts and rejects (no sim needed)

The owner's point: single-call validity does not need physics. The stack
answers it with the **loopback backend** (Intrinsic planner + ICON on the
loopback hardware module, no Isaac):

*   A new container mode `ROBOT_BACKEND=dryrun`: loopback ICON +
    `robot_service`, with the scene loaded into Intrinsic's world from
    `sim/scenes/default.yaml` (randomized per sample) instead of from Isaac.
    The planner, IK, collision checks and every argument check are real, so
    the replies are exactly what the robot would say.
*   A **call generator** samples situations (robot state, object poses) and
    calls for every tool: valid ones, and systematically invalid ones
    (speeds outside the force-dependent range, unreachable or colliding
    poses, unknown objects, wrong order such as `release` before `grasp`,
    motions while faulted). Each is sent to the dry-run stack; the reply
    (status + message) is recorded.
*   Training examples built from that:
    1.  *Situation + intent → a call that succeeds* (positive).
    2.  *Situation + failing call + error reply → corrected call* (teaches
        recovery from the messages the stack gives).
    3.  *Situation + intent that cannot be done → explanation* (teaches
        saying "no").
*   Size: about 20 000 calls (tens of calls per second without physics, so
    hours, not days).
*   Also gives the **stage-1 call-validity benchmark** (2 000 held-out
    samples).

### 5. Stage-2 data: sequences in the sim

*   Physics-only Isaac runs the train tasks.
*   Trajectories come from two sources: the **coding agent** (`coding-agent`,
    already installed and logged in, run headless with this MCP server as
    its only tool source; the teacher, about 3 runs per task) and the
    model's own rollouts after stage 1, keeping only successful, low-error
    episodes (rejection sampling). The agent's transcripts are converted to
    the recorder's JSONL format.
*   About 1 000 episodes. At about 1 minute each without rendering, roughly
    one to two days of unattended sim time.

## The three versions in detail

### A. Raw Qwen3 8B

Run the eval suite through the agent loop. Report the metrics. This is the
first trial.

### B. QLoRA fine-tune

*   4-bit base, LoRA rank 16 on attention + MLP projections, sequence length
    4 096, gradient checkpointing (Unsloth or PEFT); fits in 10 GB with Isaac
    off. Training on the 3080: hours per stage.
*   **Stage 1**: train on the stage-1 examples in Qwen3's chat and tool-call
    format.
*   **Stage 2**: continue on the sim episodes **mixed with 30 % stage-1
    examples** (replay) so stage-1 skills are not forgotten; check the
    stage-1 benchmark before and after.
*   Evaluate after each stage → B1, B2.

### C. Action head (separate worktree `../untitled-project-ah`)

*   Qwen3 8B frozen (4-bit). At each step the conversation so far is run
    through it, and the last layer's hidden state at the end of the context
    goes into the head.
*   The head (a small MLP, a few million parameters) outputs:
    *   **which tool** (classification over the ~25 tools, plus "finished"
        and "cannot do");
    *   **numeric parameters** (positions, quaternions, forces, speeds,
        joint values) by regression, squashed into each parameter's valid
        range (so out-of-range values are impossible by construction);
    *   **choices** (enums such as `LINEAR`/`ANY`) by classification;
    *   **object IDs** by pointing at one of the objects the model has
        already seen in a `list_objects` result.
*   Training: hidden states are computed once and cached, then the head
    trains in minutes per epoch. Same stage-1 and stage-2 data, same replay
    rule.
*   The head emits the call; the agent loop sends it like any other.
    Free-text answers (e.g. "cannot do, plate too wide") come from the frozen
    model when the head chooses "finished" or "cannot do".
*   Evaluate → C1, C2.

## The comparison

One report (`docs/designs/model_learning_results.md`): a table of every
metric for A, B1, B2, C1, C2 and the frontier model on the held-out suite and the
stage-1 benchmark, per task family. It also covers typical failure examples
per version, training cost (GPU hours), inference latency per call, and
memory.

## Phases

| # | What | Needs sim? | Done when |
|---|---|---|---|
| L−1 | API change for D10: `list_objects` returns IDs + attributes (`category`, `color`, size, pose), every object argument takes an ID, scene_sync assigns random IDs in sim; all checks updated | yes | phase 3–6 checks pass with IDs |
| L0 | Physics-only mode, MCP vision filter, agent loop + recorder, task suite + checkers, the frontier model reference run | yes | the frontier model scores on the held-out suite; GPU use measured |
| L1 | **A: raw Qwen3 8B** | yes | metrics for A |
| L2 | Dry-run backend + call generator + stage-1 dataset and benchmark | no | ~20 000 recorded calls; benchmark scores for A and the frontier model |
| L3 | B stage 1 (QLoRA) | no (train) / yes (eval) | B1 metrics |
| L4 | Stage-2 episodes + B stage 2 with replay | yes | B2 metrics; stage-1 benchmark not worse than B1 |
| L5 | C in the second worktree, stage 1 and 2 | no / yes | C1, C2 metrics |
| L6 | Comparison report | — | report written |

L5 can start right after L2 (it shares the data), in parallel with L3/L4,
since it lives in its own worktree. GPU use must still be taken in turns:
one training or one sim run at a time on the 3080.

## Owner questions

1.  **Inference runtime:** HF `transformers` 4-bit for all versions (fair
    comparison, slower), or Ollama for A and `transformers` for B/C?
2.  **Qwen3 thinking mode:** on or off? On usually helps tool use but costs
    tokens and time per step.
3.  **Teacher:** decided: the coding agent produces the stage-2 teacher
    trajectories. Open: which model it uses as teacher.
4.  **Action head output for free text:** is "the frozen model writes the
    text when the head says finished/cannot do" right, or should C be
    calls-only?
5.  **Budget:** the plan assumes everything on the 3080 (hours per training
    stage, one to two days of sim for stage 2). Is that acceptable, or do you want
    cloud GPUs for faster iteration?

## Risks

*   Qwen3 8B at 4-bit may score near zero on long tasks at first. The stage-1
    benchmark still shows progress on single calls.
*   Physics-only Isaac may still need some GPU for its renderer components;
    measured in L0.
*   The action head cannot easily handle tools whose arguments depend on
    long-range reasoning (e.g. computing a place pose from two objects); it
    learns only what the hidden state encodes. That is part of what the
    comparison measures.
*   Stage-2 data from the frontier model carries the frontier model's habits (e.g. probing with
    `move_to_contact`), which may or may not suit a small model.
