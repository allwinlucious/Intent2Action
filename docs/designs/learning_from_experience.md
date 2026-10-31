# Learning from experience: the robot's planner and physics teach the model

Status: approved plan (2026-10-31). Builds on [demo_to_api.md](demo_to_api.md) (program runtime, recovery, rule
checker) and the hard-task finding: a program that is correct by the API docs fails on the robot because of what the
robot can physically do (grasp access, reach, free space). Any capable model writes the docs-level program; only
experience on this robot teaches which program works. Glimmer is the only model in the product; it learns from graded
episodes, never from a bigger model.

## The loop

1. A task generator draws a layout from a hard family (below) and an intent.
2. Glimmer writes the program; on failures it is the recovery agent (handoffs), exactly as in the product.
3. A tier plays it out and the rule checker grades it from ground truth.
4. Graded episodes plus expert episodes become training data; Glimmer is fine-tuned (QLoRA SFT); back to 1.

## Two tiers

| Tier | Runs | Speed (target) | Catches |
|---|---|---|---|
| **plan** | The container with `ROBOT_BACKEND=plan`: no Isaac, no ICON. The same robot_service Pick/Place code; trajectories are planned for real and the arm jumps to their end; the gripper closes instantly to the object's width; a released object drops to its support. | ≤ 10 s per 5-cube episode | Grasp access, pick order, reach and singular configurations, collisions, free space in a box |
| **sim** | Isaac, as today (headless, `--physics-only`) | ~60 s per episode | Physics: slips, toppling, objects disturbed or settling |

The plan tier makes most of the data; the sim tier gives the final numbers, the videos and physics-only failures.
**Tier agreement** is measured: the same episodes in both tiers; the report states how often "plan OK, sim failed".

## The expert (a search script, not a model)

Python with privileged access, in the plan tier only: ground truth and unlimited planner queries.
- Program: greedy-with-backtracking over pick order. Among the remaining objects it asks `PlanGrasp` + the place
  feasibility for each, takes a feasible one (most free sides first, then the shortest motion), applies it, repeats;
  when none is feasible it backtracks. Output: the program in the runtime's format with the order it found.
- Recovery: from a failure state (a naive program run until its first failure), it searches the decision that makes
  the failed step feasible again (which neighbour or opposite pair to clear, a different order, skip) and writes the
  decision JSON of the recovery protocol.
It is a teacher only: it needs ground truth and many planner queries, and it cannot read a demonstration.

## Task families

Randomised layouts with a seed; train seeds and held-out seeds per family, plus one family held out entirely.

| Family | Layout | What it tests |
|---|---|---|
| cluster | plus / L / line / ring of 4–7 cubes, gaps 1.0–3.0 cm, random rotation | pick order from grasp access |
| packing | a box filled to 60–100 % of its spots, more cubes to add | free space, release poses, "box full" |
| stacking | towers of 2–4 to build or take apart | order, on-top placement |
| reach | objects near the workspace edge or behind others | reach, singular configurations, approach |
| mixed (held out) | a sort-by-colour or by-size intent over a cluster plus a near-full box | generalisation |

## Training data

- Expert episodes: program-writing examples (intent + scene → expert program) and recovery examples (transcript up
  to a failure → expert decision).
- Glimmer's own episodes: k samples per task (temperature 0.7) in the plan tier; the ones the rule checker passes are
  kept (rejection sampling); failures paired with an expert fix on the same scene are kept for DPO.
- Formatted with Glimmer's own chat template (tool-call arguments as dicts), pre-tokenized in Python.
- Target: ~1500 SFT examples, ≤ 30 % recovery.

## Training and serving

QLoRA SFT with the model cookbook's hyperparameters (r 32, alpha 16, dropout 0.05, attention + MLP projections of
the language model, lr 2e-4 cosine, warmup 0.1, micro-batch 1 × 4, seq 4096, 8-bit AdamW) using TRL on one H100
on Modal. Serving: the same vLLM endpoint with the LoRA adapter (`--enable-lora`); if the Glimmer image does not
support LoRA, the merged weights are served instead. DPO is optional, after SFT.

## Model versions

Each step is a separate LoRA adapter on the same base, all served by one vLLM endpoint (`--lora-modules`), so
every version runs the identical evaluation and each step's gain is measured on its own.

| Version | Trained on | Question it answers |
|---|---|---|
| G0 | nothing (zero-shot, the same prompts) | the baseline: what a capable model does from the API docs |
| G1 | expert program-writing episodes | does learning the robot's limits from the planner help? |
| G2 | G1 data + Glimmer's own successful episodes (rejection sampling) | does self-generated experience add to the expert? |
| G3 | G2 data + recovery episodes (expert decisions after failures) | does recovery improve, and does it hurt program writing? |
| G4 | G3 data + sim-tier episodes | does physics experience add over planning experience? |
| G5 (optional) | G3/G4 + DPO on failure/fix pairs | do preference pairs fix the remaining mistakes? |

## Evaluation

On held-out seeds of every family and the held-out family: every version, same prompts, same tasks.
- plan tier: 50 tasks per family; sim tier: 20 tasks in total; 3 rendered videos (before/after on the same layout).
- Metrics: task success (rule checker), first-program success, recovery success, handoffs, repeated failing calls,
  protocol errors, robot calls, time; tier agreement.
- Per version, per family, with 95 % confidence intervals (Wilson); a failure breakdown by kind (grasp, reach,
  collision, place, verify, protocol); learning curves (success vs number of training examples, from G2's data
  at 25 / 50 / 100 %); program-writing quality kept on the simple tasks of `glimmer_program_test.py` (no
  regression); generalisation gap train-family vs held-out-family.

## Phases

| Phase | What | Depends on |
|---|---|---|
| L0 | Recovery agent: always ends with a valid decision (final turn forced to JSON); invalid decisions log the reply; fill_box FinePathIK investigated | — |
| L1 | Plan tier: `ROBOT_BACKEND=plan` (PlanIcon, PlanClock, a world-backed SimService/SimLink stand-in, drop-to-support on release); `pickplace_check` passes in it | — |
| L2 | Task families: generator, intents, seeds and splits; layouts load into both tiers | — |
| L3 | Expert: program and recovery search; ≥ 95 % of generated tasks solved in the plan tier | L1, L2 |
| L4 | Data generation: Glimmer samples + expert episodes, graded, formatted, pre-tokenized | L3 |
| L5 | Fine-tuning on Modal and serving the adapter | L4 |
| L6 | Evaluation and the report (`docs/designs/learning_results.md`) with videos | L5 |
| L7 | DPO (optional) | L5 |

L0, L1 and L2 run in parallel. Compute: plan-tier and sim runs on the local machine after midnight; Glimmer sampling,
training and serving on Modal.

## Risks

- The plan tier misses physics: measured by tier agreement; physics-only failures come from the sim tier.
- LoRA serving in the Glimmer vLLM image: fall back to merged weights.
- The expert covers only the families above: Glimmer's own graded episodes cover the rest.
