# Results: raw vs. fine-tuned vs. action head (Qwen3 8B, local selection run)

*   **Branch:** `throwaway/learning` (worktree `~/untitled-project-learn`); plan: [model_learning.md](model_learning.md), not reviewed
*   **Date:** 2026-10-29
*   **Raw tables:** `logs/learn/report.md`; every episode and benchmark case: `logs/learn/*.jsonl`

## Summary

| Version | Single calls: first try | Recovery after an error | Refuses infeasible | **Sim tasks (30)** | Wrong calls in sim |
|---|---|---|---|---|---|
| Scripted expert (reference) | — | — | — | **29/30** | 2 % |
| A. Raw Qwen3 8B | 70 % | 53 % | 29 % | **9/30** | 14 % |
| B1. Recipe SFT, stage 1 | **89 %** | **95 %** | 100 % | 4/30 | **4 %** |
| B2. B1 + stage 2 (30 % replay) | 88 % | 95 % | 95 % | **10/30** | 6 % |
| C1. Frozen Qwen + action head, stage 1 | 31 % | 25 % | 100 % | 4/30 | 14 % |
| C2. C1 + stage 2 | 33 % | 27 % | 100 % | 4/30 | 20 % |

a frontier hosted model (earlier, CUJ 1 only): 5/5 pick-place with about one rejected call per run.

**Recommendation: approach B (fine-tune the language model with the recipe's SFT) for the Glimmer phase,
with stage-2 data that covers whole tasks much more densely. Drop the action head.**

## What each stage did

*   **Stage-1 fine-tuning fixes call mistakes.** B1 makes far fewer argument errors (4 % wrong calls in
    the sim against 14 %) and recovers from 95 % of stack errors. Qwen's typical errors are gone: the identity
    quaternion for "pointing down", `compute_ik` instead of moving, unit and key mix-ups. This is the
    problem the project set out to solve, and SFT on the stack's own replies solves it.
*   **Stage 1 alone also teaches giving up.** Stage-1 examples often end with a failing call followed by
    "I can't do that: …". B1 generalised that to any setback ("I'm already holding the object", "stuck in a
    loop"), so its sim success fell from 9 to 4. Single-call data must not be the only data.
*   **Stage 2 restores task-level behaviour without losing stage 1.** B2 keeps B1's single-call scores
    (88 / 95 / 95 %, replay works) and reaches 10/30. Pick-place goes from 0/8 to 3/8 and lift is 5/5.
*   **B2 learned the pattern but not all of it:**
    *   It places the cube exactly over the target (0–2 mm off in x/y) but often skips the lowering step:
        five runs left the cube 10 cm above the plate or the lower cube.
    *   It treats "10 cm above the top of the cube" as z = 0.10 m, a consistent 51 mm error in all 5 move
        tasks.
    *   It claims success without checking ("The peg is in the hole" with the peg on the table).
    *   On "Put the orange cube on …", where no orange cube exists, it picked a different cube twice
        instead of refusing.

    These are signs of too little stage-2 data: 400 turns from 200 expert episodes, one epoch, and a single
    rigid way of doing each task.
*   **The action head (C) does not work for this API.**
    *   It picks plausible tools, but regressed numbers are not precise enough for poses: speed 0.991
        instead of 1.0, a mass of 0.174 instead of 0.2, positions off by centimetres. That gives a first-try
        score of 31 % against the raw model's 70 %.
    *   The arguments here are mostly copied from context (ids, poses from `plan_grasp` and
        `list_objects`), which a language model does exactly and a regression head cannot.
    *   C1 looped until the 40-call limit in 25 of 30 tasks; C2 stops after 2–4 calls with "Done."
    *   C would need copy/pointer mechanisms for every numeric argument, at which point it is a decoder
        again.

## Insights for the Glimmer phase

1.  **Data, not method, limits B2.** Stage 2 needs many more whole-task episodes, each with a varied way of
    doing the task and with recoveries. A scripted expert gives one rigid sequence. A strong model as teacher
    (e.g. a hosted model in the agent loop, keeping only successful episodes) or the trained model's own
    successful rollouts would give that variety.
2.  **Train "check, then claim".** Add a final `list_objects` / `get_state` before the answer to the
    stage-2 sequences, and reward-check the claim. False success claims are the most harmful failure for a
    VLM operator.
3.  **Balance refusals.** Stage-1 refusals should only follow errors that mean "impossible" (no such
    object, wider than the gripper, unreachable), never transient ones (timeouts, "already holding").
4.  **Episode-level GRPO in the sim is the natural next step** once SFT gives a reasonable starting point:
    task success from the ground-truth checks is the reward. It needs the sim and the model side by side for
    hours, so a cloud machine with an RTX-class GPU (L40S) or the H100s.
5.  **The recipe's full settings are a cloud job.** Each training example is about 6.5k tokens (system
    prompt plus 27 tool schemas) for about 25 trained tokens:
    *   SFT took about 33 s per example on the RTX 3080 and 27 s on a Kaggle T4.
    *   The recipe's 3 epochs over all data, DPO and GRPO need H100s.
    *   Shortening the tool schemas would cut cost everywhere.

## How it was run (deviations from the plan)

*   **Sim:** Isaac physics-only (no camera, 1.7 GB of GPU), vision tools hidden; 3 more cube colours in the
    catalog so that grounding by attributes matters.
*   **Stage-1 data:** 2 500 single-call situations recorded against the real stack, not a dry-run
    container (`robot_service` does not run in loopback mode).
*   **Stage-2 data:** 200 scripted-expert episodes (no model calls): 191 succeeded, and the 182 without
    any failed call were used.
*   **Eval:** 30 held-out tasks, run once each, greedy decoding, thinking off. With 30 tasks, differences
    of 1–3 tasks are noise; the large effects above are not.
*   **Training:** TRL with the recipe's settings, on less data, on Kaggle 2× T4 (fp16):
    *   QLoRA nf4, r 16, alpha 32, dropout 0.05, attention projections, lr 2e-4, effective batch 16.
    *   SFT on 600 examples, 1 epoch (K1, 7.2 h including head features). Stage 2: 400 turns, 1 epoch
        (K2, 3.9 h).
    *   Kaggle GPU quota used: 11.6 of 30 h.
*   **Not run locally:** DPO (225 s per pair; the recipe's lr gives too few steps at affordable sizes) and
    GRPO (500 s per step at 19 GiB; single-call rollouts came out identical, so there was no reward
    variance). The code is tested and ready for the cloud.
