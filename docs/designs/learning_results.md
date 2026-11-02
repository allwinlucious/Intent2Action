# Learning from experience: results

Running report of [learning_from_experience.md](learning_from_experience.md). Every number below is from the plan
tier (real planning, no physics), on the **held-out** seeds: 50 tasks per family, 250 in total, the family `mixed`
never seen in training. Success = the program runs to the end and the rule checker passes on ground truth.
Generation follows the model cookbook's adapter evaluation: transformers + PEFT on an NF4 base (G0 = the same base,
no adapter), greedy, one attempt, the product's writer prompt. 95 % Wilson intervals in brackets.


## Robot fix found by the review, measured in physics (2026-11-02, night)

The review found that a held object's collisions with **its original support stayed disabled until placement**, so
a carry could be planned through the table (or the cube below). In Isaac this showed up as the "arm lag" stalls:
the held cube hit the table and the arm jammed. Fixed (supports collide again right after the lift); the corrective
move after a stall now settles the arm first (ICON accepts a trajectory only near its last setpoint).
- The 19 stacking training tasks whose label failed in physics: **16/19 now succeed** (before: 0/19). The other 3
  fail cleanly: the arm does not settle even after a stop (a real obstruction).
- `pickplace_check` all pass; `program_check` 8/9 (one place started with the held cube touching another cube; to
  investigate).
- G1b on the 50 held-out Isaac tasks: **41/50 (82 %)** (before the fix 40/50); plan/physics agreement 86 %.
- Same 50 tasks with the fixed robot: docs-level program **35/50**, G0 **34/50**, G1b **41/50**
  (stacking 6 / 6 / 8, cluster 5 / 5 / 8). Fine-tuning's gain holds in physics, smaller than in the plan tier.
The physics labels for G4 were measured before this fix, so G4's stacking relabelling was based on the bug.

## Corrections after an independent code review (2026-11-01, evening)

Three the reviewer reviewers (training, evaluation, robot code; read-only) found defects that change reported results:

1. **GRPO runs G6, G7, G7b are invalid.** Generation never stopped at Glimmer's end-of-turn token `<|eot|>`; every
   rollout ran to the 700-token limit (G6: 100 % clipped), so the reward scored the program plus arbitrary
   continuation. "GRPO leaves behaviour unchanged" (G6) and "GRPO diverges" (G7) are withdrawn; G7b was stopped.
   Fixed (`generation_kwargs` with `<|eot|>`); a corrected GRPO run is pending.
2. **Live recovery: G0 had 4 invalid decisions, not 0.** A `patch` decision without a patch passed validation and
   was rejected later by the runtime (no corrective turn). Fixed (patch and ask_user need their payload); the G0
   recovery run is to be repeated.
3. **The intervals were not valid.** Run means were rounded into binomial counts. Recomputed below as a percentile
   bootstrap over tasks. And the old "best strategy 91 %" is an **oracle** (a task counts if any of 4-5 strategies
   succeeded); the fair reference, one fixed strategy per family chosen on the training tasks, reaches **84 %**.

Current held-out results (plan tier, 250 tasks, 3 runs per program; a task's success is its run mean; 95 %
bootstrap intervals over tasks; references from single runs):

| family | DOCS | G0 | G1b | G6 | G1b_lora | G4 | docs-level (1 run) | fixed best strategy (1 run) | oracle |
|---|---|---|---|---|---|---|---|---|---|
| cluster | 47 % [37–59] | 38 % [29–49] | 65 % [53–78] | 59 % [47–72] | 64 % [53–76] | 55 % [42–67] | 52 % | 60 % | 74 % |
| mixed | 62 % [51–73] | 51 % [39–63] | 82 % [73–91] | 85 % [76–93] | 85 % [76–93] | 82 % [72–91] | 66 % | 78 % | 86 % |
| packing | 99 % [98–100] | 87 % [78–95] | 98 % [95–100] | 99 % [98–100] | 100 % [100–100] | 99 % [98–100] | 100 % | 100 % | 100 % |
| reach | 84 % [76–91] | 83 % [75–89] | 85 % [77–93] | 86 % [78–93] | 85 % [77–92] | 79 % [71–86] | 80 % | 84 % | 94 % |
| stacking | 47 % [35–59] | 47 % [35–59] | 97 % [94–100] | 98 % [95–100] | 61 % [49–73] | 99 % [98–100] | 48 % | 100 % | 100 % |
| all | 68 % [63–73] | 61 % [56–67] | 86 % [82–89] | 86 % [82–89] | 79 % [75–84] | 83 % [79–87] | 69 % | 84 % | 91 % |

**Main claim:** fine-tuning (G1b) lifts Glimmer from 61 % to 86 % — as good as the best fixed strategy per family
(84 %), learned from the scene description and the intent alone. Choosing per layout (the 91 % oracle) is not
learned. In Isaac physics G1b reaches 80 % on 50 held-out tasks (G4 82 %).

**Corrected GRPO (G8).** Rollouts now stop at `<|eot|>` (mean completion 321-406 tokens, 12-25 % clipped instead
of 100 %); G1b merged into the base and a fresh adapter trained with a KL term (β 0.04) to it, lr 1e-5, all 302
prompts, 4 samples each, verified rewards (plan-tier outcome of the strategy a sample reproduces; conflicting
outcomes averaged). Mean reward by quarter 0.80 / 0.76 / 0.84 / 0.82 (stable, no rise). G8 writes **the identical
program to G1b on all 250 held-out tasks**: 85 % [81-88] in the plan tier (G1b 86 %), 38/50 in Isaac (G1b 40/50).
GRPO in this setting does not change the learned task-type → strategy map. In 203 of the 302 groups (67 %) all four
samples earned the same reward, so those steps carry no learning signal; the remaining third did not move the policy
within one epoch at this learning rate and KL weight. An SFT model this certain samples too little diversity for
group-relative rewards; more samples per prompt, a higher temperature or fewer SFT epochs are the levers to try. (A first grading of G8 had 87 failures from one
fleet container whose world service degraded — "UpdateTransform: Too many inaccurate links"; re-graded on fresh
containers.)

Live recovery after the validation fix (G0 rerun, same 20 tasks): **G0 rescues 4/20** (32 handoffs: reobserve 8,
skip 8, patch 10, retry 3, stop 2, fix_world 1; 1 decision still invalid after its corrective turn); **G1d rescues
9/20** (25 handoffs, all patches, none invalid; unaffected by the fix). Small sample, consistent direction.

Known limitations still open (reported, not fixed): the corrective move after an arm-lag stall can also push
through a real obstruction; the retreat is planned from the commanded release joints rather than the sensed ones;
SFT train/validation are split per row, not per task (validation loss is optimistic; versions are not exactly
nested); the DPO validation pairs' labels used strategy rates that included those tasks; strategy programs skip a
held cube that is already geometrically inside its box. Earlier sections below keep their original single-run
numbers and Wilson intervals for the record; read them with these corrections.
## Training data (600 train tasks, every strategy run on each)

| Family | listed (docs) | most_free_first | outer_first | nearest_target_first | top_first | any |
|---|---|---|---|---|---|---|
| cluster | 55 % | **77 %** | 73 % | 39 % | – | 86 % |
| packing | 99 % | 99 % | 98 % | 99 % | – | 100 % |
| reach | 74 % | 79 % | **81 %** | 79 % | – | 91 % |
| stacking | 46 % | 35 % | 41 % | 41 % | **97 %** | 98 % |

Label per task: among the strategies that succeeded on it, the most reliable one on its family.

## Held-out results (2026-11-01)

| Family | docs-level program | G0 zero-shot | G1 (expert programs) | best strategy |
|---|---|---|---|---|
| cluster | 52 % | 44 % [31–58] | **62 %** [48–74] | 74 % |
| packing | 100 % | 88 % [76–94] | **100 %** [93–100] | 100 % |
| reach | 80 % | 82 % [69–90] | **88 %** [76–94] | 94 % |
| stacking | 48 % | 54 % [40–67] | **88 %** [76–94] | 100 % |
| mixed (held-out family) | 66 % | **58 %** [44–71] | 48 % [35–61] | 86 % |
| all | 69 % | 65 % [59–71] | **77 %** [72–82] | 91 % |

Failure kinds — G0: grasp 53, reachability 15, collision 9, static check / dry run 9, place 1. G1: grasp 42,
reachability 9, rules 3, place 3 (no program failed the static checks).

**Finding (mixed).** G1 never saw a two-box task. It writes one loop per colour (all red cubes, then all blue) with
the nearest-target order; in a mixed red/blue cluster a red cube boxed in by blue neighbours then has no grasp. The
expert program handles both rules in one work list and frees grasps across colours. G1 learned a per-family habit
that does not carry over to a new task structure. Next: a two-box training family with different layouts, so the
held-out family tests the cross-rule ordering rather than the bare presence of two boxes.

## G2 and G3: plain rejection sampling brings the docs-level habit back

| family | docs-level | G0 | G1 | G2 | G3 | best strategy |
|---|---|---|---|---|---|---|
| cluster | 52 % | 44 % [31–58] | **62 %** [48–74] | 46 % [33–60] | 50 % [37–63] | 74 % |
| mixed | 66 % | 58 % [44–71] | 48 % [35–61] | 56 % [42–69] | 54 % [40–67] | 86 % |
| packing | 100 % | 88 % [76–94] | 100 % [93–100] | 100 % [93–100] | 100 % [93–100] | 100 % |
| reach | 80 % | 82 % [69–90] | 88 % [76–94] | 88 % [76–94] | 80 % [67–89] | 94 % |
| stacking | 48 % | 54 % [40–67] | **88 %** [76–94] | 54 % [40–67] | 50 % [37–63] | 100 % |
| all | 69 % | 65 % [59–71] | **77 %** [72–82] | 69 % [63–74] | 67 % [61–72] | 91 % |

G2 = G1 + 903 of Glimmer's own successful samples (4 per task at temperature 0.7, 1652 of 2400 succeeded, at most
2 kept per task); G3 = G2 + 41 confirmed recovery rows (sequence length 8192: the recovery prompt carries the tool
list). Both lose most of G1's gain. Glimmer's successes are mostly the listed order working on easy layouts; 903 of
them next to 562 expert rows teach the docs-level habit again (stacking 88 % → 54 %). Rejection sampling on a binary
outcome reinforces lucky behaviour. Only 239 of the 1652 successes are on tasks where the docs-level program fails,
i.e. where success needed a deliberate order. Follow-ups: G2b (G1 + those informative successes only), G3b (G1 +
recovery rows only), G1b (G1 + a two-box sorting family, for the held-out mixed family).

## G1b: a two-box training family fixes the held-out family

| family | docs-level | G0 | G1 | **G1b** | best strategy |
|---|---|---|---|---|---|
| cluster | 52 % | 44 % [31–58] | 62 % [48–74] | **68 %** [54–79] | 74 % |
| mixed (held out) | 66 % | 58 % [44–71] | 48 % [35–61] | **78 %** [65–87] | 86 % |
| packing | 100 % | 88 % [76–94] | 100 % [93–100] | 100 % [93–100] | 100 % |
| reach | 80 % | 82 % [69–90] | 88 % [76–94] | 78 % [65–87] | 94 % |
| stacking | 48 % | 54 % [40–67] | 88 % [76–94] | **98 %** [90–100] | 100 % |
| all | 69 % | 65 % [59–71] | 77 % [72–82] | **84 %** [79–88] | 91 % |

G1b = G1 + 149 expert rows of the `sorting` family (two boxes, two short lines of red and blue cubes; the held-out
`mixed` family has a dense cluster and a pre-filled box). The cross-rule ordering learned on sorting carries over to
mixed's different layouts (48 % → 78 %). Failures — G1b: grasp 18, reachability 17, place 3, collision 1 (G0: grasp
53). Reach 88 % → 78 % is within the intervals; to watch.

## Own attempts filtered (G2b), recovery alone (G3b)

| version | data | all | cluster | mixed | reach | stacking |
|---|---|---|---|---|---|---|
| G1 | expert programs | 77 % [72–82] | 62 % | 48 % | 88 % | 88 % |
| G2b | G1 + 166 own successes on tasks where the docs-level program fails | 74 % [69–79] | 48 % | 38 % | 86 % | 100 % |
| G3b | G1 + 41 confirmed recovery rows | 79 % [73–83] | 66 % | 48 % | 82 % | 98 % |

Filtering Glimmer's own successes to the informative ones removes most of G2's damage but still does not help:
learning from the expert's search beats learning from the model's own successes (packing 100 % for all three).
Recovery rows help slightly (within the intervals). Next: G1c = G1b + recovery rows.

## G1c: recovery rows with conflicting labels

| version | all | cluster | mixed | packing | reach | stacking |
|---|---|---|---|---|---|---|
| G1b | **84 %** [79–88] | 68 % | 78 % | 100 % | 78 % | **98 %** |
| G1c = G1b + 41 recovery rows | 78 % [72–82] | 68 % | **84 %** | 98 % | 88 % | 50 % |

G1c writes the top-first order in 5 of 50 held-out stacking programs (G1b: 34). The recovery rows were made from
the first-success expert run (before the most-reliable-strategy labels), so their patches often teach another order
than the program rows: two teachers that disagree, and the stacking habit flips. Fix: regenerate the recovery
examples with the same label rule. Also a caution for every comparison here: one training run per version; a
habit such as top-first can flip between runs, so differences inside the intervals are not claims.

**Best so far: G1b, 84 % held-out (zero-shot G0: 65 %, docs-level program: 69 %, best strategy: 91 %).**

## G1d: consistent recovery labels

| version | all | cluster | mixed | packing | reach | stacking |
|---|---|---|---|---|---|---|
| G1b | **84 %** [79–88] | 68 % | 78 % | 100 % | 78 % | 98 % |
| G1c (G1b + 41 first-success recovery rows) | 78 % [72–82] | 68 % | 84 % | 98 % | 88 % | 50 % |
| G1d (G1b + 63 recovery rows, same labels as the program rows) | 82 % [76–86] | 56 % | 78 % | 100 % | 82 % | 92 % |

Consistent labels remove G1c's stacking collapse (50 % → 92 %). Recovery rows do not improve program writing
beyond G1b (inside the intervals); their value is to be measured where they apply, with the model as recovery
agent (pending).

## G5: DPO on top of G1b

| version | all | cluster | mixed | packing | reach | stacking |
|---|---|---|---|---|---|---|
| G1b | 84 % [79–88] | 68 % | 78 % | 100 % | 78 % | 98 % |
| G5 = G1b + DPO | 83 % [78–87] | 64 % | 76 % | 100 % | 84 % | 92 % |

DPO (TRL 1.9, β 0.1, lr 5e-6, one epoch, 287 pairs: the label program against a strategy that failed on the same
task, 172 of them the listed order; reference = G1b frozen) reaches 15/15 preference accuracy on held-out pairs
(margin 0.84) but leaves task success unchanged. Both G1b and G5 write **one strategy per family** on every held-out
task (cluster, mixed, reach: outer-first; stacking: top-first; packing: nearest-target-first). The model learned a
task-type → strategy map, not a layout-conditioned choice; the remaining gap to the 91 % ceiling is the per-layout
exceptions (about one task in five needs another strategy), present in the labels but too rare to be learned from
the scene description. The DPO pairs mostly contrast the label with the listed order, which G1b already avoids.
GRPO (per-task verified reward within a group of samples) addresses exactly this.

## Training seeds and evaluation noise

A second training seed of G1b (`seed: 7`) writes the **identical program on all 250 held-out tasks**: SFT on these
labels converges to the same task-type → strategy map. Its single evaluation run scored 88 % (first seed 84 %), yet
on 24 of the 250 tasks the identical program had a different outcome. The plan tier samples (MoveIt grasp
candidates, IK seeds, paths), so a task's outcome is a probability, not a fact: about 10 % of tasks flip between
runs. Consequences: the single-run differences above within about ±4 points (G1b, G1d, G5, G3b, the second seed)
are evaluation noise; the large effects (G0 vs the fine-tuned versions, the G2 regression, the mixed family) stand.
From here on every version's programs are run three times and a task's success is the mean of the runs (the
docs-level reference likewise, as the pseudo-version DOCS).

## Repeated evaluation (3 runs per program; a task's success is its mean)

| family | DOCS (docs-level) | G0 | **G1b** (QLoRA SFT) | G6 (G1b + GRPO, weak) | G1b_lora (bf16 LoRA) | best strategy* |
|---|---|---|---|---|---|---|
| cluster | 47 % | 38 % | **65 %** | 59 % | 64 % | 74 % |
| mixed (held out) | 62 % | 51 % | **82 %** | 85 % | 85 % | 86 % |
| packing | 99 % | 87 % | 98 % | 99 % | 100 % | 100 % |
| reach | 84 % | 83 % | 85 % | 86 % | 85 % | 94 % |
| stacking | 47 % | 47 % | 97 % | 98 % | 61 % | 100 % |
| **all** | 68 % [62–73] | 61 % [55–67] | **86 % [81–89]** | 86 % [81–89] | 79 % [74–84] | 91 % |

*single run of every strategy (held-out reference).

- Fine-tuning: **61 % → 86 %** (+25) over zero-shot Glimmer, +18 over the docs-level program; zero-shot is below the
  docs-level program partly because 27 of its 750 runs fail the static checks.
- G6 (GRPO from G1b: 150 prompts with 4 samples each, lr 5e-6, verified rewards, 75 steps; mean reward by quarter
  0.78/0.76/0.90/0.82) leaves behaviour unchanged (2 of 250 programs differ): too weak a run, not a verdict on GRPO;
  a stronger run (G7: 302 prompts, lr 2e-5) follows.
- bf16 LoRA (the cookbook's lora config on one 96 GB GPU, lr 5e-5, same data as G1b) is worse than QLoRA: it varies
  its strategy on cluster (most-free-first on 33 of 50) without gaining, and on stacking chooses outer-first instead
  of top-first on 20 of 50 tasks (61 %). With this data the cookbook's default QLoRA recipe is the better one.

## GRPO with a higher learning rate (G7): diverged

All 302 prompts, lr 2e-5, otherwise as G6. Mean training reward by quarter of the samples seen: 0.86, 0.59, 0.54,
0.54, with programs matching no strategy rising (16 of 472). Stopped at step 59 of 151. Together with G6 (lr 5e-6:
no change): with LoRA on this 30B model and binary rewards from four samples per prompt, we found no stable step
size between "no change" and "drifts away from what works" in two runs; a KL term to G1b (rather than to the base
model, which this TRL's adapter path uses) and more samples per prompt are the next things to try.

## Live recovery (the model as recovery agent)

20 held-out tasks where the docs-level program fails (at most 1 success in its 3 runs; 5 each of cluster, mixed,
reach, stacking; packing has none), the docs-level program run once with each model as the live recovery agent
(served by learn/chat_server.py: transformers + PEFT on NF4, as the cookbook evaluates adapters; budget 3 handoffs).

| recovery agent | rescued | decisions (25-26 handoffs) | invalid decisions |
|---|---|---|---|
| G0 (zero-shot) | 7 / 20 | reobserve 11, patch 9, skip 2, fix_world 2, retry_step 2 | 0 |
| G1d (G1b + 63 confirmed recovery rows) | 9 / 20 | patch 25 | 0 |

G1d rescues stacking 4/5 (G0 1/5) by rewriting the rest of the program in a working order; G0 mostly re-observes,
which rarely helps on a blocked grasp. On 20 tasks the 9 vs 7 is not a claim; the behaviour change is. Both always
answer with a valid decision (the corrective turn for non-decision replies).

## Physics labels (for G4)

150 training tasks (30 per family): each task's plan-tier label run in Isaac physics, then up to two alternatives
(other plan-tier successes, most reliable first) until one succeeds.

| family | label works in physics | some strategy works in physics |
|---|---|---|
| cluster | 73 % | 87 % |
| packing | 100 % | 100 % |
| reach | 73 % | 93 % |
| sorting | 93 % | 100 % |
| stacking | 37 % | 37 % |

The stacking failures are mostly not strategy failures: 23 of the 33 failed stacking attempts are ICON refusing to
start a trajectory (the Isaac arm lagging its setpoint when placing onto another cube; the half-speed default
reduced but did not remove it), plus grasp and planning failures. Physics labelling therefore mostly measures the
sim's robustness. G4 changes a label only where physics showed the label fails and another strategy succeeds (12
tasks: cluster, reach, sorting); tasks where nothing succeeded keep their plan-tier label instead of being dropped.
G4 differs from G1b in under 2 % of its rows, so no measurable effect is expected. The stacking arm lag is a
robustness fix for the demo (see the Isaac notes).

## Sim tier: G1b on Isaac physics (50 held-out tasks, 10 per family)

| family | plan tier | Isaac physics |
|---|---|---|
| cluster | 80 % | 60 % |
| mixed | 70 % | 80 % |
| packing | 100 % | 100 % |
| reach | 90 % | 70 % |
| stacking | 100 % | 90 % |
| all 50 | 88 % | **80 %** |

Agreement 76 % (38/50): both ok 36, both failed 2, plan ok / physics failed 8, plan failed / physics ok 4.
Physics-only failures: ICON stop outside position limits 2, arm lag (setpoint difference) 2, and 4 planning failures
that occur only because, in physics, the arm and the cubes end up slightly off the exact poses the plan tier assumes.
The plan tier is a good but optimistic proxy (about 8 points on this subset, mostly cluster and reach): the case
for G4 (training on Isaac-verified episodes).

## Pending

Recovery examples regenerated with the most-reliable-strategy labels (then G1c again); live-recovery evaluation
(the model as recovery agent); G4 on Isaac-verified episodes; repeat the key versions with a second training seed;
videos (G0 vs G1b on the same layout).
