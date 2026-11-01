# Learning from experience: results

Running report of [learning_from_experience.md](learning_from_experience.md). Every number below is from the plan
tier (real planning, no physics), on the **held-out** seeds: 50 tasks per family, 250 in total, the family `mixed`
never seen in training. Success = the program runs to the end and the rule checker passes on ground truth.
Generation follows the model cookbook's adapter evaluation: transformers + PEFT on an NF4 base (G0 = the same base,
no adapter), greedy, one attempt, the product's writer prompt. 95 % Wilson intervals in brackets.

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
