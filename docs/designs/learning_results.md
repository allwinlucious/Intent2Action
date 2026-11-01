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

## Pending

G2 (+ Glimmer's own successful attempts), G3 (+ confirmed recovery patches), live-recovery evaluation, Isaac
(physics) subset and plan/physics agreement, G4.
