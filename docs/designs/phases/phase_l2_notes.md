# Phase L2 implementation notes

Status: complete. Addendum A.11 resolved the mixed-box overlap recorded by the earlier partial implementation.

## Files changed

* `learn/families.py`
* `tests/test_families.py`
* `tests/e2e/hard_task.py`
* `docs/designs/learning_from_experience.md`
* `docs/designs/phases/phase_l2_notes.md`

Generated preview artifacts (ignored by git):

* `logs/families/cluster_3.png`
* `logs/families/packing_3.png`
* `logs/families/stacking_3.png`
* `logs/families/reach_3.png`
* `logs/families/mixed_100003.png`

## Test output

```text
$ .venv/bin/python -m pytest tests/ -q
........................................................................ [ 34%]
........................................................................ [ 68%]
..................................................................       [100%]
210 passed in 14.06s
```

All preview commands printed their task JSON, exited 0, and wrote 800 x 800 RGB PNGs:

```text
$ .venv/bin/python learn/families.py preview --family cluster --seed 3
$ .venv/bin/python learn/families.py preview --family packing --seed 3
$ .venv/bin/python learn/families.py preview --family stacking --seed 3
$ .venv/bin/python learn/families.py preview --family reach --seed 3
$ .venv/bin/python learn/families.py preview --family mixed --seed 100003
```

The required statistics sweep exited 0 and printed these nine JSON lines:

```text
$ .venv/bin/python learn/families.py stats --n 200
{"family":"cluster","split":"train","n":200,"mean_objects":5.71,"min_gap_m":0.010015403612132392,"median_gap_m":0.021361466122001255,"mean_retries":1.04}
{"family":"cluster","split":"heldout","n":200,"mean_objects":5.8,"min_gap_m":0.01005631310920161,"median_gap_m":0.02005085864426847,"mean_retries":1.19}
{"family":"packing","split":"train","n":200,"mean_objects":5.65,"min_gap_m":0.06077962300180594,"median_gap_m":0.11212550332034722,"mean_retries":1.715}
{"family":"packing","split":"heldout","n":200,"mean_objects":5.665,"min_gap_m":0.06197012170472413,"median_gap_m":0.10017282809848792,"mean_retries":1.725}
{"family":"stacking","split":"train","n":200,"mean_objects":3.525,"min_gap_m":0.060002815625872764,"median_gap_m":0.10081407729060932,"mean_retries":1.43}
{"family":"stacking","split":"heldout","n":200,"mean_objects":3.575,"min_gap_m":0.06085732942051735,"median_gap_m":0.09767699161197423,"mean_retries":1.55}
{"family":"reach","split":"train","n":200,"mean_objects":4.51,"min_gap_m":0.06046580048725331,"median_gap_m":0.16756340831843916,"mean_retries":55.43}
{"family":"reach","split":"heldout","n":200,"mean_objects":4.44,"min_gap_m":0.060794122931814756,"median_gap_m":0.1647149090061644,"mean_retries":48.61}
{"family":"mixed","split":"heldout","n":200,"mean_objects":9.225,"min_gap_m":0.010151198345753041,"median_gap_m":0.0202107632575184,"mean_retries":3.34}
```

No Docker, Isaac Sim, Bazel, GPU, model, or end-to-end command was run. This phase started no containers, so no
phase-owned containers were left running.
