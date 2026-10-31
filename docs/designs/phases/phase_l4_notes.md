# Phase L4 implementation notes

Status: implemented. Unit tests pass. The done-criteria small run (`solve` and `recovery`, `--n 5`, plan tier) wrote
the extended records, and `build` wrote valid G1 and G3 datasets from it. No container is left running. Not run, as
the constraints require: `generate` (paid endpoint), the bulk runs (decision 2), and `render_rows.py` (`.venv-ml`; it
downloads the tokenizer and is not among the allowed commands).

## Files changed

* `program_runtime/agent.py`: `ChatClient(..., temperature=0.0)`. The request body uses `self.temperature`, and an
  `LLM_EXTRA_BODY` temperature still wins (stated in the docstring). New module functions `tool_specs(tools)` (moved
  out of `RecoveryAgent._specs`, which now calls it) and `recovery_messages(context)` (the system prompt and user JSON
  that `RecoveryAgent.recover` builds and still uses). The prompt text is unchanged.
* `learn/episodes.py`: `Episode.objects` is the last field and defaults to an empty list. `run_episode` fills it from
  `GrpcBackend(stub=robot_stub).list_objects()` right after `families.load`. New `stack(start)` context manager,
  extracted from `search_expert.main`, which starts and stops the plan tier and runs the port check. New
  `read_jsonl(path)`, which returns `[]` when the file does not exist.
* `learn/search_expert.py`: `task_record` adds `"objects"` from the first episode. `solve` and `recovery` append to
  their output and skip `(family, seed)` already in it, and print a `resume:` line first. Solve records without
  `objects` count as absent: `solve` runs them again, and `recovery` does not read them. The solve summary covers the
  whole output file. Uses `episodes.stack`.
* `learn/sft_data.py`: `program_row` writes the fenced form (`smoke` uses it too). New `recovery_row`, `program_rows`,
  `self_rows`, `recovery_rows` (30 % cap), `validation_size` and `split` (A.4), `build` and the `build` CLI (A.6).
  The CLI now has subcommands (`smoke --out` still works).
* `learn/glimmer_sample.py` (new): `generate` and `run` (decision 4, A.5).
* `learn/render_rows.py` (new): `render(messages, tools, tokenizer)`, `render_file(source, target, tokenizer)`, and
  the CLI (A.3).
* `tests/test_search_expert.py`: the `Episode` helper passes `objects`. Added a `solve` resume test (including a
  record without `objects` being run again) and a `recovery` resume check.
* `tests/test_sft_rows.py` (new, 17 tests), `tests/test_glimmer_sample.py` (new, 3 tests).
* `docs/designs/phases/phase_l4_notes.md` (new).

Not changed: `learn/modal_train.py`. A.3 replaces decision 6, and the lead makes the `input_output` change.

Generated (git-ignored): `logs/expert/l4_test.jsonl` (20 records), `logs/expert/l4_test_recovery.jsonl` (1 record),
`logs/expert/l4_test_solve.out`, `logs/expert/l4_test_recovery.out`, `logs/expert/l4_pytest.out`,
`logs/sft/G1_test/`, `logs/sft/G3_test/`, `logs/plan_tier_container.log`.

## Test output

```text
$ .venv/bin/python -m pytest tests/ -q
........................................................................ [ 26%]
........................................................................ [ 52%]
........................................................................ [ 79%]
........................................................                 [100%]
272 passed in 25.36s
```

`test_sft_rows.py` checks the following:

* A program row equals `program_messages` followed by the fenced program, and `_program_text` recovers the program.
* A recovery row's messages and `tools` equal what `RecoveryAgent` sends for the same context, captured by patching
  `ChatClient.chat` and using the real `Tools(DESCRIPTOR_SET)`.
* G1, G2 and G3 have the right row kinds and counts on fixtures. The fixtures cover a record without `objects`, a
  held-out record, a duplicate sampled program, a failed sample, the two-per-task limit, and the recovery cap.
* G2 without samples is an error. G3 without samples falls back, and the manifest says so.
* `validation_size` on ten sizes, and the split is deterministic and complete.
* `render` and the token filter, with a fake tokenizer.

`test_glimmer_sample.py` drives `generate` through `main()` with a fake HTTP endpoint. It checks k samples per task,
that records without `objects` are skipped, that error records are written and retried, that a resume skips finished
samples, and that `temperature` is 0.7 in every request body. It also checks that an `LLM_EXTRA_BODY` temperature
wins. Finally it runs `run` with a fake `run_episode`: one record per sample, `sample` and `intent` added, error
records skipped, and resume.

## Small run (plan tier, done criteria)

Before starting, I waited for another session's Isaac `program_check.py` run to finish, because it shares the
`intrinsic` container name. Each command started its own plan-tier container with `--start-stack plan` and removed it
on exit.

```text
$ .venv/bin/python learn/search_expert.py solve --split train --n 5 --out logs/expert/l4_test.jsonl --start-stack plan
resume: 0 tasks already in logs/expert/l4_test.jsonl
...
wrote logs/expert/l4_test.jsonl
cluster: tasks=5 solved=100% listed=60% wins listed=3 most_free_first=2 outer_first=0 nearest_target_first=0 top_first=0 mean_s=9.5 mean_calls=7.4
packing: tasks=5 solved=100% listed=100% wins listed=5 most_free_first=0 outer_first=0 nearest_target_first=0 top_first=0 mean_s=3.4 mean_calls=2.4
stacking: tasks=5 solved=100% listed=20% wins listed=1 most_free_first=2 outer_first=1 nearest_target_first=1 top_first=0 mean_s=3.0 mean_calls=2.4
reach: tasks=5 solved=100% listed=100% wins listed=5 most_free_first=0 outer_first=0 nearest_target_first=0 top_first=0 mean_s=8.4 mean_calls=6.8

$ .venv/bin/python learn/search_expert.py recovery --split train --n 5 --in logs/expert/l4_test.jsonl --out logs/expert/l4_test_recovery.jsonl --start-stack plan
resume: 0 tasks already in logs/expert/l4_test_recovery.jsonl
...
cluster: tasks=2 confirmed=1 not_confirmed=0 no_handoff=1 mean_s=12.9 mean_calls=9.5; file: confirmed=1
packing: tasks=0 confirmed=0 not_confirmed=0 no_handoff=0 mean_s=0.0 mean_calls=0.0; file: confirmed=0
stacking: tasks=4 confirmed=0 not_confirmed=2 no_handoff=2 mean_s=14.7 mean_calls=3.0; file: confirmed=0
reach: tasks=0 confirmed=0 not_confirmed=0 no_handoff=0 mean_s=0.0 mean_calls=0.0; file: confirmed=0
total: confirmed=1 not_confirmed=2 no_handoff=3; 1 recovery examples confirmed; file: 1 examples
```

All 20 solve records, and every episode inside them, have a non-empty `objects` list.

## Dataset statistics printed by `build`

```text
$ .venv/bin/python learn/sft_data.py build --version G1 --solve logs/expert/l4_test.jsonl --recovery logs/expert/l4_test_recovery.jsonl --out logs/sft/G1_test
G1 -> logs/sft/G1_test: train=16 validation=4 program=20 self=0 recovery=0
  cluster: program=5 self=0 recovery=0
  packing: program=5 self=0 recovery=0
  reach: program=5 self=0 recovery=0
  stacking: program=5 self=0 recovery=0
$ .venv/bin/python learn/sft_data.py build --version G3 --solve logs/expert/l4_test.jsonl --recovery logs/expert/l4_test_recovery.jsonl --out logs/sft/G3_test
G3 -> logs/sft/G3_test: train=17 validation=4 program=20 self=0 recovery=1 (the samples file does not exist: G1 + recovery rows)
  cluster: program=5 self=0 recovery=1
  packing: program=5 self=0 recovery=0
  reach: program=5 self=0 recovery=0
  stacking: program=5 self=0 recovery=0
```

`--solve` and `--recovery` point at the A.1 small-run files rather than the defaults that decision 5's done criterion
names. `logs/samples/` does not exist, so G3 fell back to G1 + recovery. `build --version G2` on the same input exits
with "G2 needs the samples file ..., which does not exist". In both datasets, both files are non-empty and every line
is a JSON object with `messages`. The single recovery row carries `tools`.

## Uncovered by the plan (recorded; where code needed one reading, it is named)

1.  **`recovery_messages` signature.** Decision 3 gives `recovery_messages(context)`, and A.3 gives
    `recovery_messages(context, tools)`. The tools play no part in the messages: the agent sends the specs as a
    separate request field. I implemented `recovery_messages(context)`. The rows' `tools` come from
    `tool_specs(Tools(DESCRIPTOR_SET))`, the same specs that `RecoveryAgent` sends, and the test asserts this. If
    the two-argument form was intended, the change is a signature change with no effect on the output.
2.  **Recovery resume skips only confirmed tasks.** The recovery output holds only confirmed examples. So with the
    literal rule ("skip tasks whose `(family, seed)` is already in it"), `not_confirmed` and `no_handoff` tasks run
    again on every resume. Implemented literally. In the bulk run they cost one episode each per restart.
3.  **Recovery summary "over the whole output file".** The file holds only confirmed examples, so the not-confirmed
    and no-handoff counts can only come from the current run. Each per-family line keeps this run's outcome counts and
    adds `; file: confirmed=N`. The total line adds the file's example count.
4.  **Self-row scene.** A self row uses the solve record's `intent` and `objects`, which is the prompt the program was
    sampled from ("the same as a program row"). It does not use the objects of the `run` episode, whose ids can differ
    after reloading. A sample whose task has no solve record with `objects` gives no row.
5.  **`dropped_too_long` in the build manifest.** Decision 5 lists it, but A.3 moves the token filter to
    `render_rows.py`, and `build` has no tokenizer. The count is in `DIR/rendered/manifest.json` per split, and the
    build manifest has no such key.
6.  **Recovery cap arithmetic.** "Capped at 30 % of the dataset" is read as at most 30 % of the final dataset:
    `r <= floor(3 * others / 7)`, chosen with `random.Random(0).sample`. For example, 20 program rows allow 8 recovery
    rows.
7.  **G3 with no recovery file.** The plan defines "missing" only for samples. A missing recovery file (or solve file)
    stops `build` with an error, the same as G2 without samples.
8.  **Manifest additions.** `"fallback"` is `null`, or the text naming the G1 + recovery fallback. `"train"` and
    `"validation"` give the split sizes.
9.  **Which sampling errors become error records.** `urllib.error.URLError` (which includes `HTTPError`) and
    `TimeoutError`. Any other exception, such as a malformed response body, stops `generate`. The records already
    written are kept and resumed.
10. **CLI defaults.** For `generate`, `--split`, `--n` and `--out` are required, `--k` defaults to 4, and `--solve`
    defaults to `logs/expert/<split>.jsonl`. For `run`, `--in` and `--out` are required (the plan writes them without
    brackets).
11. **Status lines.** `solve` and `recovery` still print one line per episode, as in L3, plus the new `resume:` line.
    `generate` prints one line per sample, and `run` one per episode.
12. **`render_rows.py` was not run against the real template.** The `full.startswith(prompt)` assertion and the token
    counts are tested only with a fake tokenizer. If Glimmer's template renders the prompt differently once the
    reply is appended (for example, a reasoning block inserted before the final assistant turn), the first real run
    will fail on that assertion.
13. **The port check moved.** It moved with the stack helper into `episodes.stack`, which reads the address from
    `episodes.ADDRESS` (the same port, 18100).
14. **Recovery records and `objects`.** The rule "records without `objects` are treated as absent" is applied to solve
    records. Recovery examples keep their context at the top level (decision 1: "recovery records already contain the
    full context"), and their episode record now also carries `objects`.
