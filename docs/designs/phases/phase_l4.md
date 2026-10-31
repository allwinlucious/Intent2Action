# Phase L4: training data (episodes in bulk, Glimmer's own attempts, rows per model version)

Part of [learning_from_experience.md](../learning_from_experience.md). Needs L3. Produces the SFT datasets of the
model versions G1–G3 (G4, sim-tier data, and G5, DPO, follow in later phases) in the Axolotl `messages` format
that `learn/modal_train.py` uploads. The prompts are exactly the product's prompts (`program_messages`, the
recovery agent's), so a fine-tuned model is used the same way as the zero-shot one.

## Constraints for the worker

*   Allowed: editing files; unit tests; the plan tier (`--start-stack plan`, Docker via `sg docker -c`); the
    commands in the done criteria. Model calls: **only** through `learn/glimmer_sample.py` (decision 4) and only
    when the environment variable `LLM_BASE_URL` is set; the worker does not start or deploy the Modal endpoint and
    does not run `sample` (the lead runs it, it costs money). Not allowed: Isaac Sim, Modal commands,
    `git commit`, `pkill -f` (stop processes by PID), modifying `/home/mew/intrinsic-core`.
*   No design decisions beyond this plan; anything uncovered goes to `docs/designs/phases/phase_l4_notes.md`.
*   Code style of the surrounding files. Every long run prints one status line per task and a summary, and can be
    resumed: rerunning a command skips tasks already present in its output file (keyed by family, seed, and for
    samples the sample index).

## Done criteria

*   `.venv/bin/python -m pytest tests/ -q` passes, including `tests/test_sft_rows.py` and
    `tests/test_glimmer_sample.py` (decision 7).
*   With `--start-stack plan`: `.venv/bin/python learn/search_expert.py solve --split train --n 5` and
    `recovery --split train --n 5` run with the extended records (decision 1), and
    `.venv/bin/python learn/sft_data.py build --version G1 --solve logs/expert/train.jsonl --out logs/sft/G1_test`
    and the same for G3 write valid datasets (decision 5) from that small run.
*   `phase_l4_notes.md`: files changed, test output, the dataset statistics printed by `build`.

## Decisions

### 1. The scene the model saw (`learn/episodes.py`, Modify)

`Episode` gains `objects: list[dict]`: `dataclasses.asdict` of every `Obj` from `GrpcBackend(stub).list_objects()`
taken right after the task is loaded, before the program runs: the exact scene a model gets in `program_messages`.
`record()` keeps it. `search_expert.solve` records it once per task as `"objects"` (from the first episode);
`recovery` records already contain the full context.

### 2. Bulk expert runs (no code change; the lead runs them)

`solve --split train --n 150 --start-stack plan` (600 tasks) and `recovery --split train --n 150`. Both resume
(constraint above), so they can be stopped and continued.

### 3. Rows (`learn/sft_data.py`, Modify: add `build`; keep `smoke`)

*   **Program row** (from a solve record with `expert_program`): `program_messages(intent, objects)` +
    `{"role": "assistant", "content": "```python\n" + program + "```"}`, objects rebuilt with `Obj(**d)`. The fenced
    form is what `_program_text` accepts and what the zero-shot model writes.
*   **Recovery row** (from a confirmed recovery record): the recovery agent's messages for that context, built by a
    new function `recovery_messages(context: dict) -> list[dict]` in `program_runtime/agent.py` that
    `RecoveryAgent.recover` also uses (the system prompt and the user JSON it sends today; refactor only), + the
    assistant reply `json.dumps(decision)`. No tool-call turns (the expert decides directly).
*   **Self row** (G2, from a successful sample record of decision 4): the same as a program row with the sampled
    program (fenced).
*   Rows longer than 4096 tokens with Glimmer's tokenizer are dropped and counted (Axolotl's `sequence_len`);
    the token count uses `transformers.AutoTokenizer.from_pretrained("meta-models/Muse-Glimmer-30B")` and its
    chat template with `reasoning_strength="low"`, run in `.venv-ml`.

### 4. Glimmer's own attempts (`learn/glimmer_sample.py`, Create)

*   `generate --split train --n N --k 4 --out logs/samples/train_programs.jsonl`: for each task, `k` programs from
    the endpoint in `LLM_BASE_URL` (`program_runtime.agent.ChatClient` with `temperature=0.7` added to the request
    body; the reasoning strength from `LLM_EXTRA_BODY` as today), using the writer prompt and the scene from the
    solve record's `objects`; one JSON line per (task, sample index): `{"family", "seed", "split", "sample",
    "program", "seconds"}`. Up to 8 requests concurrently (thread pool).
*   `run --in logs/samples/train_programs.jsonl --out logs/samples/train_episodes.jsonl [--start-stack plan]`: each
    sampled program through `run_episode` (static check and dry run happen inside the runtime: a program that
    fails them is an episode with `ok=False`), one JSON line per sample with the episode record.
*   Generation and running are separate so the paid endpoint is up only while generating.

### 5. Datasets per version (`build --version V`)

| Version | Rows |
|---|---|
| G1 | program rows of every solved train task |
| G2 | G1 + self rows (successful samples; at most 2 per task, deduplicated by program text) |
| G3 | G2 + recovery rows (if G2's samples are missing, G1 + recovery rows; the summary says which) |

Each dataset: shuffle with seed 0, 5 % (at least 16 rows) to `validation.jsonl`, the rest `train.jsonl`, plus
`manifest.json` `{"version", "rows": {"program", "self", "recovery"}, "dropped_too_long", "families",
"sources": [paths]}`. Validation never contains a held-out seed (only train seeds are used at all).

### 6. Model versions and serving (documented in `learn/modal_train.py`'s docstring only; no code)

Adapters are saved under `runs/<version>/` on the volume; the serving phase (L6) loads them with vLLM
`--enable-lora --lora-modules G1=... G2=...`.

### 7. Tests

*   `tests/test_sft_rows.py`: program row equals `program_messages` + fenced program; recovery row's prompt equals
    what `RecoveryAgent` sends for the same context (patch `ChatClient.chat` to capture it); `build` versions
    contain the right row kinds and counts on small fixture files; validation split size and determinism; the
    token-length filter with a fake tokenizer.
*   `tests/test_glimmer_sample.py`: `generate` with a fake client (k samples per task, resume skips existing,
    temperature in the request); `run` with a fake `run_episode` (one record per sample, resume).

## Addendum A (review resolutions; supersedes the decisions above where they differ)

### A.1 Resume (`learn/search_expert.py`, Modify — decision 2 is no longer "no code change")

`solve` and `recovery` open their output in append mode and skip tasks whose `(family, seed)` is already in it;
the per-family summary is computed over the whole output file. The L3 files were moved to
`logs/expert/l3_first_*.jsonl` (they lack `objects`); records without `objects` are treated as absent by every
reader (skipped by `generate`/`build`, re-run by `solve`). The done-criteria small run writes to
`--out logs/expert/l4_test.jsonl` (solve) and `--in logs/expert/l4_test.jsonl --out
logs/expert/l4_test_recovery.jsonl` (recovery) so it never touches the bulk files.

### A.2 Episode.objects (replaces decision 1's wording)

`objects: list[dict] = dataclasses.field(default_factory=list)` as the **last** field of `Episode`, filled in
`run_episode` from `GrpcBackend(stub=robot_stub).list_objects()` right after loading. Episode records keep it;
`task_record` also copies the first episode's `objects` to the record's top level. Update the positional `Episode`
helper in `tests/test_search_expert.py`.

### A.3 Rows are rendered text (replaces decision 3's output format and decision 6)

*   `build` (in `.venv`, no transformers) writes **message rows** as decided in decision 3, plus for recovery rows a
    `tools` key holding the exact specs `RecoveryAgent` sends: `recovery_messages(context, tools) -> list[dict]` and
    a module function `tool_specs(tools)` (moved out of `RecoveryAgent._specs`, which calls it); `build` gets the
    specs from `mcp_server.server.Tools(DESCRIPTOR_SET)`.
*   A second command, run in `.venv-ml`: `.venv-ml/bin/python learn/render_rows.py --data DIR` renders every row of
    `DIR/{train,validation}.jsonl` with Glimmer's chat template:
    `prompt = tok.apply_chat_template(messages_without_last, tools=tools or None, add_generation_prompt=True,
    tokenize=False, reasoning_strength="low")`, `full = tok.apply_chat_template(messages, tools=tools or None,
    tokenize=False, reasoning_strength="low")`; asserts `full.startswith(prompt)`; writes
    `DIR/rendered/{train,validation}.jsonl` with `{"segments": [{"label": false, "text": prompt}, {"label": true,
    "text": full[len(prompt):]}]}` (Axolotl's template-free `input_output` format). Rows over 4096 tokens
    (`len(tok(full)["input_ids"])`) are dropped and counted in `DIR/rendered/manifest.json`.
*   Tokenizer: `transformers.AutoTokenizer.from_pretrained("meta-models/Muse-Glimmer-30B")` (public repo; only the
    tokenizer files are downloaded). `render_rows.py` imports transformers at top level (it only runs in
    `.venv-ml`); its pure function `render(messages, tools, tokenizer) -> dict` takes the tokenizer as an argument
    and is tested in `.venv` with a fake tokenizer.
*   `learn/modal_train.py` is changed by the lead (not the worker) to use `type: input_output` on
    `rendered/`.

### A.4 Validation split (replaces the rule in decision 5)

`n_val = ceil(0.05 * n)` if that is ≥ 16, else `max(1, min(16, n // 5))` (n ≥ 2; n < 2 → no validation). Shuffle
with seed 0 first. "Valid dataset" = both files non-empty when n ≥ 2, every line a JSON object with `messages`.

### A.5 Sampling (replaces decision 4's open points)

*   `ChatClient.__init__(..., temperature: float = 0.0)`; the request body is `{"temperature": self.temperature,
    ..., **self.extra}` (an `LLM_EXTRA_BODY` temperature wins; documented in the docstring).
*   `generate --solve PATH (default logs/expert/<split>.jsonl) --n N [--families ...] --k 4 --out PATH`: `--n` per
    family as in `solve`; every solve record with `objects` is sampled, solved by the expert or not.
    Each sample = `ProgramWriter(client).write(intent, objects)` (the product path, including its one corrective
    retry); `program` stores the returned program, plus `attempts` and `seconds`. An HTTP error or timeout writes
    `{"family", "seed", "split", "sample", "error"}`; such records are retried on the next run (resume treats only
    records with `program` as done). Output writes are serialized with a lock.
*   `run --in PATH --out PATH`: the task is `families.generate(family, seed)`; `agent=None`; the stack helper is
    extracted from `search_expert.main` into `episodes.stack(start: str)` (a context manager) and used by both
    CLIs; each output line is `record(episode) | {"sample": i, "intent": task.intent}`; resume key
    `(family, seed, sample)`.

### A.6 build CLI (replaces decision 5's interface)

`build --version {G1,G2,G3} --solve PATH --samples PATH --recovery PATH --out DIR` with defaults
`logs/expert/train.jsonl`, `logs/samples/train_episodes.jsonl`, `logs/expert/recovery_train.jsonl`. "Missing" =
the file does not exist: G2 without samples is an error; G3 without samples falls back to G1 + recovery and the
manifest says so. "At most 2 self rows per task": the two lowest sample indexes among the successful, distinct
programs. Recovery rows are capped at 30 % of the dataset (random choice, seed 0); the manifest reports rows per
kind and per family (no rebalancing). `program_row` becomes the fenced form (also used by `smoke`).

### A.7 Phase numbers

Serving the adapters is part of L5 (fine-tuning and serving); L6 is the evaluation.
