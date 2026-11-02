# Phase V3 notes: reading task intent

Plan: [phase_v3.md](phase_v3.md).

## What was built

*   `program_runtime/reader.py` samples evenly spaced JPEG frames including both endpoints, resizes them, reads
    clips or sentences through the base model, validates the last fenced JSON intent, makes one corrective turn,
    wraps specified failures in `ReadError`, and emits exactly one structured log record per read.
*   `ChatClient.chat` supports a per-call model override after the extra request body is merged. `ProgramWriter`
    and `RecoveryAgent` route their optional model on every request, including corrective and tool-related turns.
*   The CLI has `read`, `--writer-model`, and `--recovery-model`; the run path shares one client between writing
    and recovery.
*   `learn/chat_server.py` decodes data-URL image parts to Pillow images, rejects URLs that would require a
    network fetch, uses processor tensorization for image requests, and preserves the evaluated text-only path.
*   `tests/test_reader.py` and the extended server tests cover frame extraction, parsing and correction, errors,
    model routing, CLI behavior, structured logging, image decoding, and both model input branches.

## Test results

```
$ .venv/bin/python -m pytest tests/ -q
309 passed in 28.11s
```

`py_compile` passes on every changed Python file. `.venv/bin/python -m program_runtime.cli read --help` prints
the usage.

## Deviations and open questions

*   The existing fake chat clients in `tests/test_program_runtime.py` and `tests/test_sft_rows.py` were updated to
    accept the new `model` keyword. These test-only compatibility edits are outside the plan's enumerated file
    list but are required for the unchanged tests to exercise the expanded client contract.
*   No implementation deviations or open questions remain.
