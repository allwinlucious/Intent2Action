# Phase V7 implementation notes

## Built

* A local-only `ThreadingHTTPServer` demo with the static-file allowlist, JSON and clip APIs, origin and body
  limits, SSE replay, a shared MJPEG watcher, cached status probes, fake mode and real robot/model wiring.
* A three-column, fixed-screen page for typed, recorded and uploaded input; reading and clarification; editable
  intent confirmation; program review; a human-gated run; live runtime events; question and patch-approval
  controls; the robot/camera view; and the final per-object checklist. It uses system fonts and no external
  assets or libraries.
* Fourteen deterministic scene presets: nine scripted layouts and the five specified held-out tasks. Fake scene
  loads construct the objects directly from the preset placements.
* Revision-bound intent confirmation, program writing and running, plus append-only attempt and run JSONL logs.
  Scene changes, clarification and confirmation invalidate stale programs.
* Runtime cancellation that wins after successful or failed calls, and operator approval for valid recovery
  patches. Stop also wakes a pending question or approval before requesting a real robot stop.
* Copy-on-write clarification conversations, optional clip notes, retained sampled frames, and bounded two-pass
  clip decoding with pre-decode dimension checks.
* Configurable saved-frame timing through `ISAAC_FRAME_PERIOD_S`, defaulting to the previous 0.2 seconds.
* Unit and HTTP tests for the runtime changes, reader and CLI changes, gates and revisions, body security,
  questions, approvals, cleanup, fake scene loading, SSE, and MJPEG decoding/restart behavior.

## Verification

Pytest command:

```text
.venv/bin/python -m pytest tests/ -q
```

Pytest summary line:

```text
331 passed
```

`py_compile` passed for every changed Python file. The fake server smoke check served `/`, `/api/scenes`, and
JPEG multipart bytes from `/api/sim.mjpg`, and the server process was stopped by its exact PID.

## Starting the page

Fake mode, with no robot, simulator or model service:

```bash
.venv/bin/python -m demo_ui.server --fake --port 8765
```

Open `http://127.0.0.1:8765`.

Real mode prerequisites are the existing simulation started with `--gui`, the robot-service container reachable
at the configured address, and the model server configured by `LLM_BASE_URL`, `LLM_API_KEY` and `LLM_MODEL`
(directly in the environment or in the repository `.env`). To save viewer frames at 15 fps, set
`ISAAC_FRAME_PERIOD_S=0.0666667` in the simulation process. Then start:

```bash
.venv/bin/python -m demo_ui.server --port 8765 --address 127.0.0.1:18100 \
  --frames /mnt/c/isaac-sim-work/viewer_frames
```

Use `--sim-address` only when SimService is on a different address. Reader, writer and recovery model routes can
be changed with `--reader-model`, `--writer-model` and `--recovery-model`.

## Deviations and open questions

* No implementation deviations.
* No open design questions were encountered.
* A user-question timeout returns the empty string. The runtime puts that value in `context["user_answer"]` and
  asks the recovery model for its next decision; the empty answer has no separate runtime behavior. Stop is
  different: it wakes the wait with the empty result and the set cancellation event ends the run before another
  recovery decision.
* Real simulation, container and model-server operation was not run here, as required by the worker constraints.
