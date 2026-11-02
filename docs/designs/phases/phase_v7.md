# Phase V7: the demo UI

Part of [demo_to_api.md](../demo_to_api.md) (V7). This is one local web page that the owner screen-records for
the video. It shows the whole chain on one screen:

1.  the input (a typed request, a webcam recording or an uploaded clip);
2.  what the model saw and the intent it read, with a confirm step;
3.  the program the fine-tuned model wrote;
4.  the run, live: the current line, every call and its result, recovery handoffs and patches, and questions the
    agent asks;
5.  the ground-truth checklist at the end;
6.  the robot view:
    *   the live Isaac render;
    *   or a camera, for a real robot later.

The first step uses typed requests only (owner, 2026-11-02). Webcam and upload use the same path
(`IntentReader.read_clip`, phase V3). Out of scope here, and recorded as next steps:
*   audio (speech-to-text is undecided);
*   the study and results charts;
*   the kitting scene.

## Owner decisions taken by the lead (2026-11-02)

*   **Input:** all three, as tabs: Type, Record (webcam, in the browser) and Upload. Recording live in front of
    the camera is the strongest moment for the video; Upload is the fallback for pre-filmed clips.
*   **Clip plus text (owner, 2026-11-02):** a clip can carry an optional typed note, for example "only the red
    ones" or "like this, but into the blue box". The reader gets the frames and the note together. A speech
    transcript will later fill the same slot.
*   **Human verification before the robot moves (owner, 2026-11-02):**
    *   **Intent gate.** The reading is shown in plain sentences. If it is wrong, the owner types a
        clarification ("no, only the red ones") and the model re-reads in the same conversation, for as many
        rounds as needed. Confirm locks the intent.
    *   **Program gate.** The program has its own window. Run is enabled only after the program is shown, and the
        owner presses it. Rewrite asks the writer again.
    *   Every round (reading, clarification, confirm, program, run) is recorded in the run's log as the
        verification trail. The UI shows it as a small history under the intent.
*   **Robot view:** one panel with a source switch:
    *   "Isaac", the default: a live MJPEG stream of the viewer frames Isaac saves;
    *   "Camera": any camera the browser can open, for a real robot.
*   **Stack:** a Python standard-library HTTP server, with one static page in plain HTML, CSS and JavaScript.
    No build step, no CDN and no new dependency, so it runs offline on the owner's PC.

## Constraints for the worker

*   **No heavy compute:** no Isaac Sim, no Docker, no GPU, no network. Do not run `tests/e2e/*`.
*   **Allowed:** editing files, `.venv/bin/python -m pytest tests/ -q`, `py_compile`, and starting the server
    locally on a free port with fakes (section 7) to check that it serves.
*   No new dependencies. `.venv` has `opencv-python-headless`, `numpy`, `pillow`, `grpcio` and `PyYAML`.
*   Do not modify `/home/mew/intrinsic-core`. No `git commit`. No `pkill -f`. Stop only processes you started,
    by PID.
*   Make no design decisions beyond this plan. Record anything not covered in `phase_v7_notes.md` and stop that
    part.
*   **Code style:** 2-space indent in Python and JavaScript; comments only for the why.
*   **Do not name** the event, its organiser or any agent tool in code, comments or the page.

## Done criteria

*   **Worker:**
    *   `.venv/bin/python -m pytest tests/ -q` passes, including the new `tests/test_demo_ui.py`;
    *   `python -m demo_ui.server --fake --port 0` starts, prints its URL, and serves `/`, `/api/scenes` and
        `/api/sim.mjpg` (checked by a test);
    *   `phase_v7_notes.md` is written.
*   **Lead, later, with Isaac (`--gui`), the container and the model server:** one typed request runs end to
    end on screen:
    *   read;
    *   confirm;
    *   write (G1b);
    *   run with live lines, calls and results;
    *   checklist;
    *   live Isaac view;
    *   and a forced failure shows the recovery handoff live.

## Files

*   **Create:**
    *   `demo_ui/__init__.py`;
    *   `demo_ui/server.py`;
    *   `demo_ui/scenes.py` (the 14 presets; section 10.7);
    *   `demo_ui/pipeline.py`;
    *   `demo_ui/static/index.html`;
    *   `demo_ui/static/app.js`;
    *   `demo_ui/static/style.css`;
    *   `tests/test_demo_ui.py`;
    *   `docs/designs/phases/phase_v7_notes.md`.
*   **Modify:**
    *   `sim/run_scene.py`, only for the frame period (section 6);
    *   `program_runtime/runtime.py` for cancellation (section 10.1), with tests in
        `tests/test_program_runtime.py`;
    *   `program_runtime/reader.py` and `program_runtime/cli.py` for the note (section 8);
    *   `tests/test_reader.py`.

## 1. Server (`demo_ui/server.py`)

`python -m demo_ui.server [--port 8765] [--address 127.0.0.1:18100] [--sim-address ...]
[--frames DIR] [--reader-model G0] [--writer-model G1b] [--recovery-model G1d] [--fake]`

*   **Server:** a `ThreadingHTTPServer` subclass with `daemon_threads = True`, bound to `127.0.0.1` only (section 10). It serves `demo_ui/static/` at `/`, and prints `demo UI on http://HOST:PORT`.
*   **Addresses:**
    *   `--address` is robot_service. The sim service is the same port unless `--sim-address` is given: look at
        how `tests/e2e/program_check.py` / `learn/families.py` reach SimService and do the same.
    *   `--frames` defaults to `/mnt/c/isaac-sim-work/viewer_frames`, from `tests/e2e/demo_gui.py`.
*   **The model** endpoint comes from the environment (`LLM_BASE_URL`, `LLM_API_KEY`, `LLM_MODEL`). The server
    reads the repo's `.env` at start-up if it is present:
    *   `KEY=VALUE` lines;
    *   `#` comments and blank lines are skipped;
    *   one pair of surrounding quotes is stripped;
    *   a variable already in the environment is never overridden.

    Never log the key or any value from `.env`.
*   **`--fake`:** no robot, sim or model. It uses:
    *   `FakeBackend` with a fixed scene;
    *   a fake model that returns a fixed reading and program for the example tasks;
    *   a synthetic frame source (a generated image with a moving marker and a timestamp).

    This mode is for tests and for developing the page without Isaac.
*   **API** (JSON in and out unless noted). Every error answers `{"error": "..."}` with a fitting status: 400 bad
    input, 409 busy, 502 model or robot unreachable. Never a traceback.

| Method and path | Does |
|---|---|
| `GET /api/scenes` | The 14 presets of section 10.7: id, label, suggested request text. |
| `POST /api/scene {id}` | Loads the scene (`families.load_placements` with the preset's placements). Then `backend.list_objects()`, serialized to dicts → `{objects}`. 409 while a run is active. |
| `POST /api/read {text}` | `IntentReader.read_text` → `{seen, intent}`. |
| `POST /api/read_clip?note=<url-encoded text>` (raw body, `Content-Type: video/*`) | Saves the clip to `logs/demo_ui/clips/<UTC stamp>.<ext>`. Then `read_clip(path, note=note or None)` → `{seen, intent, note, frames: [data URLs of the frames the model got]}`. The note is optional and at most 1,000 characters, else 400. Maximum body 200 MB, else 413. |
| `POST /api/clarify {text}` | Continues the last reading's conversation with the owner's correction (section 9) → the same shape as `/api/read` plus `round`. 409 if there is no reading, or a run is active. At most 1,000 characters. |
| `POST /api/write {intent, confirm: true}` | Records the confirmation, then `validate_intent`. Then `ProgramWriter(model=writer).write(intent, objects)` → `{program, program_id, attempts}`. |
| `POST /api/run {program_id}` | Human-gated (see section 3). Runs the program `/api/write` returned for the confirmed intent, in a thread → `{run_id}`. 409 if a run is active, or if the id is stale or unknown. |
| `GET /api/events` | Server-Sent Events for the current or last run (section 2). A new client first gets the run's events so far. |
| `POST /api/answer {run_id, question_id, text}` | The answer to the pending question (section 10.2). 409 if it is stale, a duplicate, or there is none. |
| `POST /api/stop` | Cancels the active run (section 10.1). 409 if no run is active. |
| `GET /api/sim.mjpg` | `multipart/x-mixed-replace` JPEG stream of the newest frame in `--frames`, at up to 15 fps. A frame is sent only when a newer file exists. Frames are re-encoded to JPEG (quality 85) with cv2. Skip a file that is still being written: it fails to decode, so retry it on the next tick. |
| `GET /api/status` | `{robot, model, frames, run}` (section 10.9). The page polls it every 2 s for the status chips. |

*   **Newest frame:** the files are `frame_00000.png` …, and thousands accumulate. Keep the last index served and
    probe upward with `exists()`. Do a full `os.scandir` only on the first request, or when the probe finds
    nothing for 5 s, because Isaac restarts at index 0.
*   **Logging:** one log line per request (method, path, status, ms) to the `demo_ui` logger at INFO, plus one
    line per pipeline stage. Every run's events are also written to `logs/demo_ui/runs/<run_id>.jsonl` in the
    CLI's format.

## 2. Pipeline and events (`demo_ui/pipeline.py`)

*   A `Pipeline` class holds the clients and stubs, the current scene, and the active run. It runs one at a time,
    under a lock.
*   **A run:**
    1.  `before = truth_snapshot(sim)`;
    2.  `runtime.run(program, api, agent=RecoveryAgent(..., model=recovery), log=emit, ask=ask)`;
    3.  `after = truth_snapshot(sim)`;
    4.  `check_intent(intent, before, after)`;
    5.  a final event.

    In `--fake` mode, skip the truth snapshots and send `rules: null`.
*   **Event stream:** each runtime event goes to the SSE stream unchanged, as `event: runtime`, `data: <json>`.
    The pipeline adds its own events:
    *   `stage` `{name: read|write|run|check, state: start|done|failed, detail}`;
    *   `question` `{text}`, when `ask` is called;
    *   `final` `{ok, error, rules: {passed, detail per rule}}`.

    Look at `program_runtime/runtime.py` (`emit` calls) for the runtime event kinds the page must show:
    `line`, `call`, `result`, `handoff`, `patch`, `report`, `check` and `dry_run`.
*   **`ask`** blocks the run thread on a queue until `/api/answer` arrives, or 120 s pass. On timeout it returns
    `""`; look at what the runtime does with an empty answer and record it in the notes.

## 3. The page (`demo_ui/static/`)

It is built to be filmed: one 1920×1080 screen, nothing scrolls during a run, and the text is readable in a
video. Dark neutral background, one accent colour, a monospace face for code and calls, and a clean sans for the
rest. Use system fonts only, since there is no network.

**Layout:** three columns, 28 % / 32 % / 40 %.

*   **Left column (input and intent), top to bottom:**
    *   **Task:**
        *   a scene dropdown, with a Load button;
        *   tabs: Type (textarea and Send), Record (webcam preview; Record and Stop; then the clip plays back;
            Send), Upload (file input and Send);
        *   Record uses `getUserMedia` and `MediaRecorder` (webm) and sends the clip to `/api/read_clip`;
        *   Record and Upload each have an optional one-line note field under the clip ("Add a note, e.g. only
            the red ones"), sent as `note`. The Understood section shows the note next to the frames.
    *   **Understood:**
        *   the `seen` text;
        *   the frames the model got, as a thumbnail strip (clips only);
        *   the intent rendered as readable sentences (for example "cubes · color red → in → box · color red"),
            with a toggle to the raw JSON, which can be edited;
        *   a clarify box under the intent: "Not right? Tell it what you meant", with Send. Each round adds the
            correction and the new reading to a small history list ("round 2: only the red ones");
        *   buttons: Confirm & write, Edit JSON.
*   **Middle column (program and run):**
    *   **Program** (its own window, the largest text on screen after the robot view):
        *   the program with line numbers and syntax colouring for Python keywords, strings and calls; plain JS,
            no library;
        *   buttons: Run and Rewrite (asks the writer again).
        *   During a run, the current line is highlighted (`line` events), and a patched program replaces the
            shown one. Look at what the `patch` event carries; if it does not carry the new source, record it in
            the notes.
    *   **Call log**, under the program:
        *   one row per call, showing the function, its arguments and the result (ok or the failure message);
        *   handoffs, patches and questions are full-width highlighted rows;
        *   a question row has an inline answer box;
        *   the log auto-scrolls.
*   **Right column (the robot):**
    *   **Robot view**, at the top: the MJPEG `<img>`, or a `<video>` for Camera, with the source switch and a
        small LIVE dot that turns grey when the newest frame is older than 2 s.
    *   **Result**, underneath: the final checklist, one row per rule with a pass or fail mark, and the overall
        verdict.
*   **Run is human-gated (owner, 2026-11-02).** Nothing moves the robot unless a person presses Run.
    *   Run is enabled only when all of these hold:
        *   the intent is confirmed;
        *   a program for that confirmed intent is shown;
        *   no run is active.
    *   A new reading, a clarification, Edit JSON or Rewrite disables Run again until the new program is shown.
    *   The server enforces the same gate; the page alone is not enough:
        *   `/api/run` needs the `program_id` returned by `/api/write`, for the currently confirmed intent;
        *   a stale or unknown id is answered 409 with the reason.
        *   `/api/write` gets `{intent, confirm: true}` and records the confirmation.
    *   Neither the server nor the page ever starts a run automatically.
    *   Tests:
        *   `/api/run` without a `program_id`, with a stale one, or after a clarification → 409;
        *   the page never calls `/api/run` except from the Run button's click handler. This is a code check:
            grep `app.js`.
*   **Top bar:**
    *   the title "Show once";
    *   status chips from `/api/status`: robot, model, sim view;
    *   a Stop button, red, enabled during a run;
    *   a Reset button, which reloads the current scene.
*   **Stage indicator** across the top: Read → Confirm → Write → Run → Check. The current stage is lit and a
    failed stage turns red with its message.
*   **States:** every button shows a busy state while its request runs, and every error appears inline next to
    its section, never as `alert()`.
*   **Keyboard:** Ctrl+Enter sends the typed request.
*   **Accessibility:**
    *   labelled controls and visible focus;
    *   colour is never the only signal: pass and fail also show ✓ and ✗.

## 4. Fakes for `--fake` and tests

*   **The fake model** answers the reader prompt with a fixed valid `SEEN:` and `json` reply. The intent depends
    on keywords in the request ("box" → cubes in the red box; "sort" → sort by colour; "on" → yellow on red).
*   It answers the writer prompt with the matching `program_runtime/examples/*.py` program.
*   It returns `stop` decisions for recovery.
*   **The fake run** uses `FakeBackend`, so the runtime's real events stream.

## 5. Tests (`tests/test_demo_ui.py`)

Start the server in a thread on port 0 with `--fake`-equivalent construction, and use `urllib`. Cover:

*   `/` serves the page, and the static files load;
*   `/api/scenes` lists 14 presets;
*   read, write and run on the typed path: the events stream contains `stage`, `line`, `call`, `result` and
    `final`, in order;
*   a second `/api/run` while one is active → 409;
*   `/api/answer` with no question → 409;
*   bad JSON → 400;
*   an oversized clip body → 413 (send only the headers with a large `Content-Length`);
*   `/api/sim.mjpg`: a temporary frames directory with two PNGs written by cv2 gives two JPEG parts. A
    half-written file is skipped without error;
*   the newest-frame probe finds index 3 after 0–2, and recovers after a restart at 0;
*   `ask` → `/api/answer` round trip, with a fake agent that asks a question;
*   no traceback text in any error body.

## 6. Isaac frame period (`sim/run_scene.py`)

`GUI_FRAME_PERIOD_S` is read from the environment `ISAAC_FRAME_PERIOD_S` (default 0.2, today's behaviour), so the
demo can run at 1/15 s. Change only that line and add a one-line comment. Whether Isaac keeps real time at
15 fps is checked later by the lead.

## 9. Clarification (`program_runtime/reader.py`)

*   `Reading` gets a `messages` field: the conversation that produced it, including the accepted assistant
    reply.
*   `IntentReader.clarify(reading, correction) -> Reading`:
    *   appends `{"role": "user", "content": "That is not what I meant: <correction>. Read the task again
        with this correction and reply in the same format."}` to `reading.messages`;
    *   then the same reply, parse and one-correction loop as `_read`, with the same model, logging (source
        `"clarify"`, frame count 0) and `ReadError` wrapping;
    *   the clip's images stay in the conversation, because they are in `messages`.
*   The pipeline keeps the latest `Reading` and a round counter. A new `/api/read` or `/api/read_clip` resets
    both.
*   **Tests:**
    *   `clarify` sends the earlier turns plus the correction;
    *   it returns the new intent;
    *   a failed clarification raises `ReadError` and leaves the previous reading unchanged in the pipeline;
    *   `/api/clarify` without a reading → 409.

## 8. Clip plus note (`program_runtime/reader.py`, `program_runtime/cli.py`)

*   **Reader:** `IntentReader.read_clip(path, count=6, note=None)`.
    *   A non-empty note is appended to the clip's text part: `The person also wrote: "<note>". Use it together
        with the frames; where they disagree, the note wins.`
    *   Without a note, the message is exactly today's.
*   **Log record:** add `note: bool`.
*   **CLI:** `read --clip PATH [--note TEXT]`. `--note` without `--clip` is an argparse error.
*   **Tests:**
    *   the note is in the text part, after the images;
    *   without a note, the text part is unchanged;
    *   CLI `--note` is forwarded;
    *   `--note` with `--text` is rejected.

## 7. Notes

`phase_v7_notes.md`:
*   what was built;
*   the pytest summary line;
*   how to start it:
    *   fake mode;
    *   the real mode, with the Isaac `--gui`, container and model-server prerequisites;
*   every deviation or open question.

## 10. Review fixes (binding; where they differ from sections 1–9, these win)

### 10.1 Stop cancels the run
*   `runtime.run(..., cancel: threading.Event | None = None)`. With `None`, behaviour is exactly today's.
*   When `cancel` is set, the runtime:
    *   checks it before every API call, before any handoff or recovery, and after a call that fails;
    *   raises no new call;
    *   never hands off to the agent;
    *   ends with one `{"kind": "done", ...}` event, with `ok=False` and `error="stopped by the operator"`.

    Look at the existing `done` event and keep its fields.
*   `/api/stop`, in this order:
    1.  sets the run's event;
    2.  answers a pending question with the stop sentinel; `ask` returns `""` and the runtime sees `cancel` set;
    3.  then calls robot_service `Stop` (`FakeBackend` has none: skip it in fake mode) to halt the motion that
        is in progress.
*   **Tests:**
    *   in `test_program_runtime.py`: a cancel set during a call leaves no further calls, no agent decision, and
        the stopped `done`;
    *   in `test_demo_ui.py`: stop during a pending question ends the run and clears the active state.

### 10.2 Locking and questions
*   The pipeline lock guards only short state changes:
    *   active run;
    *   reading;
    *   confirmed intent;
    *   program id;
    *   pending question.
*   Nothing slow happens under it: no execution, no RPC, no model call, no wait and no socket write.
*   **Each question:**
    *   gets a fresh one-shot `queue.Queue(maxsize=1)` and a `question_id`;
    *   the `question` event carries `run_id` and `question_id`;
    *   `/api/answer` checks both under the lock, takes the pending slot once, and puts the text;
    *   stale or duplicate answers → 409.
*   **Timeout** (120 s) clears the slot and returns `""`.

### 10.3 Clips: bounded decoding
*   `reader.frames` reads without keeping every frame:
    1.  a first pass with `capture.grab()` counts the frames, and stops with `ValueError` above
        `MAX_CLIP_FRAMES = 3600` (about 2 min at 30 fps);
    2.  a second pass, on a fresh capture, `grab()`s up to each chosen index and `retrieve()`s only those;
    3.  frames whose longer side is above 4096 px are rejected.
*   `Reading` gets `frames: list[bytes]` (empty for text). The server returns those JPEGs as data URLs; it never
    decodes the clip twice.
*   The upload is read in 1 MB chunks up to its `Content-Length`. A missing length → 411, above 200 MB → 413, and
    a short body → 400. The saved file is deleted if the read fails.

### 10.4 Run lifecycle
*   The run thread is one `try/except/finally`. Every failure, in every stage (truth snapshot, run, check),
    becomes the final event with its stage and message.
*   **`finally`:**
    *   emits exactly one `final` if none was sent;
    *   clears the active run and the pending question;
    *   wakes the SSE waiters;
    *   closes the JSONL log.
*   **Final `ok`** = the runtime's `ok` and the rule result's `ok`; in fake mode, the runtime's alone.

### 10.5 Local-server security
*   Bind `127.0.0.1` only; there is no `--host` flag.
*   Every POST:
    *   is rejected (403) if an `Origin` header is present and is not `http://127.0.0.1:<port>` or
        `http://localhost:<port>`;
    *   needs `Content-Type: application/json` (`/api/read_clip`: one of the allowed video types), else 415;
    *   carries a JSON body of at most 64 KB, else 413.
*   **Clip types:** a fixed allowlist, `video/webm` → `webm`, `video/mp4` → `mp4`, `video/quicktime` → `mov`;
    anything else → 415. The file name is always `<UTC stamp>-<6 random hex>.<ext>`.
*   **Static files:** an explicit allowlist (`/`, `/index.html`, `/app.js`, `/style.css`); anything else → 404.

### 10.6 The result checklist
*   `check_intent` returns `RuleResult(ok, per_object, invalid_for_scene, summary)`, not per-rule results.
*   The `final` event carries `{ok, error, rules: {ok, per_object, invalid_for_scene, summary} | null}`.
*   The checklist has one row per object:
    *   the object's id, colour and category, looked up from the before snapshot;
    *   its status text;
    *   ✓ or ✗.
*   The summary sits above the rows, and the overall verdict uses the final `ok`.

### 10.7 Presets (`demo_ui/scenes.py`)
*   **The nine scripted scenes:** `cubes_in_box`, `sort_by_color` and `yellow_on_red`, each with layouts 0, 1 and
    2.
    *   Copy the deterministic layout generator and the boxes from `tests/e2e/program_check.py` into this
        module. The e2e modules are reference code, not imports.
    *   Ids: `<scene>:<layout>`.
    *   Request text: "put all the cubes in the red box" / "sort the red and blue cubes into the boxes of the
        same colour" / "put the yellow cube on the red cube".
*   **The five held-out tasks:** `cluster:100001`, `packing:100005`, `stacking:100004`, `reach:100007` and
    `mixed:100043`, through `families.generate`.
    *   Every strategy succeeded on them in the expert's plan-tier pass. They are candidates that worked
        before, not guaranteed takes.
    *   The request text is the task's intent description.
    *   The label says "held-out".
*   Every preset carries its placements; loading is `families.load_placements`.

### 10.8 SSE and MJPEG
*   **SSE:**
    *   each run has an append-only event list and a `threading.Condition`;
    *   a client keeps an index, and under the condition copies the new events or waits; it writes to the
        socket outside the lock;
    *   the stream closes after `final`;
    *   a client connected to an old run never switches to a new one; it reconnects and gets the current run;
    *   a disconnect (`BrokenPipeError`, `ConnectionResetError`) ends the handler quietly.
*   **MJPEG:**
    *   one shared frame watcher thread finds the newest frame (probe upward; full scan on start, and after 5 s
        without a new frame, since Isaac deletes the old frames on restart);
    *   it encodes the frame to JPEG once, and stores it with a generation counter;
    *   each client keeps its own last generation and sends only newer frames, at up to 15 fps.

### 10.9 Status
*   **Robot:**
    *   one cached probe at a time, with a 2 s deadline;
    *   at most every 2 s;
    *   requests in between get the cached value;
    *   statuses: `ok`, `error` (with the message) or `unknown` before the first probe.
*   **Model:** `unknown` until the first model call, then the last call's outcome.
*   **Frames:** the newest frame's age in seconds, or `null`.
*   **Run:** `idle`, `running` or `waiting` (a question is pending).

### 10.10 Runtime events the page handles
*   **Event kinds:** `check`, `dry_run`, `line`, `call`, `result`, `report`, `failure`, `handoff`, `decision`,
    `patch` and `done`. Read `program_runtime/runtime.py` for their fields.
*   **Patched program:** the page caches `decision.patch` (the new source). It replaces the shown program only
    when the next `patch` event has `accepted: true`.

### 10.11 Tests (in addition to section 5)
*   **Deterministic holds:** a fake backend whose call waits on a test latch holds a run open (for the second-run
    409, stop and the question tests).
*   **MJPEG:**
    *   connect with one frame present, then write the next; that gives two parts in order;
    *   then delete the sequence and write `frame_00000.png`; the stream continues.
*   **Body errors:** missing, invalid JSON, short body, oversized body, wrong content type, foreign `Origin`.
*   **Cleanup:** an injected `truth_snapshot` failure gives one `final` with the stage, and the next run can
    start.
*   **No tracebacks:** server logging is captured with `caplog`; no `Traceback` in the logs or in any body.

## 11. Second review fixes and patch approval (binding)

### 11.1 Stop wins over success
*   The runtime also checks `cancel` after every API call and agent call returns (success or failure), and once
    more just before it records success. A set cancel always ends as the stopped `done`.
*   After a stopped run, the pipeline skips the after-snapshot and the rule check, and the `final` event is
    `{ok: false, error: "stopped by the operator", rules: null}`.
*   **Test:** Stop while the last call is latched; the call then returns successfully; the result is still
    stopped.

### 11.2 Revisions bind the program to the scene and the intent
*   The pipeline keeps a `scene_rev`, which Load and Reset increment, and an `intent_rev`, which a reading, a
    clarification, Edit JSON or a confirm increments.
*   Load and Reset also clear the confirmed intent and the program.
*   `/api/write` captures both revisions at its start, and commits its program only if both are unchanged at the
    end; otherwise it returns 409 with "the scene or intent changed while writing".
*   A `program_id` is bound to both revisions, and `/api/run` checks them.
*   **Tests:** Load after write → `/api/run` 409; Load during a latched write → that write returns 409.

### 11.3 Clarify never mutates the earlier reading
`clarify` works on a copy of `reading.messages`, and returns a new `Reading` only after a valid reply. A failure
leaves the input `Reading` and the pipeline's current reading untouched.

### 11.4 Frame-size check before decoding
*   Before either pass, the capture's reported width and height (`CAP_PROP_FRAME_WIDTH`/`HEIGHT`) must be
    positive and at most 4096, else `ValueError`.
*   The shape check after `retrieve()` stays.
*   **Test:** a clip with too large dimensions is rejected before any frame is retrieved.

### 11.5 The verification trail
*   An `attempt_id` is created at each new `/api/read` or `/api/read_clip`. It is returned to the page and kept
    through clarification, confirm, write and run.
*   Every step appends an immutable record (kind, time, inputs, outputs) to
    `logs/demo_ui/attempts/<attempt_id>.jsonl`, written as it happens:
    *   reading;
    *   clarification (correction and new reading);
    *   confirm;
    *   program (id, source, attempts);
    *   rewrite;
    *   run start (`run_id`);
    *   patch approval or decline;
    *   final.
*   The run's own JSONL stays as specified; its first line names the `attempt_id`.

### 11.6 Fake-mode scenes
In `--fake`, Load builds the `FakeBackend` objects directly from the preset's placements: use
`families.fake_objects` for held-out tasks, and the same object construction for the scripted presets. No RPC is
made. Test: load one scripted and one held-out preset in fake mode.

### 11.7 Recovery patches need the owner's approval (owner, 2026-11-02)
*   `runtime.run(..., approve: Callable[[str, str], tuple[bool, str]] | None = None)`. With `None`, behaviour is
    exactly today's.
*   **When set:** after a `patch` decision passes `check` and the budget, and before it is applied, the runtime:
    1.  emits `{"kind": "approval", "patch": <source>, "reason": <decision reason>}`;
    2.  calls `approve(patch_source, reason)`.
*   **Approved:** today's accepted path.
*   **Declined:** the `patch` event has `accepted: false` and `problems: ["declined by the operator: <note>"]`,
    and the run fails through today's rejected-patch path. The owner can then clarify, rewrite and run again.
*   **Cancel** during approval counts as a stop (11.1).
*   **In the pipeline,** `approve` works like `ask` (10.2):
    *   one-shot queue;
    *   `question_id`;
    *   `POST /api/approve {run_id, question_id, approved: bool, note?}`;
    *   409 if stale or a duplicate;
    *   no timeout; the run waits for the owner or for Stop;
    *   run status `waiting`.
*   **On the page:**
    *   the call log shows an approval row with the reason;
    *   the Program window shows the patch as a diff against the current program: removed lines struck through
        in red, added lines in green, each also marked with − or +;
    *   buttons: Approve patch, Decline (with an optional note);
    *   the stage indicator shows "Waiting for you".
*   **Only patches change the program,** so only patches are gated. Retry, skip and re-observe repeat or drop a
    step of the program the owner already approved. They show in the call log as today.
*   **Tests:**
    *   in `test_program_runtime.py`: approve → the patch is applied; decline → a rejected patch with the
        operator's note, and the run fails; `None` → unchanged;
    *   in `test_demo_ui.py`: the approval round trip; a stale approval → 409; stop while waiting.
