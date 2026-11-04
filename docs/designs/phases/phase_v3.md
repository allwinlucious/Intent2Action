# Phase V3: reading the intent from a demo clip (or a typed sentence)

Part of [demo_to_api.md](../demo_to_api.md) ("Reading a demo video", V3 and V5). The owner's decision of
2026-11-02 replaces the SAM 3.1 step: **frames go straight to Glimmer**. SAM is added later only if the clip reader
misreads the owner's clips. Glimmer takes several images per message, and its model card handles video as
individual frames.

The chain this phase completes:

```
clip.mp4 ──frames──▶ Glimmer (base, "G0") ──▶ intent JSON ──▶ (owner confirms) ──▶ cli run --write --recover
"sentence" ─────────▶ Glimmer (base, "G0") ──┘                                     (G1b writes, G1d recovers)
```

Reading uses the base model. The adapters were trained only on program writing and recovery, and the server
switches adapters per request (`model` field).

## Constraints for the worker

*   **No heavy compute:** no Isaac Sim, no Docker, no GPU, no model downloads. Do not run `tests/e2e/*`.
*   **Allowed:**
    *   editing files;
    *   `.venv/bin/python -m pytest tests/ -q`;
    *   `py_compile`;
    *   `python -m program_runtime.cli --help`.
*   **No network calls.** Tests use fakes; the real model runs later on the GPU host.
*   No new dependencies. `.venv` already has `opencv-python-headless`, `pillow` and `numpy`. `learn/chat_server.py`
    runs on the GPU host, which has transformers 5, pillow and torch.
*   Do not modify `/home/mew/intrinsic-core`. No `git commit`. No `pkill -f`.
*   Make no design decisions beyond this plan. Record anything not covered in `phase_v3_notes.md` and stop that
    part.
*   **Code style:** 2-space indent; comments only for the why.

## Done criteria

*   **Worker:**
    *   `.venv/bin/python -m pytest tests/ -q` passes, including the new `tests/test_reader.py` and the extended
        `tests/test_chat_server.py`;
    *   `python -m program_runtime.cli read --help` prints the usage;
    *   `phase_v3_notes.md` is written.
*   **Lead, later, on the GPU host and in a sim window:**
    *   the reader returns a valid intent for the three example tasks, typed as sentences;
    *   the reader returns a valid intent for at least one real or Isaac-rendered clip;
    *   the intent then runs end to end with `--write --recover` (G1b as writer).

## Files

*   **Create:**
    *   `program_runtime/reader.py`;
    *   `tests/test_reader.py`;
    *   `docs/designs/phases/phase_v3_notes.md`.
*   **Modify:**
    *   `program_runtime/cli.py`: the `read` subcommand, plus `--writer-model` / `--recovery-model` on `run`;
    *   `program_runtime/agent.py`: `ChatClient.chat(..., model=None)`, a per-call model override (section 2);
    *   `learn/chat_server.py`: image input;
    *   `tests/test_chat_server.py`.

## 1. Frames (`reader.frames`)

*   `frames(path, count=6, max_side=768) -> list[bytes]`, with `count >= 2` (else `ValueError`): JPEG bytes of `count` frames spaced evenly from the
    first frame to the last, both included. Each frame is resized so its longer side is at most `max_side`.
    Uses `cv2.VideoCapture`.
*   **Errors:** an unreadable file, or one with fewer than 2 frames, raises `ValueError` with the path and the
    reason.
*   **Why the first and last frames:** the filming protocol starts and ends with the hands out of view, so
    those two frames are the before and after.

## 2. The reader (`reader.IntentReader`)

*   `IntentReader(client, model="G0")`;
*   `read_clip(path, count=6) -> Reading` (passes `count` to `frames`) and `read_text(sentence) -> Reading`;
*   `Reading` is a dataclass with three fields:
    *   `intent`: a dict that passes `rules.validate_intent`;
    *   `seen`: the text after the first line starting with `SEEN:`, up to the json fence, stripped;
    *   `raw`: the assistant content string of the accepted reply.
*   **Model routing:** `ChatClient.chat` gains `model: str | None = None`. A non-`None` value is set in the request
    body **after** `self.extra` is merged, so it wins over `LLM_EXTRA_BODY`. Every reader call, including the
    corrective turn, passes `self.model`. Without the argument, the body is exactly today's.
*   **Errors:** `ReadError(message, problems: list[str], replies: list[str])`. The reader wraps three kinds of
    failure in it:
    *   frame `ValueError`;
    *   any exception from `client.chat` (timeouts, URL errors, malformed responses), with the cause chained;
    *   two invalid replies.

### The message

*   **System prompt**, defined in `reader.py`:
    *   the intent schema exactly as `rules.validate_intent` accepts it: root keys `rules`, `description` and
        `others: "stay"`; each rule is `select` / `relation` (`in` or `on`) / `target`;
    *   the selector keys and values `_validate_filter` allows (category, color, size small/medium/large);
    *   the three files in `program_runtime/examples/*.json` as worked examples;
    *   the instruction to name boxes and targets by attributes, never by position or id.
*   **The reply format:** a line starting with `SEEN:` and the account, then the intent in one ```json fenced
    block.
*   **The user message for a clip** (OpenAI content parts):
    *   one `{"type": "image_url", "image_url": {"url": "data:image/jpeg;base64,..."}}` part per frame, in order;
    *   then a text part: "Frames of a person demonstrating a tabletop task, in order; the first is before, the
        last after. ..."
*   **The user message for a sentence:** the sentence as text, with no images.

### Parsing and correction

*   Take the last ```json block and `json.loads` it. Then run `validate_intent`.
*   A missing or empty `SEEN:` is also a problem, and is corrected in the same turn.
*   **The schema in the system prompt:** `rules` is required. `description` and `others` are optional under
    `validate_intent`, but the reader asks for both: a short description, and `others: "stay"`.
*   On a parse error or schema problems, send **one** corrective turn: the problems as a list, and "reply again
    in the same format". This is the same pattern as `ProgramWriter`'s corrective turn; look at it and reuse its
    helpers where they fit.
*   If the second reply also fails, raise `ReadError` with the second reply's problems and both replies.

### Logging

*   Each `read_clip` / `read_text` call logs exactly one record to the `program_runtime.reader` logger, on success
    and on failure. The record holds the source (clip path or "text"), the frame count (0 for text), latency in
    seconds, and `ok` or the problems / error.

## 3. CLI

*   `python -m program_runtime.cli read (--clip PATH | --text SENTENCE) [--frames 6] [--model G0] [--out intent.json]`:
    *   prints the `SEEN:` account and the intent (pretty JSON) to stdout;
    *   writes the intent to `--out` if given;
    *   exit code 0 on success;
    *   exit code 1 on `ReadError`, with the message and problems on stderr and no traceback;
    *   `--frames` below 2 is rejected by argparse.
*   `run` gets `--writer-model` and `--recovery-model`, both defaulting to `None`, which keeps today's behaviour
    (`LLM_MODEL`). The CLI still builds one `ChatClient`:
    *   `ProgramWriter` and `RecoveryAgent` each get an optional `model` and pass it on every `chat` call;
    *   `read` builds its own `ChatClient` and an `IntentReader` with `--model`.
*   The owner's confirm step belongs to the demo UI (V7), not to this CLI. `read` then `run --intent` is the
    command-line form.

## 4. Image input in `learn/chat_server.py`

*   A message `content` may be a list of parts:
    *   `{"type": "text", "text"}`;
    *   `{"type": "image_url", "image_url": {"url": "data:image/...;base64,..."}}`.
*   Only data URLs are accepted. Any other URL is answered 400 with the reason: the server never fetches from the
    network.
*   `template_messages` turns image parts into the cookbook form `{"type": "image", "image": <PIL.Image>}`. Text
    parts stay in order.
*   **When any message has an image:** build the inputs with
    `processor.apply_chat_template(messages, tools=..., add_generation_prompt=True, tokenize=True,
    return_dict=True, return_tensors="pt", **kwargs)` and move them to CUDA.
*   **Text-only requests keep today's exact path**, because the evaluated results were produced through it.
*   Decoding the reply and the stop tokens are unchanged.
*   **Tests:**
    *   data-URL decoding, and the rejection of other URLs;
    *   `Model.chat` with a fake processor, tokenizer and model (build `Model` with `object.__new__` and set the
        attributes `chat` uses), on both branches:
        *   an image request goes through `apply_chat_template(tokenize=True, return_dict=True)`, and the image
            tensors reach `generate`;
        *   a text-only request still calls `apply_chat_template(tokenize=False)`, then `tokenizer(...)`, with
            the same prompt slicing and decoding as today.

## 5. Tests (`tests/test_reader.py`)

*   **`frames`:** write a tiny synthetic video with `cv2.VideoWriter` to `tmp_path` (MJPG/AVI, 20 frames, each
    frame a distinct solid colour). Assert the count, that the first and last frames are included (compare
    colours), and the resize.
*   **The reader with a fake client:**
    *   a valid first reply;
    *   an invalid first reply followed by a valid corrective reply (assert the corrective turn lists the
        problems);
    *   two invalid replies, which raise `ReadError`;
    *   a clip request that carries `count` image parts before the text part.
*   **Model routing:**
    *   `ChatClient.chat(model=...)` puts that model into the HTTP body even when `LLM_EXTRA_BODY` contains
        `model` (patch `urllib.request.urlopen`);
    *   the reader's corrective turn passes the model too;
    *   `ProgramWriter` and `RecoveryAgent` send their own model names.
*   **CLI**, in process: monkeypatch `program_runtime.cli.ChatClient` with a fake, then call
    `cli.main([...])` and use `capsys`. The existing CLI test runs a subprocess and cannot inject a client.
    Cases:
    *   `read --text` succeeds and writes `--out`;
    *   an unreadable clip gives exit 1 and a message on stderr, with no traceback;
    *   a client that raises gives exit 1;
    *   `--frames 1` is rejected;
    *   `--frames 4` reaches `frames` as `count=4`.
*   **Logging:** `caplog` tests check one record for a successful read and one for a failed read, each with the
    source, frame count, latency and outcome.
*   **The two-failure test** asserts the reported problems and both raw replies.
