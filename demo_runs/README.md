# Demo runs: the record behind every take in the video

Every model call and every robot run shown in the video is logged here, unedited.

| File | What it proves |
|---|---|
| `transcript.jsonl` | every model call of the recording session, one JSON line each: the endpoint (`G0` is base Glimmer, `G1d` the fine-tuned model), the exact messages sent, the raw reply, token usage and timing. Written by the chat client when `LLM_TRANSCRIPT` is set; clip frames are stored as their SHA-256, not the image |
| `<take>.program.py` | the program the robot ran in that take |
| `<take>.events.jsonl` | the runtime's events for that take: every robot call, its result, failures, recovery decisions (`tools/record_take.py`) |
| `attempts/*.jsonl` | the demo panel's verification trail: reading, clarifications, confirm, program, run, every approval or decline (copied from `logs/demo_ui/attempts/`) |

Programs and replies of the held-out evaluations are in `results/` (`python tools/results.py restore`).
