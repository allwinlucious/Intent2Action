# Demo runs: the record behind every take in the video

The model calls and robot runs behind the video's takes are logged here, unedited.

| File | What it proves |
|---|---|
| `transcript.jsonl` | every model call of the recording session, one JSON line each: the endpoint (`G0` is base Glimmer, `G1d` the fine-tuned model, `G1e` G1d + push experience), the exact messages sent, the raw reply, token usage and timing. Written by the chat client when `LLM_TRANSCRIPT` is set; clip frames are stored as their SHA-256, not the image |
| `<take>.program.py` | the program the robot ran in that take |
| `<take>.events.jsonl` | the runtime's events for that take: every robot call, its result, failures, recovery decisions (`tools/record_take.py`) |

Programs and replies of the held-out evaluations are in `results/` (`python tools/results.py restore`).

Takes not driven by a model are named as such: `finale_push_scripted.*` (the robot's force-limited moves, scripted),
`gift_stack_scripted_*` (hand-written test programs), and the scripted takes of `tools/record_scripted.py`. The
panel's own verification trail (reading, clarifications, confirm, program, run, approvals) is written to
`logs/demo_ui/attempts/` when the panel runs.
