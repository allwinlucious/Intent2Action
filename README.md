# intent-to-motion

[![CI](https://github.com/allwinlucious/intent-to-motion/actions/workflows/ci.yml/badge.svg)](https://github.com/allwinlucious/intent-to-motion/actions/workflows/ci.yml)
[![Model: program writer](https://img.shields.io/badge/%F0%9F%A4%97%20model-program%20writer%20(G1b)-yellow)](https://huggingface.co/mewbot97/show-once-glimmer-programs)
[![Model: recovery agent](https://img.shields.io/badge/%F0%9F%A4%97%20model-recovery%20agent%20(G1d)-yellow)](https://huggingface.co/mewbot97/show-once-glimmer-recovery)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue)](LICENSE)

**Show a robot a task once, as a short video, a sentence, or both. A fine-tuned open model turns it into explicit
rules and a short, checkable robot program. A person approves, and an industrial robot stack executes it with
collision-checked, force-limited motion.**

The model never moves the robot directly. It writes a program against a small robot API. The runtime checks and
dry-runs the program before anything moves, executes it with a budgeted recovery ladder, and verifies the result
against ground truth. A person signs off at three gates:
1.  confirm the intent;
2.  press Run;
3.  approve every recovery patch.

## Results

The model is Muse Glimmer 30B, fine-tuned with QLoRA on what the robot actually achieved in simulation ("learning
from experience").

| | base Glimmer | fine-tuned (G1d, the one model used) |
|---|---|---|
| Programs that succeed, 250 held-out tasks × 3 runs (planning simulator) | 61% [56–67] | **83% [79–87]** |
| Isaac Sim physics, 50 held-out tasks | 34/50 | **42/50** |
| Live recovery: failed runs rescued, 20 tasks | 4/20 | **9/20** |

*   Intervals are 95% bootstrap over tasks.
*   The best program writer, G1b, reaches 86% [82–89].
*   References: the best fixed strategy per task family reaches 84%; an oracle that picks the best strategy per
    scene reaches 91%.
*   Every number, the ablations (rejection sampling, DPO, GRPO, bf16 LoRA, seeds), what did not work, and the
    limitations: [docs/designs/learning_results.md](docs/designs/learning_results.md).

**Trained adapters on Hugging Face**, Apache-2.0, for `meta-models/Muse-Glimmer-30B`:
*   [mewbot97/show-once-glimmer-recovery](https://huggingface.co/mewbot97/show-once-glimmer-recovery) (G1d): writes
    programs and handles recovery. This is the model the demo uses.
*   [mewbot97/show-once-glimmer-programs](https://huggingface.co/mewbot97/show-once-glimmer-programs) (G1b): the best
    program writer.

## How it works

```
demo clip / sentence ──► reader (Glimmer, images + text; speech via Muse Voice Transcribe)
                         └► explicit rules ("every cube → into the red box")      ◄── person confirms or corrects
                              └► program writer (fine-tuned Glimmer)
                                   └► short Python program (find / pick / place / verify)  ◄── person presses Run
                                        └► runtime: static check → dry run → execute
                                             ├► robot service (gRPC): Intrinsic Core world model, motion and grasp
                                             │   planning, ICON real-time control ⇄ ROS 2 ⇄ Isaac Sim (Franka)
                                             └► on failure: budgeted recovery (retry / skip / patch / ask / stop)
                                                  └► patches                         ◄── person approves each one
                                                       └► rule checker on simulator ground truth: ✓ / ✗
```

| Part | Where |
|---|---|
| Robot API and runtime (check, dry run, recovery ladder, rules) | [`program_runtime/`](program_runtime) |
| Robot service (gRPC API over Intrinsic Core; Pick, Place, force moves) | [`container/services/`](container/services), [`robot_service/robot_service.proto`](robot_service/robot_service.proto) |
| Intrinsic Core in one container | [`container/`](container) ([README](container/README.md)) |
| Isaac Sim scene, ROS 2 bridge, video look | [`sim/`](sim), [`ros_ws/`](ros_ws) |
| MCP server (every robot RPC as a tool) | [`mcp_server/`](mcp_server) ([README](mcp_server/README.md)) |
| Demo panel (human gates, live view, approvals) | [`demo_ui/`](demo_ui) |
| Learning: task families, expert, training, evaluation | [`learn/`](learn), [`runpod/`](runpod) |
| Designs, plans and results | [`docs/designs/`](docs/designs) |

## Quick start (no GPU, no simulator)

```bash
git clone https://github.com/allwinlucious/intent-to-motion && cd intent-to-motion
make setup          # .venv with requirements-dev.txt (Python 3.12)
make test           # 334 unit tests, about 1 min
make ui-fake        # demo panel with a fake robot and model: http://127.0.0.1:8765
make tables         # rebuild the results tables from the published data in results/
```

## Full setup

| Level | Needs | Gives |
|---|---|---|
| Unit tests, fake panel, result tables | Python 3.12 | everything above |
| Planning simulator ("plan tier") | Docker; the Intrinsic Core image (`container/README.md`) | real motion and grasp planning without physics; the evaluation fleet (`learn/fleet.py`) |
| Physics | NVIDIA Isaac Sim 5.1 (Windows host, WSL2 with mirrored networking), an NVIDIA GPU | `tests/e2e/*`, Isaac evaluations, demo takes |
| Models | a GPU with about 40 GB (RunPod, `runpod/setup.sh`) or the Meta Model API | training, generation, the live panel |

Step-by-step commands for every experiment are in [docs/REPRODUCE.md](docs/REPRODUCE.md).

## Repository layout

```
program_runtime/  robot API, runtime, intent reader, rules        demo_ui/      the demo panel
container/        Intrinsic Core container and robot service      sim/          Isaac Sim scene and tools
learn/            task families, expert, SFT/DPO/GRPO, evaluation  runpod/       GPU pod scripts
mcp_server/       MCP tools generated from the proto              tests/        unit tests; tests/e2e: stack checks
results/          published experiment data (restore: make tables) docs/        designs, phase plans, results
```

## Limitations

*   Synthetic tabletop tasks: cubes, boxes and plates, in five generated families.
*   No real robot, no perception noise: the program gets the true object list.
*   The model decides the strategy (the order of picks and places); grasps and motion stay with the robot stack.
*   Physics evaluations cover 50 tasks.
*   This is not certified functional safety. A real cell still needs its own safety layer: guarding, safety-rated
    stops and speed monitoring, and a risk assessment.

## License

Apache-2.0 ([LICENSE](LICENSE)); third-party parts keep their own licences ([NOTICE](NOTICE)).
