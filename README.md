# Intent2Action

[![CI](https://github.com/allwinlucious/Intent2Action/actions/workflows/ci.yml/badge.svg)](https://github.com/allwinlucious/Intent2Action/actions/workflows/ci.yml)
[![Models on Hugging Face](https://img.shields.io/badge/%F0%9F%A4%97%20Hugging%20Face-models-yellow)](https://huggingface.co/collections/mewbot97/intent2action-6aeb96029fcffdc5134cd261)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue)](LICENSE)

**Show a robot a task once, with a short phone video, a sentence, or both, and it does the task in a new scene.**

<p align="center">
  <img src="docs/media/gift_pack.gif" width="480" alt="The robot puts the small heart box into the grey box and closes the lid, in Isaac Sim">
  <br><sub>From a phone clip of a person packing a gift box: the model read the task, a person corrected it once, and the
  robot ran the model's program in Isaac Sim physics (2× speed).</sub>
</p>

Programming an industrial robot for a new task usually takes an integrator days. Intent2Action lets the person who
knows the task show it instead:
1.  **Read:** an open model (Muse Glimmer) watches the clip or reads the sentence and turns it into explicit rules,
    such as "every cube → into the red box". The person checks and corrects them.
2.  **Write:** a version of the same model, fine-tuned on the robot's own experience, writes a short Python program
    against a small robot API (`find`, `pick`, `place`, `verify`).
3.  **Run:** the program is checked and dry-run before anything moves. Nothing moves until the person presses Run.
    An industrial robot stack (Intrinsic Core) executes it with collision-checked, force-limited motion, here on a
    Franka arm in NVIDIA Isaac Sim.
4.  **Recover:** if a step fails, the model proposes a fix: a retry, a corrected program, or a question to the person.
    Every changed program and every robot move it wants to make needs the person's approval.
5.  **Verify:** an independent checker compares the result with the simulator's ground truth.

The model never steers the robot on its own.

## In action

| Learning from experience | Experience beats size |
|---|---|
| <img src="docs/media/tower_untrained_vs_finetuned.gif" width="400" alt="The base model pulls the bottom cube and fails; the fine-tuned model unstacks top first"> | <img src="docs/media/experience_wonder_vs_ours.gif" width="400" alt="Muse Wonder finds no feasible grasp; the fine-tuned model takes the outermost cube first"> |
| Unloading a tower. The base model grabs the bottom cube and fails (left); after training on the robot's experience, it unstacks top first (right). 3× speed. | Cubes packed close together. Muse Wonder, a much larger hosted model, writes a correct-looking loop, but the fingers fit around none of its first choices (left). Ours takes the outermost cube first (right). 3.5× speed. |
| **A new skill, also learned from experience** | **Held-out tasks it solves in physics** |
| <img src="docs/media/push_finale.gif" width="400" alt="The carton is too wide to grasp; the model asks, then pushes it onto the green zone"> | <img src="docs/media/success_grid.gif" width="400" alt="Sixteen successful tasks"> |
| The carton is wider than the gripper opens. The model says so and asks; told "can you get it done somehow?", it pushes the carton onto the zone with the robot's force-limited moves, each move approved. It learned pushing on other objects and had never seen this carton. 10× speed. | Sixteen takes at once: fourteen tasks the models' programs solved in Isaac Sim physics, and two scripted capability takes (peg in hole, gift tower). 6× speed. |

All takes are recorded at 30 fps in real time. The model calls behind them are logged in [`demo_runs/`](demo_runs).

## Try it

**1. In two minutes, no GPU: the demo panel, answering with the real recorded model replies.**

```bash
git clone https://github.com/allwinlucious/Intent2Action && cd Intent2Action
make setup        # .venv (Python 3.12)
make ui-replay    # then open http://127.0.0.1:8765
```

1.  Choose the scene **Heart box · pack and close**.
2.  Type `put the small heart box into the open box and close the lid`. The model's recorded reading appears (it
    misses the lid).
3.  Correct it: `the small heart box is a gift, not a box: the mauve gift goes into the box, then the lid goes on the
    box`.
4.  Confirm: the fine-tuned model's recorded program appears. Press ▶.

Every reply comes from `demo_runs/transcript.jsonl`, replayed exactly, so only recorded requests work. The robot here
is a simple stand-in without physics.

**2. With the real models**, on any GPU host with about 40 GB of memory and CUDA ≥ 13 (RunPod, Modal, a workstation):

```bash
bash runpod/serve_from_hf.sh      # Muse Glimmer 30B + the trained adapters from Hugging Face, on port 8000
LLM_BASE_URL=http://<gpu-host>:8000/v1 LLM_API_KEY=unused make ui-fake     # the panel, asking the real models
```

**3. Check the numbers:** `make test` (346 unit tests, about a minute) and `make tables` (rebuilds every result table
from the published data in [`results/`](results)).

**4. The full stack**, with the robot software in Docker and Isaac Sim physics: [docs/REPRODUCE.md](docs/REPRODUCE.md).

## Results

The base model is **Muse Glimmer 30B**, an open-weights model. We fine-tuned it with QLoRA on what the robot actually
achieved in simulation: for 600 generated training tasks, every strategy was tried on a real motion planner, and the
model learned from the ones that worked. All tasks below are held out: never seen in training, and one whole task
type never trained at all.

| | base model | **fine-tuned** |
|---|---|---|
| Programs that succeed: 250 tasks × 3 runs, motion planner | 61% [56–67] | **83% [79–87]** |
| Tasks solved in Isaac Sim physics, 50 tasks | 34/50 | **42/50** |
| Failed runs rescued by the model's recovery, 20 tasks | 4/20 | **9/20** |

**Against much larger hosted models**, through exactly the same pipeline (same prompt, temperature 0, one attempt):

| | motion planner, 250 × 3 | Isaac Sim physics, 50 |
|---|---|---|
| Muse Spark 1.3 | 71% [67–76] | 37/50 |
| Muse Wonder | 76% [72–81] | 37/50 |
| **fine-tuned Muse Glimmer 30B** | **83% [79–87]** | **42/50** |

*   **Margins:** the fine-tuned model is 6.8 points ahead of Muse Wonder (paired 95% interval +2.7 to +10.9) and 11.5
    ahead of Muse Spark 1.3.
*   **Where the large models lose:** they write correct programs, but don't know what this robot can physically
    grasp, for example a cube wedged in a cluster.
*   **Upper bounds:** the best fixed strategy per task type reaches 84%; an oracle choosing the best strategy for each
    scene reaches 91%.

**A new skill, pushing:** we added experience of objects the gripper cannot hold (crates, trays and totes, pushed onto
a mat or a pad).
*   **Results:** on a carton and a goal zone it had never seen, the model said it could not grasp the carton and asked,
    in 15 of 15 scenes. After a yes, it pushed the carton onto the zone in 11 of 15. Without that experience: 0 of 10.
*   **The cost:** this version lost some stacking skill (89% → 58%; overall 77%). Balancing the training mix is the
    next step.

Every number, the ablations (rejection sampling, preference training, reinforcement learning, seeds), what did not
work and the limitations are in [docs/designs/learning_results.md](docs/designs/learning_results.md). Intervals are
95% bootstrap over tasks.

**The models**, Apache-2.0 adapters for [Muse Glimmer 30B](https://huggingface.co/meta-models/Muse-Glimmer-30B), in
one [Hugging Face collection](https://huggingface.co/collections/mewbot97/intent2action-6aeb96029fcffdc5134cd261):

| Model | Used for | Name in the results docs |
|---|---|---|
| [Intent2Action-glimmer-recovery](https://huggingface.co/mewbot97/Intent2Action-glimmer-recovery) | the fine-tuned model above: writes the programs, handles recovery | G1d |
| [Intent2Action-glimmer-push](https://huggingface.co/mewbot97/Intent2Action-glimmer-push) | the same plus push experience (the carton) | G1e |
| [Intent2Action-glimmer-programs](https://huggingface.co/mewbot97/Intent2Action-glimmer-programs) | the best program writer alone (86% [82–89]) | G1b |

## How it is built

```
phone clip / sentence ──► reader: Muse Glimmer (video frames + text; speech via Muse Voice Transcribe)
                          └► explicit rules                                    ◄── the person confirms or corrects
                               └► program writer: the fine-tuned model
                                    └► short Python program (find / pick / place / verify)  ◄── the person presses Run
                                         └► runtime: static check → dry run → execute
                                              ├► robot service (gRPC) on Intrinsic Core: world model, motion and grasp
                                              │   planning, real-time control ⇄ ROS 2 ⇄ Isaac Sim (Franka arm)
                                              └► on failure: recovery (retry / skip / corrected program / ask / stop)
                                                   └► every corrected program and model-issued move  ◄── approved
                                                        └► checker on the simulator's ground truth: ✓ / ✗
```

| Part | Where |
|---|---|
| Robot API and runtime: check, dry run, recovery, rule checker | [`program_runtime/`](program_runtime) |
| Robot service: gRPC API over Intrinsic Core (pick, place, force-limited moves) | [`container/services/`](container/services), [`robot_service/robot_service.proto`](robot_service/robot_service.proto) |
| Intrinsic Core in one Docker container | [`container/`](container) ([README](container/README.md)) |
| Isaac Sim scene, ROS 2 bridge, the video look | [`sim/`](sim), [`ros_ws/`](ros_ws) |
| MCP server: every robot call as a tool | [`mcp_server/`](mcp_server) ([README](mcp_server/README.md)) |
| Demo panel: the person's checkpoints, live view, approvals | [`demo_ui/`](demo_ui) |
| Learning: task generator, expert, training, evaluation | [`learn/`](learn), [`runpod/`](runpod) |
| Designs, plans and results | [`docs/designs/`](docs/designs) |

**What each level of setup needs:**

| Level | Needs | Gives |
|---|---|---|
| Tests, demo panel, result tables | Python 3.12 | the "Try it" steps 1 and 3 |
| Motion planner | Docker and the Intrinsic Core image ([`container/README.md`](container/README.md)) | real motion and grasp planning without physics; the evaluation fleet |
| Physics | NVIDIA Isaac Sim 5.1 (Windows host, WSL2 with mirrored networking) and an NVIDIA GPU | the end-to-end checks, physics evaluations, the video takes |
| Models | a GPU with about 40 GB, or the Meta Model API for the hosted models | training, generation, the live panel |

## Limitations

*   **The tasks:** synthetic tabletop tasks (cubes, boxes, plates, gift boxes) in generated families.
*   **No real robot yet, and no perception noise:** the program gets the true object list from the simulator.
*   **What the model controls:** the strategy, meaning which object, in which order, where to. Grasps and motion stay
    with the robot stack.
*   **Physics coverage:** the physics evaluations cover 50 tasks.
*   **Safety:** this is not certified functional safety. A real cell still needs its own safety layer: guarding,
    safety-rated stops, speed monitoring and a risk assessment.

## Prior work and what was built

*   **Before 27 October 2026** (the first commit, "Snapshot before implementation"):
    *   a build of Intrinsic Core packaged in one Docker container: build scripts, Dockerfile, generated protobuf
        stubs and a motion planner config;
    *   one early Isaac Sim pick-and-place script;
    *   a plan document.

    No robot service, models, learning or demo existed yet.
*   **Built from 27 October to 4 November 2026** (every later commit):
    *   the robot service and the ROS 2 bridge to Isaac Sim;
    *   the MCP server;
    *   the program runtime, the rule checker and the intent reader;
    *   the task generator, the expert and the push experience;
    *   the training and evaluation pipeline, and the trained models;
    *   the hosted-model benchmark;
    *   the demo panel, the textured props and the video takes;
    *   CI.
*   **Third-party work**, credited in [NOTICE](NOTICE):
    *   Intrinsic Core;
    *   NVIDIA Isaac Sim;
    *   the Franka model;
    *   ROS 2 and MoveIt;
    *   Muse Glimmer and its fine-tuning cookbook;
    *   the Meta Model API models.

## License

Apache-2.0 ([LICENSE](LICENSE)). Third-party parts keep their own licences ([NOTICE](NOTICE)).
