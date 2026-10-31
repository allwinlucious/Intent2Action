# Failure detection, recovery, and execution-grounded fine-tuning for robot agents

_Search and metric snapshot: 2026-10-31. “Citations” and “influential citations” below are Semantic Scholar counts retrieved on that date; stars and Hugging Face (HF) upvotes are likewise point-in-time counts. A promised future release is not counted as released code._

## 1. Search log

### Scope and queries

I searched the requested 2026-04-01 through 2026-10-31 submission window with the arXiv API, sorted newest first. The four productive broad queries were:

1. `"failure recovery" AND (robot OR robotic) AND (LLM OR VLM OR "language model")` — 11 hits returned.
2. `("error recovery" OR "failure detection") AND (robot OR robotic) AND (agent OR LLM OR VLM)` — 18 hits returned.
3. `(replanning OR "self-correction" OR "human handoff") AND (robot OR robotic) AND ("language model" OR "vision-language")` — 65 hits returned.
4. `("execution feedback" OR "environment feedback" OR "verifiable reward") AND ("tool calling" OR "tool use" OR "program generation" OR robot)` — 117 matches; the API cap returned the newest 100.

These produced **194 returned records and 188 unique papers** after deduplication. I title/abstract-screened all 188, then did full-text or project-page checks for 23 plausible candidates. Targeted follow-ups covered `robot program search`, `coding agent robot safety`, `pairwise progress critic`, `counterfactual recovery`, `verifiable process reward`, `tool-integrated RL`, and the exact titles of leading candidates. I queried Semantic Scholar for venue and citation metadata, checked HF paper pages for upvotes, and checked linked GitHub repositories and project pages for release status and stars.

### Filtering outcome

I kept **8 papers**: seven from the requested 2026 window and one 2025 paper whose **437 citations / 65 influential citations** make it clearly foundational for execution-feedback tool-use RL. Each kept paper has at least two credible signals (for example: an established group, a top/strong venue, released artifacts, real-robot results, large quantitative evaluation, or unusually fast citations). I excluded papers with narrow demonstrations, unquantified correction claims, no meaningful baselines/ablations, or only a promised release plus small experiments.

## 2. Kept papers

### 1. SafeHarness: Coding Agents with Harness for Safe Robot Control

- **Paper:** [arXiv:2609.20822](https://arxiv.org/abs/2609.20822), first submitted **2026-09-17** (v2 renamed the paper on 2026-10-02).
- **Authors/lab:** Bingxin Xu, Yuzhang Shang, Zhen Dong, Emilio Ferrara; **USC, UCF, and UCSB**.
- **Credibility signals:** arXiv preprint; **2 citations / 0 influential** ([Semantic Scholar](https://api.semanticscholar.org/graph/v1/paper/arXiv:2609.20822?fields=title,citationCount,influentialCitationCount,venue,publicationVenue)); established university groups; evaluation on **all 32 SafeLIBERO tasks with 10 seeds each** (320 episodes per method); real-robot evaluation on two tasks with **20 trials each**. Public implementation: **unknown/not linked**; HF upvotes: **unknown**.

**What they did and results.** The paper diagnoses a coding agent that can state a collision constraint yet fails to make it operational. It splits manipulation into free-space routing and contact-rich execution, then adds two harness components: waypoint-route generation with geometric verification/replanning, and obstacle-aware contact-pose selection. With GPT-6-Astra, SafeHarness reaches **81.2% task success** and **91.9% collision avoidance**, gains of 31.2 and 57.5 points over the same agent plus skills. On a PiPER-X arm it reaches **65.0% task success / 82.5% collision avoidance**, versus 27.5% / 45.0% for the same coding-agent backbone ([paper results](https://arxiv.org/html/2609.20822#S4)).

**Borrow for our project.**

- Treat safety and semantic constraints as executable harness checks, not prompt text: compile `must_not_touch`, allowed regions, and gripper/contact constraints into preconditions checked before the runtime calls the motion or grasp planner.
- Give generated programs an explicit phase structure—`route_to_pregrasp`, `contact/grasp`, `route_to_place`, `contact/release`—so recovery can identify and repair the failed phase rather than rewrite the whole task.
- Report **task success, constraint satisfaction, and joint safe success** separately; a task completed through a collision should not count as reliable success.

**Difference from our project:** SafeHarness keeps a frontier coding model in the execution harness and focuses on geometric safety, whereas our target compiles inferred intent into a short program, runs it without a model in the loop, and invokes the model only after a typed runtime failure.

### 2. RoboAuto: Autonomous Robot Program Search through Multi-Level Physical Understanding

- **Paper:** [arXiv:2610.36533](https://arxiv.org/abs/2610.36533), submitted **2026-10-27**.
- **Authors/lab:** Buzhao Liu, Jianing Ye, Xiu Yuan, Yangtong Feng, Chengsong Huang, Tyler Dinh, Yevgeniy Vorobeychik, Ning Zhang, Chongjie Zhang; **Washington University in St. Louis**.
- **Credibility signals:** arXiv preprint; **0 citations / 0 influential** at four days old ([Semantic Scholar](https://api.semanticscholar.org/graph/v1/paper/arXiv:2610.36533?fields=title,citationCount,influentialCitationCount,venue,publicationVenue)); established robotics/RL group; experiments over **five benchmark families** plus **three long-horizon real-robot tasks**, extensive baselines and component/transfer/randomness/token ablations. Code: **unknown/not linked**; HF upvotes: **unknown**.

**What they did and results.** RoboAuto searches for Code-as-Policy programs using (1) a hierarchical store of object affordances, spatial relations, action effects, and verified skills; (2) scene-specific grounding of action-critical quantities; and (3) stage-wise execution with expected-effect checks and self-designed diagnostic experiments. It reaches **89.1% mean success on LIBERO-PRO** versus 71.7% for ASPIRE, **73.4% on LIBERO-PRO Long** versus 30.8%, and **90.6% on Robosuite** versus 81.0%. The base execution-and-rewrite loop scores only 5.2%; adding search/exploration, knowledge, and grounding raises this to 19.2%, 38.4%, and **91.2%**, respectively. It also reports 98.6% mean real-robot stage completion and evaluates final programs on 50 test scenes ([main paper and appendices](https://arxiv.org/html/2610.36533)).

**Borrow for our project.**

- Make every generated program step declare an **expected symbolic state change**, such as `holding(cube_3)` or `inside(cube_3, box)`. The runtime can verify that postcondition and return a compact discrepancy to the recovery model.
- Persist two different memories: general physical facts (affordance, precondition, likely effect) and tested code snippets. Retrieve both when generating or repairing a program, but re-ground all numeric quantities from the current Isaac Sim scene.
- Let recovery run bounded, safe diagnostic tool calls when evidence is ambiguous—for example, move the camera, re-segment the target, or test gripper closure—before choosing a patch. Track search rollouts and token cost as reliability costs.

**Difference from our project:** RoboAuto performs expensive multi-attempt program search with an LLM throughout task solving; our intended system generates once, executes deterministically, and pays model cost only on failure.

### 3. Robot Critics that Sweat the Small Stuff

- **Paper:** [arXiv:2606.21572](https://arxiv.org/abs/2606.21572), submitted **2026-06-19**.
- **Authors/lab:** Sruthi Sudhakar, Junbang Liang, Sreehari Rammohan, Pavel Tokmakov, Richard Zemel, Carl Vondrick; **Columbia University and Toyota Research Institute**.
- **Credibility signals:** arXiv preprint; **5 citations / 1 influential** in about four months ([Semantic Scholar](https://api.semanticscholar.org/graph/v1/paper/arXiv:2606.21572?fields=title,citationCount,influentialCitationCount,venue,publicationVenue)); established labs; [code released](https://github.com/SruthiSudhakar/robocritic) (**1 star**); four real-robot tasks and 18 RoboCasa365 tasks, with matched baselines and failure-data ablations. HF upvotes: **unknown**.

**What they did and results.** The authors collect successful and failed rollouts from matched initial conditions and fine-tune a Qwen2.5-VL critic on pairwise “which state made more progress?” judgments. Failure pairs are policy-specific; approximately **21,000 training pairs per RoboCasa task** are created, with about 5,000 held-out pairs. The full critic reaches **93.6% average fine-grained detection accuracy** versus 88.6% for success-only training and 72.1% for ROVER. Used with action-conditioned video predictions to rank candidate action chunks, it improves real-robot success by an average **11 points** and RoboCasa365 success from **43.6% to 49.5%** across 18 tasks ([project results](https://robocritic.cs.columbia.edu/)).

**Borrow for our project.**

- Build the semantic decision head from **matched success/failure states generated by the same robot stack and initial scene**, rather than generic success-only demonstrations. Include small pose/contact errors because those are the deployment boundary cases.
- Train a pairwise progress/failure objective in addition to absolute labels. Pairwise same-scene comparisons remove much irrelevant visual variation and could feed the typed runtime verdict `{satisfied, violated, uncertain}`.
- Evaluate the head by temporal gap and evidence scale (coarse object motion versus millimetre/contact differences), plus OOD objects and layouts—not only aggregate accuracy.

**Difference from our project:** their critic stays in the low-level action-selection loop and relies on generated future video; ours needs a lightweight, calibrated semantic checker and line-level recovery for an interpretable robot program.

### 4. MobiAgent: Dual-Loop Recursive Policy Self-Improvement for Long-Horizon Mobile Manipulation

- **Paper:** [arXiv:2610.03476](https://arxiv.org/abs/2610.03476), submitted **2026-10-02**.
- **Authors/lab:** Chenzhi Liu, Yue Zhang, Jiehong Lin, Jianan Wang, Bo Wang, Zhongrui Wang, Xiaojuan Qi; **University of Hong Kong, Astribot, and SUSTech**.
- **Credibility signals:** reported by the authors as **CoRL 2026** and present in a [CoRL 2026 accepted-paper index](https://github.com/Studeas/corl-2026-paper-list); **0 citations / 0 influential** ([Semantic Scholar](https://api.semanticscholar.org/graph/v1/paper/arXiv:2610.03476?fields=title,citationCount,influentialCitationCount,venue,publicationVenue)); real-robot Astribot S1 experiments; [code and training recipes](https://github.com/kaiknower/MobiAgent) released (**16 stars**) with model-weight access; **1 HF upvote** ([HF page](https://huggingface.co/papers/2610.03476)).

**What they did and results.** MobiAgent separates an online inner loop (VLM planner, reusable skill policies, and visual reflection critic) from an offline outer loop that segments deployment rollouts, verifies them, clusters atomic skills, and fine-tunes skill policies without manual labels. On four BEHAVIOR-1K tasks the full system averages **65.0% success**, versus 40.0% without global replanning and 10.0% without the reflection critic; **39% of successful episodes require at least one replan**. Recycling roughly 20 automatically curated trajectories per task per round improves four Astribot tasks from **32.5% to 57.5%**, while the monolithic π0.5 baseline scores 10.0% ([quantitative results](https://arxiv.org/html/2610.03476#S4)).

**Borrow for our project.**

- Copy the **two-loop data flywheel**, but apply it to program generation: online, log the program, failing line, typed error, before/after state, attempted patches, and eventual outcome; offline, verify and deduplicate successful recoveries into SFT/LoRA examples.
- Separate local retry from global replan in the runtime contract. Measure success with only retries, with full replanning, fraction of successes requiring recovery, and replans per successful episode.
- Adopt its failure buckets—critic false positives, coordination failures, inter-skill transition/pose mismatch, and low-level perception limits—and add program-specific buckets such as invalid API call, stale object handle, violated precondition, and semantic uncertainty.

**Difference from our project:** MobiAgent fine-tunes VLA skill policies and keeps VLM planning/critique online; our learning target is an open program-writing/recovery model, while execution remains in a conventional planner/grasp stack.

### 5. FailBench: How Reliable are VLMs at Judging Robot Task Success?

- **Paper:** [arXiv:2609.03611](https://arxiv.org/abs/2609.03611), submitted **2026-09-03**.
- **Authors/lab:** Zaruhi Navasardyan, Tatul Danielyan, Hrant Davtyan; **Metric AI Lab**.
- **Credibility signals:** arXiv preprint; **3 citations / 0 influential** in under two months ([Semantic Scholar](https://api.semanticscholar.org/graph/v1/paper/arXiv:2609.03611?fields=title,citationCount,influentialCitationCount,venue,publicationVenue)); unusually broad evaluation of **2,197 attempts from 14 sources** (12 real, 2 simulated) and **13 detectors**; **4 HF upvotes**. The project page labels [code and dataset as “soon”](https://metric-ai-lab.github.io/failbench/), so they are **not counted as released**.

**What they did and results.** FailBench puts heterogeneous success detectors under one protocol using 1,176 failures and 1,021 successes; 75% of failures occurred organically. The best detector reaches only **0.77 macro balanced accuracy**, and specialist failure detectors underperform their own base VLMs by up to 8.8 points. No detector exceeds **0.60 balanced accuracy on contact-rich assembly**, and ambiguous evidence produces a systematic success bias. Cropping to the outcome-relevant region improves the best detector from **0.773 to 0.797** (p=0.0015) without retraining ([leaderboard and ablations](https://metric-ai-lab.github.io/failbench/)).

**Borrow for our project.**

- Validate the semantic runtime head across independently collected sources and report **balanced accuracy, per-source worst case, false-success rate, calibration error, and abstention coverage**. Do not treat in-stack accuracy as proof of transfer.
- Add an evidence-localization stage: use tracked masks and the program’s current predicate to crop or pool only the relevant object/receptacle/contact region before a typed semantic decision.
- Under ambiguity, default to `uncertain → reobserve or ask user`, not success. Audit separately by evidence type: visible displacement, containment, grasp/contact, insertion, and occluded state.

**Difference from our project:** FailBench evaluates post-hoc binary outcome judgments and offers no recovery or program synthesis; it is most useful as a warning and evaluation recipe for our semantic checker and data curator.

### 6. ForkCast: Predictive Recovery for Vision-Language-Action Policies via Counterfactual Futures

- **Paper:** [arXiv:2610.18650](https://arxiv.org/abs/2610.18650), submitted **2026-10-13**.
- **Authors/lab:** Chenru Wen, Huajian Zeng, Haokai Ding, Jian Kang, Anqing Duan, Dawei Zhou, Dezhen Song, Xingxing Zuo; **MBZUAI and Virginia Tech**.
- **Credibility signals:** arXiv preprint; **0 citations / 0 influential** at 18 days old ([Semantic Scholar](https://api.semanticscholar.org/graph/v1/paper/arXiv:2610.18650?fields=title,citationCount,influentialCitationCount,venue,publicationVenue)); established robotics group; simulation across five tasks, three VLA backbones, controlled disturbances and ablations; real-robot evaluation on two tasks × two visibility settings × **30 trials per method/setting**. Code is only promised, not released; HF upvotes: **unknown**.

**What they did and results.** ForkCast replaces a binary failure threshold with a same-state comparison among three futures: **Commit**, **Reobserve**, and **Replan**. A candidate-conditioned latent world model and progress head rank these continuations at a common horizon. On five RoboTwin 2.0 tasks, pooled slip/occlusion success rises from **13.3% to 30.0%** for frozen π0.5; the same recovery model raises XR-0 from 0.0% to 21.7% and RDT from 5.0% to 23.3% without retraining. Candidate-selection hit rate is **75%**, versus 37.5% random and 25% for a direct regressor; real-robot success improves in all four tested settings ([paper results](https://arxiv.org/html/2610.18650#S4)).

**Borrow for our project.**

- Give recovery a small typed action space before arbitrary rewriting: `CONTINUE`, `REOBSERVE`, `RETRY_STEP`, `PATCH_PROGRAM`, `ASK_USER`. Compare candidates from the **same failure state**, which reduces dependence on globally calibrated progress scores.
- Make active perception a first-class recovery: reframe the camera or request another view before patching code when the error is “insufficient evidence.”
- Add candidate-quality metrics beyond final success: best-recovery hit rate, lost progress/regret, unnecessary intervention rate, and recovery latency.

**Difference from our project:** ForkCast predicts latent futures for fixed VLA policies and predefined motor repairs; our system can inspect explicit program state, call symbolic/geometric tools, and modify the program at the failing line.

### 7. Verifiable Process Rewards for Agentic Reasoning

- **Paper:** [arXiv:2605.10325](https://arxiv.org/abs/2605.10325), first submitted **2026-05-11**; **AAMAS 2026**.
- **Authors/lab:** Huining Yuan, Zelai Xu, Huaijie Wang, Xiangmin Yi, Jiaxuan Gao, Xiao-Ping Zhang, Yu Wang, Chao Yu, Yi Wu; **Tsinghua University**.
- **Credibility signals:** strong multi-agent venue (AAMAS 2026); **13 citations / 0 influential** in about five months ([Semantic Scholar](https://api.semanticscholar.org/graph/v1/paper/arXiv:2605.10325?fields=title,citationCount,influentialCitationCount,venue,publicationVenue)); established lab; [code](https://github.com/thu-nics/VPR) (**15 stars**) and [models](https://huggingface.co/collections/nics-efc/vpr) released; extensive five-run evaluation, OOD benchmarks, ablations, and general tool-use tests. HF upvotes: **0**.

**What they did and results.** VPR turns a task-solving oracle into dense action-level rewards, samples groups of candidate actions from the same state, and locally normalizes their advantages. On Qwen3-4B, VPR beats outcome-reward GRPO on Sokoban (**28.4% vs 12.2% success**), Sudoku (**80.6% vs 29.0%**), and Minesweeper (**32.6% vs 4.2%**), over five 100-episode evaluations. With an expert acting as an approximate oracle on τ²-bench, held-out Airline pass¹/pass³ improves from 34.8/17.4 for GRPO to **46.4/30.4**, and held-out Retail from 43.4/28.3 to **55.3/32.1** ([tables](https://arxiv.org/html/2605.10325#S3)).

**Borrow for our project.**

- Convert the simulator and runtime into process verifiers: reward parse/type validity, legal API arguments, satisfied preconditions, collision-free plan existence, successful grasp, and verified postconditions at the line where each becomes known.
- At a logged failure state, sample several candidate next calls or patches, execute them in cloned simulator states, and train with **same-state locally normalized rewards**. This gives cleaner credit assignment than one terminal task-success bit.
- Report both pass@1 and **repeatability** (all of three runs succeed, analogous to pass³). Corrupt verifier labels in an ablation to quantify how calibration errors poison RL.

**Difference from our project:** VPR studies structured games and generic service tools, not visual robotics; our contribution would be the mapping from robot-stack feedback and semantic predicates to trustworthy process rewards.

### 8. ReTool: Reinforcement Learning for Strategic Tool Use in LLMs (foundational 2025 inclusion)

- **Paper:** [arXiv:2504.11536](https://arxiv.org/abs/2504.11536), submitted **2025-04-15**.
- **Authors/lab:** Jiazhan Feng, Shijue Huang, Xingwei Qu, Ge Zhang, Yujia Qin, Baoquan Zhong, Chengquan Jiang, Jinxin Chi, Wanjun Zhong; **ByteDance Seed**.
- **Credibility signals:** arXiv preprint but highly influential: **437 citations / 65 influential** ([Semantic Scholar](https://api.semanticscholar.org/graph/v1/paper/arXiv:2504.11536?fields=title,citationCount,influentialCitationCount,venue,publicationVenue)); established industry lab; [code released](https://github.com/ReTool-RL/ReTool) (**398 stars**); **63 HF upvotes** ([HF page](https://huggingface.co/papers/2504.11536)); repeated AIME evaluation with strong baselines and behavioral analysis.

**What they did and results.** ReTool first SFTs an open Qwen model on automatically transformed, format-checked, answer-verified traces containing interleaved code and interpreter output. It then runs PPO while executing code inside a sandbox and feeding results or error messages back into the rollout, using only a rule-based final-answer reward. Qwen2.5-32B rises from **26.7% base / 40.9% SFT-only to 67.0%** on AIME 2024; DeepSeek-R1-Distill-Qwen-32B reaches **72.5%**. Training also produces executable-code self-correction, while responses become about 40% shorter ([paper](https://arxiv.org/html/2504.11536)).

**Borrow for our project.**

- Use the same two-stage recipe: SFT on verified robot-program and recovery traces for format/API competence, followed by RL on sandboxed Isaac Sim rollouts for actual task success and recovery.
- Interleave runtime error observations with generated recovery actions, but **mask environment/tool-output tokens from the language-model loss**; train only on model-authored tokens.
- Include both successful returns and structured exceptions in SFT so the model learns to repair undefined names, bad arguments, unreachable poses, failed grasps, and stale scene assumptions. Track tool-call format rate, execution rate, correction rate, calls to success, and token/latency cost.

**Difference from our project:** ReTool uses a single Python interpreter for mathematical answers and a 32B model; it neither grounds calls in perception nor handles physical state, motion planning, semantic uncertainty, or human handoff.

## 3. Rejected examples

- **Teaching a Tree to Recover: Online Distillation of an LLM Supervisor with Counterfactual Replay** ([arXiv:2610.39775](https://arxiv.org/abs/2610.39775)) — promising “LLM only on novel anomalies” idea, but the evidence is only 20 unseen navigation layouts in simulation; no top venue, real robot, public artifact, or citation signal was available.
- **ProAct-VLM: Pre-Failure Vision-Language Task Replanning with Continuous Perception Feedback** ([arXiv:2609.37681](https://arxiv.org/abs/2609.37681)) — real Franka experiments and [code](https://github.com/moured/ProAct-VLM) exist, but evaluation is one sorting setup with seven scripted disturbance scenarios (10 trials each), the repo had only **1 star**, and venue/citations were unknown; too narrow relative to the kept systems.
- **Retrieval-grounded robot program generation and simulation-based correction via Model Context Protocol** ([arXiv:2608.21417](https://arxiv.org/abs/2608.21417)) — useful ABB/RobotStudio engineering case study, but the paper reports a 30-query retrieval test and scoped case studies rather than a large end-to-end program-success/recovery comparison with strong baselines and ablations.
- **Beyond Next-Token Prediction: An RLVR Proof of Concept for Tool-Use Agents on Atlassian Workflows** ([arXiv:2607.01465](https://arxiv.org/abs/2607.01465)) — explicitly a proof of concept on only five synthetic API scenarios; the authors note reward construction does not scale and one scenario already saturates, so it does not meet the requested robustness bar.
- **Learning to Use Tools: Reinforcement Learning for Tool-Integrated Mathematical Reasoning** ([arXiv:2608.28447](https://arxiv.org/abs/2608.28447)) — the 1,024-problem held-out evaluation is clean, but it covers one calculator tool on one synthetic Countdown task, with no robot/API diversity, public artifact, venue, or community signal found; ReTool and VPR provide broader, more influential recipes.

## 4. Takeaways

- **Put reliability in the executable interface.** SafeHarness and RoboAuto show that constraints, preconditions, expected effects, and verification stages must be represented as code/harness operations. A prompt saying “be safe” or an opaque exception string is not enough. The project should expose typed planner/grasp/perception errors and require postconditions after every state-changing API call.
- **Train on the stack’s own hard negatives, then use verifiable feedback.** Robot Critics supports matched same-scene success/failure pairs; ReTool supports verified SFT followed by sandbox RL; VPR supports dense same-state process rewards. Together they suggest: bootstrap with format-checked program/recovery traces, add policy-specific failures from Isaac Sim, then optimize parse validity, legal calls, step postconditions, recovery, task success, and safety—while keeping a held-out-layout test set.
- **The verifier is a first-class model with its own safety case.** FailBench shows that a detector with 0.77 average balanced accuracy can be near chance on contact tasks and biased toward declaring success. Calibrate the typed head, localize evidence with tracked masks, measure false-success rate and abstention coverage per predicate type, and route low-confidence/contact-ambiguous cases to reobservation or a human rather than silently labeling them.
- **Recovery should be tiered and measurable.** Use `CONTINUE → REOBSERVE → RETRY_STEP → PATCH/REPLAN → ASK_USER`, with a small budget and an auditable reason for escalation. Report clean task success, success after injected failures, recovery success conditional on detection, unnecessary intervention rate, time/calls to recovery, replans per success, repeatability across seeds, and joint task-plus-safety success.
- **Novelty must be stated narrowly.** RoboAuto already performs execution-grounded robot-program search; SafeHarness already wraps coding agents with verified routing/replanning; MobiAgent already couples runtime replanning to offline rollout curation and fine-tuning; Robot Critics already trains on policy-specific failures. The defensible novelty is their **combination in a small open system**: infer a symbolic task rule from one phone demonstration; compile it into a short readable sandboxed program over a production robot API; execute without a foundation model in the nominal loop; recover at the exact failing line from typed stack feedback; use calibrated semantic decisions with human handoff; and fine-tune the open program/recovery model on this stack’s verified traces. No kept paper demonstrates that complete path end to end.
