# Search log

Search run on 2026-10-31, restricted to arXiv submissions from 2026-04-01 through 2026-10-31. Citation counts are Semantic Scholar counts retrieved on that date; GitHub stars and Hugging Face (HF) upvotes are likewise snapshots, not lifetime-stable numbers. “Venue unknown” means neither arXiv nor Semantic Scholar listed an accepted venue. No 2025 paper was included: the 2026 set already supplied ten papers that passed the two-signal credibility filter.

ArXiv API queries (all sorted by newest first, `max_results=100`):

1. `all:"robot program"` — 19 results.
2. `("code as policies" OR "code policy" OR "program synthesis") AND (cat:cs.RO OR cat:cs.AI)` — 84 results.
3. `robot AND (code OR program) AND (LLM OR VLM)` — 210 results.
4. `manipulation AND ("large language model" OR "vision-language model") AND (program OR code OR planning)` — 257 results.
5. `robot AND ("program repair" OR verification OR debugging OR sandbox)` — query ran, but arXiv returned HTTP 429 before a total count; relevant hits were recovered from the other result sets and exact-paper pages.

The four counted queries returned 570 raw, heavily overlapping rows. I title/abstract-screened them, deeply checked 19 unique candidates, and kept 10. Verification used arXiv HTML/papers, Semantic Scholar, HF paper pages, project pages, and GitHub APIs. The hard filter was applied literally: every kept paper below has at least two of established group, released artifacts, real-robot work, large quantitative evaluation with baselines/ablations, unusually early citations, or notable attention. None had a confirmed top-venue acceptance yet.

# Kept papers

## 1. RAPID: Robot Agentic Programming from Demonstrations

**Paper:** [arXiv:2609.30249](https://arxiv.org/abs/2609.30249)  
**Submitted:** 2026-09-24  
**Authors/labs:** Yuyao Liu, Jiayuan Mao, David Hsu, Leslie Kaelbling, Tomás Lozano-Pérez; MIT, National University of Singapore, University of Pennsylvania, and NVIDIA.

**Credibility signals.** Venue unknown (arXiv preprint). [5 citations](https://www.semanticscholar.org/paper/8be895e09d09c8ca55f5ae5764c41c7add015b3f); established MIT/NUS/UPenn/NVIDIA group; [code](https://github.com/nature21/RAPID) released (3 stars); 8 real-robot tasks with 10 scenes each; large simulation study with 50 novel scenes per task, three trials, six baselines/ablations, plus LIBERO-Pro. HF upvotes: 0.

**What they did.** RAPID turns one visual human demonstration plus a language description into a task specification, success predicates, manipulation primitives, and a reusable strategy program. It reconstructs a MuJoCo environment from RGB-D using VLM labeling, SAM 3/SAM 3D, and FoundationPose, perturbs that scene into a verification set, then lets a coding agent execute, test, and revise programs. Its key representation is an object-centric relational program: primitives are local trajectory-optimization programs with semantic roles, effect parameters, geometry resolvers, costs, and executable success predicates; task phases are joined by relational constraints rather than fixed poses.

**Key quantitative results.** Across eight contact-rich tasks, RAPID averaged **75.9% ± 2.4%** simulation success versus **14.6% ± 2.5%** for CaP-Agent0; removing synthesized scene variants reduced it to **53.2% ± 7.1%**. It led five of six LIBERO-Pro settings (for example, 1.00/0.95 on LIBERO-Object position/instruction perturbations) and was evaluated on a real Franka across all eight tasks.

**Borrow for our project.**

- Make the program schema explicitly relational: `Role(target, container)`, `Predicate(in(target, container))`, and geometry resolved at runtime. This directly supports intent-as-rule rather than replaying demonstrated motion.
- Generate a compact verification suite by perturbing object pose, geometry, material, and distractors in Isaac Sim; discard variants that violate task feasibility before using them for repair or SFT data.
- Require every generated primitive to carry an executable postcondition. Store the failed predicate, trace, and scene as a repair example for LoRA/QLoRA training.

**Difference from our project.** RAPID generates task-specific trajectory-optimization primitives and uses an agent during offline construction; our design writes short programs over a fixed production grasp/motion API and emphasizes runtime line-level recovery and calibrated semantic queries.

## 2. HexaShell: A Thin Harness for Exploring the Limits of VLM Agents as Robot Coding Agents

**Paper:** [arXiv:2610.23479](https://arxiv.org/abs/2610.23479)  
**Submitted:** 2026-10-16  
**Authors/lab:** Hongcheng Gao et al.; HexaFuture (the paper links the company site).

**Credibility signals.** Venue unknown. [2 citations, 1 influential](https://www.semanticscholar.org/paper/121908ad342cf02a09b08f7cef111c075d6e9c8f); [open-source tools](https://github.com/HexaFuture/HexaShell) with **238 stars**; **124 HF upvotes**; official hidden-seed RoboDojo evaluation on 42 tasks × 50 episodes; transfer demonstrations on a different real dual-arm robot.

**What they did.** HexaShell gives a coding agent a deliberately narrow command-line robot interface. The acting agent sees RGB images, calibration, arm state, bounded motion commands, and JSON execution results; it cannot access simulator state or scoring code and runs in a container with no direct network or host-directory access. After failure, an optimizer reads the commands, images, and messages and writes or repairs small Python tools; delivery checks require tools to load and prohibit task/object names in their public interfaces. Object positions are measured through calibrated multi-view geometry rather than privileged poses.

**Key quantitative results.** In RoboDojo’s official hidden-seed evaluation, the frozen system achieved **46.9% success / 53.2 progress**, ahead of the strongest prior leaderboard entry at **39.4% / 46.6**. It reached **81.0%** on memory tasks but only **32.0%** on precision; 42 tasks produced 769 optimizer rounds and about 78,000 lines of evolved tool code. Multi-view geometry beat a monocular depth network by **8.7 points**; the paper also reports two real-robot transfer episodes after changing the arm model.

**Borrow for our project.**

- Copy the isolation boundary: generated code gets only a small allow-listed API, refreshed observations, no simulator handle, no filesystem outside a task directory, and structured JSON results with remaining budget.
- Add a “delivery check” before accepting a repair: import/type check, API allow-list scan, interface/schema validation, then replay on the failed layout and held-out variants.
- Use active second views when metric uncertainty is high, and charge camera motion to the task budget; log perception-induced versus contact-induced failures separately.

**Difference from our project.** HexaShell evolves reusable tools across simulator attempts and does not infer intent from a phone demonstration; our runtime executes a compact task program on an existing motion/grasp stack and asks the model back only on failure.

## 3. A Few Words Go a Long Way: Language Guided Robot Policy Synthesis (ARCHITECT)

**Paper:** [arXiv:2607.23784](https://arxiv.org/abs/2607.23784)  
**Submitted:** 2026-07-26  
**Authors/labs:** Daphne Chen, Archit Ritesh Jain, Eric Goossen, Emma Romig, Michael Murray, Nick Walker, Maya Cakmak; University of Washington, Microsoft Research, and MIT.

**Credibility signals.** Venue unknown (Semantic Scholar lists arXiv only). [4 citations, 1 influential](https://www.semanticscholar.org/paper/2dbb834a49ea97a2626cab9743ad77e2f1f1a165); established UW/MSR/MIT group; [Franka code](https://github.com/robo-architect/architect-franka) released (9 stars); **400 real-robot benchmark rollouts** over eight tasks and five methods, with baselines and an ablation; a six-person transfer study. HF upvotes: 0.

**What they did.** ARCHITECT has a coding agent synthesize modular robot programs over perception, VQA, grasp, proprioception, motion-planning, and CLI tools. When execution is wrong, the user supplies a short language correction; the agent combines that correction with the execution trace to localize the responsible tool call or code region, regenerate the policy, and distill the correction into a persistent skill library. The library is subsequently loaded as long-term in-context memory.

**Key quantitative results.** The table’s mean success is **81.25%** for ARCHITECT-HiTL versus **17.5%** CaP, **27.5%** π0, **36.25%** π0.5, and **37.5%** automatic VLM correction. It scored 80–90% on articulated, deformable, and long-horizon tasks where most baselines were 0–10%. Reusing the skill library reduced mean human queries from **4.67 to 0.83** (`p=0.036`), with 3/6 users needing no correction on the transfer task.

**Borrow for our project.**

- Present recovery context as `{program, failing_line, structured_trace, pre/post world state, user_rule}` and require the model to identify the faulty assumption before patching.
- Distill successful repairs into short, human-readable rules plus code templates, indexed by failure signature (for example, “placement target occupied” or “grasp too high”).
- Measure correction burden—model repair calls and user questions per successful task—not just final success.

**Difference from our project.** ARCHITECT relies on a human supervisor for correction and keeps the model in the iterative teaching loop; our default execution is autonomous and deterministic, with user questions reserved for calibrated semantic uncertainty.

## 4. CodeActionBench: Evaluating Agentic Code-as-Policy for Embodied Manipulation

**Paper:** [arXiv:2609.33807](https://arxiv.org/abs/2609.33807)  
**Submitted:** 2026-09-27  
**Authors/lab:** Yiheng Lyu, Xueying Jiang, Wenhao Li, Shijian Lu, Gongjie Zhang; Nanyang Technological University plus an independent researcher.

**Credibility signals.** Venue unknown. [3 citations](https://www.semanticscholar.org/paper/e45cc559a26a8e18afb2204e4d8250ab4c72c645); established NTU group; [benchmark repository](https://github.com/lyhkk/CodeActionBench) released (3 stars); **675 attempts** across 25 tasks and nine configurations, with fixed budgets, hidden verifiers, baselines, and trajectory-level analysis. HF upvotes: 0.

**What they did.** This benchmark fixes the robot API, tasks, budgets, and hidden physical-outcome verifier while varying model/harness combinations. Agents receive multi-view RGB, calibrated geometry operations, bounded motion, robot/contact feedback, and `run_code`, but no object poses, task targets, specialist grasp outputs, or verifier result. It separately records the final physical outcome and the agent’s own completion claim, plus subgoal and spatial checkpoints.

**Key quantitative results.** Success ranged from **2.7% to 73.3%**; the best configuration solved 22/25 tasks and supplied a correct completion claim on **68/75 (90.7%)** attempts. Of 491 failures, **280** never confirmed arrival at the operation region, **77** lacked object control, **122** reached the region but failed/lost the subgoal, **6** failed only the final gripper condition, and **6** were unresolved.

**Borrow for our project.**

- Adopt its information boundary and hidden verifier so semantic success checks cannot leak ground truth to generated code.
- Report both task success and self-assessment calibration: overclaim, underclaim, abstain/ask-user, and Brier/ECE for typed semantic decisions.
- Reuse the staged failure taxonomy—localization/arrival, object control, subgoal achievement, subgoal retention, final gripper, unresolved—and add syntax/API/motion-planner categories specific to program execution.

**Difference from our project.** It is a simulator benchmark for general-purpose interactive coding agents, not a demonstration-to-intent system or a trainable recovery pipeline.

## 5. Make Code as Policy Great Again: Frontier Agents Write, Call, and Evolve Robot Tools (URAI)

**Paper:** [arXiv:2609.39018](https://arxiv.org/abs/2609.39018)  
**Submitted:** 2026-09-30  
**Authors/labs:** Shijia Ge et al.; HexaFuture, Peking University, Beijing Institute of Technology, and Tsinghua University.

**Credibility signals.** Venue unknown. [6 citations](https://www.semanticscholar.org/paper/eef53805c65a5ac9b997f65aa9e724e3ce84a620), unusually early for a one-month-old preprint; established university/company group; quantitative evaluation over five RoboDojo tasks and four frozen agents; **seven real-world AgileX dual-arm tasks**. HF upvotes: 2. Code/repository: unknown.

**What they did.** URAI separates a programming agent, which writes and revises reusable robot tools from intent and feedback, from an execution agent, which chooses and parameterizes those tools between observations. Each tool runs a complete local multi-phase motion before control returns to the model. Validated revisions persist without updating model weights, and humans and agents share the same GUI/API.

**Key quantitative results.** Across five RoboDojo tasks and four agents, tools raised aggregate success from **18.0% to 53.0%** versus direct fingertip control. With identical tools, an ahead-of-time program obtained **24%**, while two agents making decisions between tool calls obtained **56%**. Three agents were **1.3–1.5× faster** and used **1.5–1.7× fewer output tokens**; the system was also tested on seven real-robot tasks.

**Borrow for our project.**

- Keep two artifacts separate: a short top-level task program and a vetted tool library. Allow repairs to promote recurring sequences into tools only after replay validation.
- Compare “entire program runs model-free” with a bounded model-between-skills ablation; this will quantify the reliability/latency cost of our chosen no-model-in-loop design.
- Track execution time and model tokens/cost alongside success and recovery.

**Difference from our project.** URAI deliberately retains model decisions between tool calls; our runtime runs the generated program without a model until an explicit failure or uncertain semantic predicate.

## 6. VLCP: Vision Language Control Policy Closed-Loop Code Replanning for Robot Manipulation

**Paper:** [arXiv:2608.16978](https://arxiv.org/abs/2608.16978)  
**Submitted:** 2026-08-17  
**Authors/labs:** Dhia Naouali, Minghan Wu, Claudia Wong, Abhinav Puthran, Omar G. Younis; cross-institution group spanning University of Monastir, Cornell, Carnegie Mellon, Mila, Silverstream AI, and St. Mildred’s-Lightbourn School.

**Credibility signals.** Venue unknown. [5 citations](https://www.semanticscholar.org/paper/75393e84a797d99e73604225c2a20d60c6520c39), high for its age; large **57-task** MuJoCo/RoboVerse evaluation with the identical open-loop system as control and confidence-interval analysis; established institutions. Code: unknown; HF upvotes: 0; real robot: no.

**What they did.** A frozen VLM writes a short Python control function without demonstrations or fine-tuning. Every `K` steps, it re-observes multi-view RGB, proprioception, and state delta, then rewrites the function inside the episode. Recovered control blocks persist in a cross-episode skill library. This tests repair at the code level rather than merely retrying a fixed policy or selecting a new subtask.

**Key quantitative results.** Closed-loop rewriting achieved **35.1% pooled success** versus **3.5%** for one program query per episode. Failed grasps had **27.3% within-episode recovery**. Episodes used about 10 compact queries, with a median **84% input-token cache hit rate**.

**Borrow for our project.**

- Use state deltas, not only the latest frame, in the repair payload so the model can distinguish “never grasped” from “grasped then dropped.”
- Make within-episode recovery rate a primary metric and report recovery by failure class and number of repair calls.
- Cache static API documentation and program context, sending only the failing slice and state delta on repair.

**Difference from our project.** VLCP rewrites periodically even without an explicit runtime exception and uses privileged simulator object poses according to CodeActionBench’s comparison; our model is re-entered only on failure and must ground through perception APIs.

## 7. World Action Agent: Harnessing VLMs for Robot Manipulation via World Action Rehearsal

**Paper:** [arXiv:2609.29964](https://arxiv.org/abs/2609.29964)  
**Submitted:** 2026-09-24  
**Authors/labs:** Yehang Zhang et al.; HKUST(GZ), CUHK, and Knowin AI.

**Credibility signals.** Venue unknown. [4 citations](https://www.semanticscholar.org/paper/bc3804ef7428e52c45e247189370b475f765b7d3); **15 HF upvotes**; established labs; large LIBERO-Pro/robosuite evaluation with baselines, transfer, and small-model trace fine-tuning. Code/repository: unknown; real robot: not reported.

**What they did.** WAA lets a VLM propose actions in a visual workspace, automatically chooses contact-centric views, rehearses each action before execution, and revises it using planning feedback. It also learns multimodal skills from expert videos/human teaching under evidence-based review and trains a smaller VLM on collected interaction traces. Although it is not Python program synthesis, its proposal–simulate–revise boundary is directly relevant to verifying generated robot actions before committing them.

**Key quantitative results.** Skills evolved only from LIBERO-90 reached **75.6% average success on LIBERO-Pro** and transferred to robosuite without further learning. Fine-tuning Qwen3.5-9B on harness traces raised out-of-domain success from **1.7% to 43.3%**.

**Borrow for our project.**

- Before executing each motion-producing API call, rehearse it in a short-horizon clone of the current Isaac Sim state and return collision, IK, reachability, and expected postcondition diagnostics.
- Store the proposal, simulated outcome, revision, and eventual real/sim execution outcome as paired SFT or preference data for the open code model.
- Give the repair model contact-centric crops rather than the entire scene when the failing line is a grasp/place operation.

**Difference from our project.** WAA generates and revises action proposals in a persistent VLM loop rather than emitting a readable, sandboxed task program that runs model-free.

## 8. PhysCaP: Grounding Code-as-Policy Agent with Physics-Informed Exploration

**Paper:** [arXiv:2608.21031](https://arxiv.org/abs/2608.21031)  
**Submitted:** 2026-08-21  
**Authors/labs:** Chen-Yu Lin et al.; National Taiwan University, NVIDIA Research, and National Yang Ming Chiao Tung University.

**Credibility signals.** Venue unknown. [3 citations](https://www.semanticscholar.org/paper/1a47d7909220cbbb5e41e729cab290e6d451235a); **4 HF upvotes**; established NTU/NVIDIA/NYCU group; three real-world tasks with 10 trials each plus LIBERO simulation; progressive baselines and module ablations. Project page exists; code/repository unknown.

**What they did.** PhysCaP extends a code-as-policy agent with active tests for latent physical properties. Training-free `get_mass` and `get_stiffness` modules infer properties from ordinary robot proprioception and gripper behavior. A Planner decides whether evidence is sufficient and when to stop; a separate Prioritizer removes implausible interactions and orders the remaining tests before the code generator acts.

**Key quantitative results.** Compared with exhaustive `CaP+PhysX`, PhysCaP reduced mean interactions from **4.0→2.0** (empty can), **4.0→2.0** (ripe avocado), and **9.17→5.50** (grocery packing). Execution time fell from **515.61±9 s to 300.47±53 s** on avocado and **1019.32±376 s to 572.79±61 s** on packing; it tied for lowest interaction count on the can task.

**Borrow for our project.**

- Treat uncertain semantic predicates as information-gathering plans: before asking the user “is this trash?”, try cheap evidence calls such as inspect label, estimate fullness/mass, or compare against a learned prior.
- Separate “should we gather evidence/stop?” from “which evidence action next?”; calibrate both typed decisions independently.
- Log evidence cost (extra interactions and seconds) and condition calibration metrics on whether the system had direct visual evidence, physical evidence, or only a prior.

**Difference from our project.** PhysCaP focuses on actively measuring hidden physics and keeps an agentic planning loop; our central problem is inferring a demonstrated task rule and repairing a deterministic program.

## 9. MoCaP: Mobile Code-as-Policies for Zero-Shot Mobile Manipulation from Passive Human Videos

**Paper:** [arXiv:2610.20998](https://arxiv.org/abs/2610.20998)  
**Submitted:** 2026-10-14  
**Authors/labs:** Yiyang Cai, Tatsuya Kamijo, Ninghao Lu, Zijin Hu, Sherry Yang; NYU, NYU Shanghai, and Fudan University.

**Credibility signals.** Venue unknown. [1 citation](https://www.semanticscholar.org/paper/81c9cd4f4a03a0e3c6795d755fe065be55ee79fd); established NYU/Fudan group; [code](https://github.com/MoCAP-7/MoCAP) released (7 stars); full evaluation on a real mobile manipulator with matched baselines and ablations. HF upvotes: 0.

**What they did.** MoCaP extracts reusable spatial memory and human approach positions from passive egocentric activity video. A VLM code agent then writes short Python programs over navigation, search, docking, perception, grasp, IK, and cuRobo tools. Failed calls abort the remaining program and return diagnostics; fresh observations and persistent conversation history inform the next generated program.

**Key quantitative results.** On the real robot, MoCaP succeeded on **16/20 box** and **8/10 can** trials (80% each), while CaP-X and a collision-aware CaP-X+ variant achieved zero complete successes. Video spatial memory increased target-navigation success from **70% to 100%**; the human approach prior produced a reachable base pose in **5/5** unfavorable-start trials versus **0/5** without it. The authors explicitly limit claims to one environment, one recording, and three target objects.

**Borrow for our project.**

- Abort a generated block on the first failed robot API call; never execute later lines whose preconditions may now be false.
- Standardize tool errors into stage, reason, measured state, retained alternatives, and suggested next evidence—exactly the payload needed for line-level recovery.
- Distinguish reusable video-derived task/scene memory from live geometry; keep historical and current observations explicitly labeled in prompts.

**Difference from our project.** MoCaP uses passive video as navigation/approach prior rather than inferring a one-shot tabletop intent rule, and it regenerates programs repeatedly instead of resuming one persistent program.

## 10. Generalizing Manipulation Skills with a Local Coding Agent

**Paper:** [arXiv:2609.26499](https://arxiv.org/abs/2609.26499)  
**Submitted:** 2026-09-22  
**Authors/lab:** Raman Talwar, Elias Nijs, Andreas Verleysen, Francis Wyffels; IDLab-AIRO, Ghent University–imec.

**Credibility signals.** Venue unknown. [2 citations](https://www.semanticscholar.org/paper/c708623a75a79d9ce185e3e612614b302342ddee); established Ghent/imec lab; real UR3e evaluation on nine tasks × five trials; linked open [AIRO stack](https://github.com/airo-ugent/airo-mono) has 34 stars. HF upvotes: 0.

**What they did.** A local open-weight Qwen3.8-27B VLM runs as a coding agent on one workstation, writing and debugging code above services for kinematics, safety limits, and classical vision. Nine toy-based tasks probe one-shot transfer across color, size, shape, and task variations. The study is useful because it exposes latency and reliability limits of an entirely local, inspectable stack rather than relying on a frontier hosted model.

**Key quantitative results.** It generalized successfully in **30/45 trials (66.7%)**. Runs took **3.4–67.5 minutes** depending on task complexity. Asking the agent to repeat a task after a prior success reduced duration by **50%**, suggesting reuse/self-improvement, though the evaluation remains small.

**Borrow for our project.**

- Include attribute-factorized splits: unseen colors, shapes, sizes, combinations, and entirely new object instances; report which attribute caused grounding failure.
- Benchmark the fine-tuned open model both fully local and against a frontier-model upper bound, including wall time, GPU memory, repair calls, and program validity.
- Cache successful readable programs and retrieve by task-rule signature before generation; measure whether reuse improves both speed and success.

**Difference from our project.** It starts from a language instruction and lets the coding agent stay active throughout; it neither infers intent from a demonstration nor isolates model-free execution with explicit runtime recovery.

# Rejected examples

- [Retrieval-grounded robot program generation and simulation-based correction via Model Context Protocol](https://arxiv.org/abs/2608.21417) — useful ABB/RobotStudio engineering, but only a 30-query retrieval test plus simulated-cell case studies; zero citations, no real robot, and no verified public code, so it did not reach two strong signals.
- [APPROVE: Visual End-User-in-the-Loop Robot Programming with LLMs](https://arxiv.org/abs/2608.19281) — proposes Blockly visualization, confirmation, and reuse, but the retrieved paper record reports no quantitative robot evaluation; zero citations and no verified artifact signal.
- [ORCESTRA: VLM-driven Visual Robot Programming in Mixed Reality](https://arxiv.org/abs/2608.00775) — demonstrates digital-twin preview across embodiments, but no quantitative baselines/ablations or physical-robot results were retrieved; zero citations.
- [SkillComposer: Learning Reusable Skills for Natural-Language Robot Programming](https://arxiv.org/abs/2608.14944) — relevant generate–test and macro-library idea, but simulation-only evaluation and a 12-person study are too small for this hard filter; no real robot or verified code release.
- [Weights or Skills? A Survey of Robot-Learning Techniques](https://arxiv.org/abs/2608.01851) — a relevant 77-system taxonomy, but it is a survey with no new robot experiments and no confirmed top-venue survey status; excluded by the stated survey rule.

# Takeaways

- **Closest overlap / novelty risk:** RAPID already covers one visual human demonstration → inferred task predicates and primitives → reconstructed simulation → iterative program verification → reusable relational robot program. Our honest novelty must therefore center on *rule intent for simple tabletop tasks*, a *small readable sandboxed API program*, *production motion/grasp services*, *model-free execution*, *line-level runtime resume*, *calibrated typed semantic decisions/user deferral*, and *stack-feedback fine-tuning*—not the broad claim “one video to a verified robot program.”
- **Best representation:** combine RAPID’s semantic roles, relations, effect parameters, and executable postconditions with a deliberately tiny Python DSL/API. Resolve object identities/geometry at runtime; never bake demonstration coordinates into the program.
- **Best safety/reliability architecture:** copy HexaShell’s container and information boundary, MoCaP’s fail-fast block semantics, and CodeActionBench’s hidden verifier. Preflight each API call in Isaac Sim, but keep success scoring hidden from generated code to prevent reward leakage.
- **Best repair/data recipe:** package the failing line, structured error, state delta, contact-centric images, planned versus observed postcondition, and simulator replay. Validate a patch on the failed scene plus perturbed scenes, resume from a checkpoint, and retain successful `(failure, diagnosis, patch, outcome)` tuples for SFT/preference/RL training. ARCHITECT, VLCP, WAA, and HexaShell each support a different part of this recipe.
- **Best evaluation:** report syntax/API validity, first-run success, held-out-layout success, within-episode recovery, number of repair calls, resume correctness, wall time/tokens, and CodeActionBench-style failure stages. For semantic checks report coverage versus risk, abstain/user-question rate, Brier score/ECE, overclaim/underclaim, and evidence cost; stratify grounding by unseen color/shape/size combinations as in the local-agent study.
