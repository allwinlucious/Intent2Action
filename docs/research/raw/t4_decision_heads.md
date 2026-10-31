# Fast calibrated decision making on LLM/VLM representations

## 1. Search log

Search performed 2026-10-31, with the requested primary window of 2026-04-01 through 2026-10-31. Citation and repository counts below are point-in-time counts from that date.

### arXiv API queries

| Query (sorted by submission date, `max_results=100`) | arXiv total | Returned | In-window |
|---|---:|---:|---:|
| `all:"hidden state" AND (all:classification OR all:probe OR all:confidence)` | 621 | 100 | 100 |
| `all:"selective prediction" AND (all:"language model" OR all:"vision language")` | 73 | 73 | 40 |
| `all:calibration AND (all:"vision-language" OR all:multimodal)` | 1,602 | 100 | 100 |
| `all:robot AND (all:JEPA OR all:"joint embedding") AND all:perception` | 11 | 11 | 9 |
| `all:"open-vocabulary" AND all:manipulation AND (all:predicate OR all:affordance)` | 25 | 25 | 5 |
| `all:"concept learning" AND all:robot AND all:demonstration` | 8 | 8 | 1 |
| `all:uncertainty AND all:"vision language model" AND all:robot` | API failed after retries | 0 | 0 |
| `all:"semantic predicate" AND all:robot` | API failed after retries | 0 | 0 |

The six successful broad queries produced **254 unique in-window candidates after deduplication**. I screened titles and abstracts, read the paper sources/tables for the finalists, and ran targeted exact-title searches for project pages and venue records. I also checked the Semantic Scholar Graph API (citations/venue), GitHub API (stars), Hugging Face paper pages (upvotes), official project pages, and official conference pages. All nine retained 2026 papers lacked a Hugging Face paper page, so their HF upvotes are **unknown**. One older 2025 paper was added through a targeted follow-up because it is a top-venue benchmark with 181 citations.

**Kept: 10 papers** (nine from the primary window and one clearly foundational 2025 paper). Ranking combines credibility and usefulness to the proposed robot system, not citation count alone.

## 2. Kept papers

### 1. Recover, Discover, Plan: Learning Skills and Concepts from Robot Failures (ReSYNC)

- **Paper:** [arXiv:2606.18328](https://arxiv.org/abs/2606.18328), submitted **2026-06-16**.
- **Lab:** CMU, Princeton, UCSD, MIT, Centaur AI, and Bosch Center for AI ([project page](https://jaraxxus-me.github.io/ReSYNC/)).
- **Credibility signals:** accepted at **CoRL 2026**; **5 citations** ([Semantic Scholar](https://www.semanticscholar.org/paper/9ed86439dac8a691c35917dc54cc473bd8cd2598)); [code repository](https://github.com/Jaraxxus-Me/ReSYNC), **1 star**; four simulated domains, five seeds and 50 tasks per scenario, plus sim-to-real non-prehensile manipulation. HF upvotes: unknown.

**What they did.** ReSYNC alternates skill learning from failures with synthesis/refinement of relational predicates that explain when the learned recovery works. The new predicates and operators enter an abstract planner, turning a recovery observed locally into something the planner can use before related failures occur. Across four domains it reports **70% average success on compositionally novel tasks**, with **20–70 percentage-point** gains over recovery, predicate-learning, policy, VLM-planning, and HRL baselines; removing concept fine-tuning costs 37 points on held-out tasks.

**Borrow for our project:**

- Mine `(pre-state, failed line, error, recovery, post-state)` runtime traces into candidate relational predicates and preconditions, rather than using stack feedback only for program SFT.
- Maintain a small versioned predicate library whose classifiers and symbolic meanings can be refined after failures; expose it to program generation as typed API calls.
- Add a “compositional failure generalization” split: train on isolated failure modes, test programs that must combine the resulting concepts in unseen layouts.

**Difference from our project:** ReSYNC learns skills and symbolic abstractions for a planner, whereas our system infers a rule from one video and writes an executable Python program over an existing production robot stack.

### 2. Why Generate a Click? Plug-in Decision Heads for Web Agents

- **Paper:** [arXiv:2610.20107](https://arxiv.org/abs/2610.20107), submitted **2026-10-14**.
- **Lab:** independent researcher plus University of Chinese Academy of Sciences, Pusan National University, Monash University, and Shenzhen University of Advanced Technology.
- **Credibility signals:** citations **0** ([Semantic Scholar](https://www.semanticscholar.org/paper/efa1bffce86ab13c6809970ef682466fb9b8b56a)); venue unknown and no code release found, but a large quantitative study: **213k training steps**, seven offline test sets, 119 MiniWoB++ tasks, and 174 live WebArena tasks, with 4B and 8B backbones and per-seed reporting. HF upvotes: unknown.

**What they did.** Small heads read a learned mixture of five hidden layers from a **frozen** Qwen3-VL generator and factor a decision into action type, target box, and text argument. A confidence cascade executes the head when the product of component probabilities exceeds a threshold and otherwise resumes generation from the same KV cache. At a strict threshold, heads decide **12–64%** of steps while remaining from 0.3 points below to 1.4 points above the generator; at a looser threshold WebArena success rises from **12.5 to 13.8%** (4B) and **13.6 to 14.6%** (8B), with fewer than half the generated tokens. Jointly training heads and generator hurt generation by as much as 4.9 points.

**Borrow for our project:**

- Factor each fast semantic decision into typed heads, e.g. predicate value (`trash/not_trash/unknown`), object candidate, and optional destination candidate, instead of generating a free-form answer.
- Train the heads after the VLM is frozen; the paper's joint-training regression is a concrete reason to keep the fast gate separate from program-writing LoRA.
- Use an explicit confidence cascade: execute a typed decision only above a calibrated threshold; otherwise invoke a richer inspection tool or ask the user. Report coverage at fixed risk as well as raw accuracy.

**Difference from our project:** It selects GUI actions and falls back to token generation, not semantic object predicates or robot programs executed without a model in the control loop.

### 3. AbstentionBench: Reasoning LLMs Fail on Unanswerable Questions

- **Paper:** [arXiv:2506.09038](https://arxiv.org/abs/2506.09038), submitted **2025-06-10** (the sole older foundational inclusion).
- **Lab:** FAIR/Meta; authors include Polina Kirichenko, Mark Ibrahim, Kamalika Chaudhuri, and Samuel Bell.
- **Credibility signals:** **NeurIPS 2025 Datasets & Benchmarks Track** ([proceedings](https://proceedings.neurips.cc/paper_files/paper/2025/hash/fb122bfc3f0127a94ded048b5b03496f-Abstract-Datasets_and_Benchmarks_Track.html)); **181 citations, 32 influential** ([Semantic Scholar](https://www.semanticscholar.org/paper/077d0cb33c63a0b40de3d6fb6167673d5f76f180)); [code/data](https://github.com/facebookresearch/AbstentionBench), **101 stars**; **6 HF upvotes** ([HF page](https://huggingface.co/papers/2506.09038)); 20 datasets × 20 frontier models.

**What they did.** AbstentionBench organizes unanswerability into unknown answers, underspecification, false premises, subjectivity, and stale information, and evaluates whether models answer or abstain appropriately. Scaling alone did not solve the problem. Most importantly for fine-tuning plans, reasoning fine-tuning **reduced abstention by 24% on average**, including math/science tasks, while better prompting helped but did not remove the underlying failure.

**Borrow for our project:**

- Turn its taxonomy into robot-specific negative sets: invisible attribute, missing task rule, false object premise, subjective predicate (“trash”), and stale world state.
- Evaluate the semantic head as a three-way selective classifier, separating “negative” from “insufficient evidence”; do not count every refusal as correct.
- Re-run calibration after every program-writing or recovery fine-tune, because capability tuning may silently degrade abstention.

**Difference from our project:** It is a text-only evaluation benchmark and proposes no hidden-state head or embodied intervention policy.

### 4. SIEVES: Selective Prediction Generalizes through Visual Evidence Scoring

- **Paper:** [arXiv:2604.25855](https://arxiv.org/abs/2604.25855), submitted **2026-04-28**.
- **Lab:** TU Darmstadt and hessian.AI's Multimodal Reliable AI group.
- **Credibility signals:** **1 citation** ([Semantic Scholar](https://www.semanticscholar.org/paper/d08ec1ebfa948430242c0defe6fadc6969098968)); [released code/artifacts](https://github.com/hector-gr/SIEVES), **2 stars**; five OOD VQA benchmarks, three reasoners (Pixel-Reasoner, o3, Gemini-3-Pro), and ablations of correctness, localization, coherence, annotation model, and bounding-box source. Venue unknown; HF upvotes unknown.

**What they did.** SIEVES requires the reasoner to return localized visual evidence, then trains a separate selector to score answer correctness, localization quality, and crop–image coherence. It transfers without adaptation from Pixel-Reasoner traces to stronger proprietary reasoners. On o3 it raises coverage at 5% risk on V*Bench from **0.0% for a correctness-only selector to 23.1%**, and on Gemini-3-Pro it raises AdVQA coverage at 10% risk from **32.5% to 33.3%** while improving AURC; across settings the paper reports up to **3×** coverage at fixed risk.

**Borrow for our project:**

- Require every semantic predicate decision to carry a mask/bounding box and a short evidence type (material cue, location cue, state cue); train the gate on correctness **and** evidence quality.
- Use SAM-track consistency and crop–full-frame coherence as features for “is this trash/fragile?” rather than trusting a class probability detached from the pixels.
- Include OOD risk–coverage curves across layouts, cameras, clutter, and object categories; tune on one VLM but test selector transfer after swapping the program-writing model.

**Difference from our project:** SIEVES is a post-hoc selector over VQA outputs and crops, not a single-pass hidden-state head or robot execution system.

### 5. Look Again Before You Abstain: Budgeted Conformal Evidence Acquisition for Reliable Vision-Language Models (BCEA)

- **Paper:** [arXiv:2606.16667](https://arxiv.org/abs/2606.16667), submitted **2026-06-15**.
- **Lab:** RIKEN iTHEMS/AIP, South China University of Technology, and Columbia University.
- **Credibility signals:** **3 citations** ([Semantic Scholar](https://www.semanticscholar.org/paper/5c3b673c678ccbeab3faf9355d283b0c0f2d30f7)); four open VLMs, roughly **11.5k POPE claims**, COCO existence/spatial claims, **300 calibration/test repetitions**, risk guarantees and ablations. Venue and code release unknown; HF upvotes unknown.

**What they did.** BCEA replaces answer/abstain with **answer / acquire evidence / abstain**. It shows that naively adding zoom/crop acquisition after calibration breaks exchangeability and can overshoot the risk target by **up to 17 points**. Folding the acquisition policy into the nonconformity score and recalibrating restores the finite-sample guarantee; on the image-level certified protocol it reports zero violations, while practical POPE coverage is often **2–4×** the guaranteed-abstention baseline.

**Borrow for our project:**

- Use a three-way runtime action: decide now, acquire another crop/view/segmentation, or ask the human. “Ask” should be the final branch, not the only alternative to guessing.
- Calibrate the **whole adaptive policy** on held-out robot episodes. A threshold calibrated before camera motion, re-segmentation, or tool use is not statistically valid afterward.
- Define predicate-specific acquisition tools: inspect the bin and surrounding region for `trash`, rotate/view the object for `fragile`, and query weight/material metadata where available.

**Difference from our project:** BCEA verifies VLM claims with extra visual evidence; it does not learn typed robot predicates or execute manipulation programs.

### 6. From Knowing to Abstaining: Bridging the Representation–Action Gap in Vision–Language Models (Rep2Act)

- **Paper:** [arXiv:2609.32653](https://arxiv.org/abs/2609.32653), submitted **2026-09-26**.
- **Lab:** Hong Kong University of Science and Technology (Guangzhou).
- **Credibility signals:** citations **0** ([Semantic Scholar](https://www.semanticscholar.org/paper/b15e1a7cefca07973b15eeab2759084be99e892c)); a new **1,852-example** shortcut-filtered benchmark, two backbone sizes, multiple open/closed baselines, two OOD benchmarks, general-VQA preservation checks, and loss/layer ablations. It is an ICLR 2027 submission, not an accepted paper; code and HF upvotes unknown.

**What they did.** Probes show answerability is strongly separable in mid/late frozen VLM states even when the model still answers: development macro-F1 peaks around **90.8–91.7%**. Rep2Act pools selected layers into a two-output route head, fuses that prior with answer/abstain vocabulary logits, and trains an explicit route token plus signed-margin and route-contrastive losses. It raises action accuracy from **56.67 to 86.33%** (3B) and **59.33 to 88.67%** (7B); its 3B model reaches **53.3 F1** on OOD TUBench, while general VQA moves by at most 0.64 points.

**Borrow for our project:**

- Probe all layers first, then pool a narrow mid/late band; do not assume the final token or final layer is the best semantic feature.
- Train a frozen-backbone two-class/three-class route head before any LoRA. If LoRA is later used, preserve the original backbone and re-check head calibration.
- Adapt route-contrastive training to typed predicates: the same world state with the route flipped should make the correct downstream verification/program continuation more likely only under the true route.

**Difference from our project:** Rep2Act ultimately conditions VLM text generation and jointly trains LoRA; our desired predicate head should return a calibrated typed value directly to a deterministic runtime.

### 7. OpenDexGrasp: Open-vocabulary Task-Oriented Dexterous Grasping

- **Paper:** [arXiv:2609.18117](https://arxiv.org/abs/2609.18117), submitted **2026-09-16**.
- **Lab:** Peking University CFCS and National Key Laboratory for Multimedia Information Processing.
- **Credibility signals:** **CoRL 2026 Oral** ([official project page](https://opendexgrasp.github.io/)); citations **0** ([Semantic Scholar](https://www.semanticscholar.org/paper/3fac4b0342477d55a93100ac075a439f6f7a5030)); linked [repository](https://github.com/opendexgrasp/OpenDexGrasp), **0 stars**; 1.24M automatically synthesized grasps plus 27.42k aligned grasps, simulation and real-robot evaluation. HF upvotes: unknown.

**What they did.** OpenDexGrasp maps free-form language, multi-view RGB, and object geometry to functional dexterous grasps. Its “Coverage-to-Alignment” recipe combines **1.24M** broad automatic/VLM-annotated grasps with **27.42k** teleoperated or category-transferred high-quality grasps. Affordance-map supervision shapes a shared perception–action latent but is not a brittle test-time cascade. It reports **71.65%** simulation success and **72.0%** real-robot success versus **59.0%** for its adapted DexGraspNet 2.0 baseline.

**Borrow for our project:**

- Copy the data mixture: broad synthetic predicate labels from simulator/VLM rules, then a much smaller human-verified set for alignment and calibration.
- Use dense affordance grounding as an auxiliary target for the frozen representation/head, even if runtime exposes only a typed boolean plus object mask.
- Evaluate functional success on unseen categories (“grasp so the object remains usable”), not merely pick stability.

**Difference from our project:** It directly generates high-DoF task-oriented grasps rather than inferring a demonstrated rule and compiling it into Python over an external grasp planner.

### 8. AffordSim: A Scalable Data Generator and Benchmark for Affordance-Aware Robotic Manipulation

- **Paper:** [arXiv:2604.11674](https://arxiv.org/abs/2604.11674), submitted **2026-04-13**.
- **Lab:** Xuguang Lan's robotics/AI group at Xi'an Jiaotong University.
- **Credibility signals:** accepted as a **NeurIPS 2026 poster** (listed in the [official NeurIPS 2026 downloads](https://neurips.cc/Downloads/2026)); citations **0** ([Semantic Scholar](https://www.semanticscholar.org/paper/1adbe7f56d7a64a14e86c54f8156b0357ee8a4a9)); 50 tasks, seven task categories, five robot embodiments, 500+ objects, four imitation-learning baselines, and zero-shot real Franka FR3 tests. No public repository was found; HF upvotes unknown.

**What they did.** AffordSim integrates open-vocabulary 3D affordance prediction into an Isaac Sim data engine: language produces an affordance query, VoxAfford grounds it on the point cloud, and grasp/motion planners collect region-conditioned trajectories. It reaches **93% of manual-annotation collection success** on affordance-critical tasks and **89%** on hard composite tasks. Policies trained on the generated data achieve **24% average zero-shot real-robot success**; benchmark results also expose large gaps on pouring (1–43%) and mug hanging (0–47%).

**Borrow for our project:**

- Add an Isaac Sim predicate-data generator that varies object, pose, texture, lighting, camera, and distractors while recording masks, affordance regions, planner feasibility, and outcome labels.
- Make the motion/grasp planner part of label generation: a semantic region is useful only if it yields an executable action, which gives the head stack-native feedback.
- Reuse its task taxonomy to build hard semantic checks around handles, rims, openings, avoid-regions, and support surfaces.

**Difference from our project:** AffordSim generates demonstrations for end-to-end policies; our system generates symbolic programs and uses affordance judgments only as typed runtime checks and planner inputs.

### 9. Knowing When Not to Answer: Cross-Domain and Multi-Turn Generalization of Latent Underspecification Signals

- **Paper:** [arXiv:2610.08413](https://arxiv.org/abs/2610.08413), submitted **2026-10-06**.
- **Lab:** ITMO University.
- **Credibility signals:** citations **0** ([Semantic Scholar](https://www.semanticscholar.org/paper/8430cd4d338eb87ca61294423d9dbc9414f26849)); six datasets, six open-weight LLMs, three probe seeds, and a human-audited multi-turn benchmark of **423 conversations / 1,661 turn states** with a simulated answering user. Under review; code is promised but not yet linked; HF upvotes unknown.

**What they did.** Linear probes on a single prompt-only forward pass transfer well when the **cause** of missing information matches: math-to-math AUROC **0.77–0.97** and extractive QA transfer **0.77–0.90**. They transfer poorly across different causes of unanswerability, and single-turn probes do not automatically detect when a dialogue becomes answerable. A Platt-scaled gate reaches firing precision **0.74–0.88** versus 0.54–0.58 for random, but fails to beat vanilla/prompted strategies consistently end to end—an important negative result.

**Borrow for our project:**

- Calibrate separate heads or thresholds by uncertainty cause/predicate family; do not assume a `trash` gate transfers to `fragile`, occlusion, or missing-instruction cases.
- Build a scripted-user clarification harness: measure whether asking one question actually raises task success, not just whether the uncertainty detector has high AUROC.
- Always compare hidden-state probes with cheap lexical/metadata baselines and test after the user answer; decodability alone is not operational value.

**Difference from our project:** It studies text underspecification and dialogue, not multimodal object semantics or physical manipulation.

### 10. DepState-JEPA: Learning Depth-Grounded State Representation for Visual Perception and Manipulation

- **Paper:** [arXiv:2610.29246](https://arxiv.org/abs/2610.29246), submitted **2026-10-21**.
- **Lab:** Fudan University, Shanghai Innovation Institute, Shanghai Jiao Tong University, Shandong University, and Zhejiang University.
- **Credibility signals:** citations **0** ([Semantic Scholar](https://www.semanticscholar.org/paper/f9bf4ddb275a97fa30c4b4f349fdc5441e0cbeea)); [project/code repository](https://github.com/ShawnLeung-SII/depstate-jepa), **0 stars**; 20 DMControl and 50 Meta-World tasks plus two real-robot tasks, three seeds and **90 real trials per method/task**, with extensive component ablations. It is an ICLR 2027 submission, not accepted; HF upvotes unknown.

**What they did.** DepState-JEPA adds RGB/depth streams, metric ray conditioning, shared/private latent factorization, and action-conditioned state prediction to V-JEPA 2.1. The same latent supports local feature readouts, proprioception, forecasting, actor–critic control, and latent MPC. It reports **77.1%** Meta-World success with online policy learning, **90.1%** with latent MPC, and frozen sim-to-real full-task success of **81.1±5.1%** for drawer manipulation and **78.9±3.8%** for cup pick/place.

**Borrow for our project:**

- For spatial predicates, train small readouts over an RGB-D latent grounded in metric rays rather than over language features alone; keep shared scene state separate from modality-private appearance.
- Add action-conditioned prediction as a verification signal: compare the predicted postcondition embedding with the observed post-action embedding after every program line.
- Benchmark frozen readout quality, controller success, and sim-to-real success separately so a good embedding probe is not mistaken for a good robot system.

**Difference from our project:** It learns a control/world-model representation and has no open-vocabulary semantic predicates, calibrated abstention, demonstrated intent inference, or program synthesis.

## 3. Rejected examples

- [ANCHOR: A Physically Grounded Closed-Loop Framework for Robust Home-Service Mobile Manipulation](https://arxiv.org/abs/2604.25323) — rejected because the authors withdrew the manuscript after identifying errors and inconsistencies requiring substantial revision, despite relevant real-robot recovery results.
- [FLIP: Final Layer Inference-Time Probing for Vision-Language Models](https://arxiv.org/abs/2609.30993) — accepted at an ICML 2026 interpretability workshop and has code, but explicitly presents a mechanistic validation intervention rather than a calibrated classifier, abstention policy, or robot semantic predicate.
- [Density Ridge Selective Prediction for LLM and VLM Hallucination Detection under Calibration Label Scarcity](https://arxiv.org/abs/2606.10198) — interesting six-benchmark/eight-model abstract, but a single-author preprint with no verified venue, code, established lab, or embodied evaluation; it did not clear two independent credibility signals.
- [Instinct vs. Reflection: Unifying Token and Verbalized Confidence in Multimodal Large Models](https://arxiv.org/abs/2604.17274) — relevant fusion idea, but no accepted venue or released code was verified and the abstract gives no concrete evaluation scale or headline calibration numbers; weaker evidence than the retained selective-prediction papers.
- [Answer-Level Trust Selection for Physical Vision-Language Reasoning](https://arxiv.org/abs/2608.19807) — evaluation spans 20 VLMs, but the paper is a preprint with code only promised and relies on repeated black-box queries/interventions, making it a poor fit for the requested fast single-pass hidden-state head.

## 4. Takeaways

- **Use a factored, frozen decision head and an explicit cascade.** The most direct precedent is *Why Generate a Click?*: pool several informative layers, return typed distributions, and hand off only low-confidence cases. Keep this head separate from program-writer LoRA because joint training can damage the generator.
- **Calibrate the whole decision–inspection–ask policy.** BCEA shows that adding evidence acquisition after calibration can invalidate the risk guarantee. Calibrate after the exact sequence of crop, re-segmentation, extra view, and human-escalation tools is fixed.
- **Ground confidence in pixels and geometry.** SIEVES suggests scoring mask/localization quality and crop coherence; DepState-JEPA suggests metric RGB-D latents for spatial predicates. A predicate should expose `{value, probability, object_id, evidence_mask, evidence_type}` rather than a bare boolean.
- **Turn stack feedback into concepts, not only SFT examples.** ReSYNC is the closest overlap with learning relational predicates from failures; AffordSim and OpenDexGrasp supply complementary recipes for broad synthetic coverage plus small human-aligned data. The project's novelty should therefore be stated as combining one-video rule induction, readable sandboxed program synthesis, production-stack execution, typed calibrated semantic gates, and line-level model-assisted recovery—not as the first system to learn predicates from robot failures or to ground open-vocabulary affordances.
- **Evaluate operational value, not just probe AUROC.** Include risk–coverage/AURC and ECE/Brier score, but also human-question rate, added latency, post-clarification success, recovery success, and end-task success under new layouts/categories. AbstentionBench and the ITMO study warn that reasoning fine-tuning can hurt abstention and that a decodable uncertainty signal may still fail to improve the full interaction.
