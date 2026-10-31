# Related work: what recent papers did, and what we borrow

*   **Date:** 2026-10-31
*   **Method:** four independent scouts searched arXiv (April–October 2026), Semantic Scholar, Hugging Face
    Papers and GitHub. They kept only papers with at least two credibility signals (top venue,
    citations for their age, established lab, released code, real-robot or large-scale evaluation,
    community attention) and rejected papers without numbers, toy studies, position papers and withdrawn
    work. 36 papers were kept. Their reports, with search logs and rejected examples, are in `raw/`.
*   **Spot check:** titles of 9 cited papers verified on arXiv, all real.
*   **Caveat:** the numbers below come from the papers and are not reproduced by us.

## The closest prior work (state our novelty honestly)

| Paper | What overlaps with our design | Where we differ |
|---|---|---|
| **RAPID**, Robot Agentic Programming from Demonstrations ([2609.30249](https://arxiv.org/abs/2609.30249); MIT, NUS, UPenn, NVIDIA; code released) | One visual demo + language → task predicates → a reusable **relational program**, verified in a reconstructed simulation; 75.9 % vs. 14.6 % for a Code-as-Policies agent; real Franka | Builds task-specific trajectory-optimisation primitives with an agent offline. We write a short program over a fixed production grasp/motion API, run it without a model, recover line by line at run time, and fine-tune an open model on the stack's own feedback |
| **MoCaP**, Mobile Code-as-Policies from passive human videos ([2610.20998](https://arxiv.org/abs/2610.20998); NYU, Fudan; code) | Short Python programs over trusted primitives; stop at the first failed call; return diagnostics; the agent writes the rest; 80 % vs. 0 % for CaP-X | Its video gives environment memory, not the task rule, and its model writes programs during normal execution. Ours compiles one program from the demonstrated rule and calls the model only on failure |
| **Dex-One2Many** ([2610.12470](https://arxiv.org/abs/2610.12470)) | Extracts a relational task rule (entities, predicates, order) from one demo, not a trajectory | No readable program, no sandboxed API, no run-time recovery |
| **Coding Agents with Harness for Safe Robot Control** ("SafeHarness", [2609.20822](https://arxiv.org/abs/2609.20822)) | Constraints as executable harness checks; phase-structured programs; 81.2 % task success, 91.9 % collision avoidance | Keeps a frontier model in the loop; no demonstration |
| **Why Generate a Click? Plug-in Decision Heads** ([2610.20107](https://arxiv.org/abs/2610.20107)) | Typed decision heads on a **frozen** VLM's hidden layers, with a confidence cascade (head if confident, otherwise generate) | GUI agents, not robot predicates |

**What we can claim:** no kept paper combines all of these:
*   one phone video → an explicit rule → a readable, sandboxed program on a production robot API;
*   execution with no model in the loop;
*   model-assisted recovery at the failing line;
*   calibrated typed semantic checks that ask the user when unsure;
*   fine-tuning an open model on the stack's own verified traces.

**What we must not claim:**
*   the first to turn one video into a robot program (RAPID);
*   the first to extract a rule from a demo (Dex-One2Many);
*   the first fail-fast code-as-policy with diagnostics (MoCaP).

## Ideas to borrow, by component

### Program format (V2)

*   **Relational programs with postconditions** (RAPID): every state-changing call states its expected
    effect (`place(o, box)` → `inside(o, box)`), checked right after the call. A failed postcondition is a
    typed failure for recovery and a labelled example for training.
*   **Phase structure** (SafeHarness): `route → grasp → route → release`, so recovery repairs the failed
    phase instead of rewriting the program.
*   **Never bake demo coordinates into a program;** resolve objects and geometry at run time (RAPID,
    MoCaP). Our `find(...)` already does this.

### Runtime and sandbox (V2)

*   **Isolation boundary** (HexaShell, [2610.23479](https://arxiv.org/abs/2610.23479), 238 GitHub stars):
    a small allow-listed API, no simulator handle, structured JSON results.
*   **A "delivery check" before accepting a repair:** parse, allow-list scan and schema check, then a replay
    on the failed layout and on held-out variants.
*   **Fail-fast block semantics** (MoCaP): stop at the first failed call; hand over the call, a structured
    diagnostic, a fresh observation and the history; resume from the repaired remainder.
*   **Keep the success scorer hidden from generated code** (CodeActionBench): our rule checker reads Isaac's
    ground truth, which programs never see.

### Recovery (V2, V5)

*   **A tiered, budgeted ladder** (recovery scout): continue → re-observe → retry the step → patch → ask
    the user, each escalation with a logged reason. This replaces our free-form "retry / skip / resume /
    stop" and makes recovery measurable per tier.
*   **Report recovery by failure class**, as MoCaP and CodeActionBench do: perception, reachability,
    collision, grasp, verification.

### Training data and fine-tuning (V2, V6)

*   **Same-scene success/failure pairs** (Robot Critics, [2606.21572](https://arxiv.org/abs/2606.21572)):
    record a failed and a successful attempt in the same scene. These make strong recovery data and DPO
    pairs.
*   **Verified SFT first, then RL in the sandbox** (ReTool, [2504.11536](https://arxiv.org/abs/2504.11536)),
    with **dense step rewards** (Verifiable Process Rewards, [2605.10325](https://arxiv.org/abs/2605.10325)):
    *   rewards for parse validity, legal calls, step postconditions, recovery and task success;
    *   this matches the recipe's GRPO step, planned for the cloud.
*   **Scene perturbation for verification and data** (RAPID): perturbing poses and adding distractors
    improved its success from 53 % to 76 %. Our random layouts do this; add distractor and size
    perturbations.

### Semantic checks, the System One head (second demo)

*   **Frozen VLM + factored typed heads + a confidence cascade** (Plug-in Decision Heads):
    *   read a mix of several hidden layers, not only the last;
    *   **train the heads after freezing the model;** joint training hurt generation by up to 4.9 points.
        So keep the head separate from the program-writer LoRA.
*   **Predicates return evidence:** `{value, probability, object_id, evidence_mask}`, not a bare boolean
    (SIEVES). Calibrate the whole decide → inspect → ask policy, after its order is fixed (BCEA).
*   **Verifiers fail on contact tasks** (FailBench, [2609.03611](https://arxiv.org/abs/2609.03611): balanced
    accuracy 0.77 overall, near chance on contact, biased towards "success"). Our final `verify` uses ground
    truth in the sim; on real hardware, a VLM verifier needs its own calibration.

## Metrics to add (cheap and credible)

*   **Joint task-and-safety success:** a task completed with a collision does not count (SafeHarness).
*   **Recovery:** success after injected failures, conditional on detection; model calls and time to
    recover; unnecessary interventions.
*   **Semantic checks:** ECE/Brier score, risk–coverage, and the rate of questions to the user.
*   **Program writing:** first-try validity, success on held-out layouts **and on held-out rules**.

## For the next 4 days (priority)

1.  **Postconditions** on `pick`/`place` in the program format, and **typed failure classes**.
2.  **Fail-fast plus the tiered recovery ladder** in the runtime, with a recovery budget.
3.  **Scene perturbation with distractors** in the task suite, and a held-out-rule split.
4.  **The novelty statement above** in the README and the video. Cite RAPID and MoCaP as related work.

Everything on the System One head, process rewards and RL stays as next steps.
