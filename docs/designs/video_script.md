# Video script (about 3:00)

The owner narrates. Every number on screen comes from [learning_results.md](learning_results.md), or from a run
recorded for the video. Shots marked *(to measure)* are filmed only if the measurement supports them.

## 0:00–0:15 · Hook

*   **Visual:** the owner's real clip: unloading a stack of cartons into a bin, wearing gloves. Cut to Isaac: the
    robot does the same task in a different layout.
*   **Narration:** "I showed a robot a task once. It worked out the rule, wrote its own program, and did it in a
    scene it had never seen."

## 0:15–0:30 · The problem

*   **Visual:** a diagram: video → intent → program → robot, with a person approving between the steps.
*   **Narration:** "In a factory you can't let a language model steer a robot freely. So the model doesn't drive
    the arm. It writes a short, readable program on top of a real robot stack, and a person approves every step."

## 0:30–0:55 · The system, live

*   **Visual:** the panel beside Isaac:
    *   the request, or the clip;
    *   **Rules** ("Every cube → into the red box");
    *   a clarification ("only the red ones");
    *   Confirm;
    *   the program;
    *   ▶ Play;
    *   the live call log;
    *   the ✓ checklist.
*   **Narration:** "It reads the demonstration into explicit rules I can check and correct. It writes a program.
    Nothing moves until I press Play. Afterwards an independent checker verifies the result against ground
    truth."

## 0:55–1:25 · Learning from experience

*   **Visual:** tower unload (`stacking:100007`), side by side:
    *   the untrained model grabs the bottom carton and fails;
    *   the fine-tuned model works top-first and passes.

    Then the chart.
*   **Data:**
    *   61% → **86%** on 250 held-out tasks, 3 runs each;
    *   stacking 47% → 97%;
    *   an unseen task family 51% → 82%;
    *   Isaac physics 34/50 → 41/50.
*   **Narration:** "We fine-tuned a small open model on what the robot actually achieved in simulation: learning
    from experience. It learned which order works for which kind of task."

## 1:25–1:50 · Small beats big *(to measure)*

*   **Visual:** the same scene twice:
    *   the large hosted model (Muse Spark Open) writes a program that fails, and the log shows why;
    *   our fine-tuned model passes ✓.

    Then a bar chart over all 250 tasks: base model, fine-tuned model, large model.
*   **Data:**
    *   a task where the large model fails all 3 runs and ours passes all 3, confirmed in Isaac;
    *   the large model's overall score (X%) next to 86%.
*   **Narration:** "A model a hundred times larger still fails here, because it has never seen what this robot can
    actually do. Experience beats size on this task." On the overall numbers: "On average it scores X%."
*   **If the measurement does not support it** (the large model wins everywhere), replace the shot with the
    honest comparison bar chart and the line "a small model gets within N points of one a hundred times its
    size".

## 1:50–2:10 · What didn't work

*   **Visual:** a short table.
*   **Data:**
    *   training on its own successes brought its old habits back (69–74%);
    *   preference training and reinforcement learning added nothing, because nearly every attempt in a group got
        the same reward;
    *   it learned a rule per task type, not judgement per scene (86% vs a 91% oracle);
    *   physics is harder than planning (82% vs 88%).
*   **Narration:** "Some things didn't work, and that shows where the real problem is."

## 2:10–2:45 · Industrial finale: a real limit

*   **Visual:**
    *   a carton too wide for the gripper;
    *   the small untrained model just tries to pick it;
    *   a strong reasoning model composes a push from the robot's existing motions: it approaches from behind,
        touches with a 10 N guarded move, then makes one soft push;
    *   **"Waiting for you"** appears, and the owner approves;
    *   the carton lands **3 mm** from the target, never above 12 N.
*   **Narration:** "The robot already knows how to move, approach and push. The reasoning model works out how to
    combine those capabilities to solve a problem it cannot handle with a grasp. And it still asks first."
*   **Framing (keep the claim exact):** the model does not discover a new physical capability. It composes the
    robot's existing, force-limited motions into a different strategy. On screen, show those calls as they were
    issued: `move_to_contact`, then `move_compliant`.

## 2:45–3:00 · Close

*   **Visual:** the panel with ✓, then the repository link.
*   **Narration:** "Show once, verify, approve, run. This isn't certified safety; a real cell still needs its own
    safety layer. But the model never moves anything on its own say-so."

## Shots and their status

| # | Shot | Status |
|---|---|---|
| 1 | Owner's real clips (cartons or cubes, gloves) | Owner |
| 2 | Panel + Isaac full run | Ready to film; needs G1b served on a GPU |
| 3 | Tower unload: untrained vs fine-tuned | Measured; needs the preset, the panel's model switch, and G0 + G1b served |
| 4 | Small beats big | Needs the large-model benchmark (Baseten key) |
| 5 | Charts | To build from the results |
| 6 | Push finale with approval | Physics proven; needs approval for model-issued motions and the live agent |
| 7 | "What didn't work" table | To build from the results |
