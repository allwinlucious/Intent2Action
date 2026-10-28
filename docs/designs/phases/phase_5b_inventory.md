# Phase 5b inventory: `TareWrench`, `MoveToContact`, `MoveCompliant`, CUJ 6

Part of [robot_platform.md](../robot_platform.md) (G8, CUJ 6, "RobotService
API"). This is a factual inventory. It proposes nothing beyond one-line
consequences, and those are marked **derived** when no source line states them.

Path prefixes:

| Prefix | Path |
|---|---|
| `ic/` | `/home/mew/intrinsic-core` (branch `fix/so3-from-matrix-reject-reflections`, HEAD `b2f9336`) |
| `project/` | `/home/mew/untitled-project` (HEAD `82240d4`) |
| `ICON/` | `ic/intrinsic_control/intrinsic/icon` |
| `APIS_ICON/` | `ic/intrinsic_apis/intrinsic/icon` |
| `FORCE/` | `ic/intrinsic/manipulation/skills/force` |

Method: I read the sources, searched them with `rg`, read the existing project
logs, and ran two experiments that write nothing to either repo. I did not run
Bazel, start Isaac Sim, start Docker or start ICON. I changed no file except
this one.

| ID | What | Command / result |
|---|---|---|
| X1 | Do the force-action protos compile with our proto include roots? | `source project/container/proto_includes.sh`, then `grpc_tools.protoc "${INCLUDES[@]}" --include_imports --descriptor_set_out` for `intrinsic/icon/actions/force_primitive.proto` and `cartesian_admittance_action.proto`, then `--python_out` into `/tmp/p5b_protos.*` (deleted afterwards). Result: 20 non-WKT files, all compiled; a `ForcePrimitiveFixedParams` with `make_contact` and `virtual_translational_inertia` built and serialized (28 bytes). |
| X2 | Approach-speed arithmetic (§1.4) | `python3 -c` with the formula from §1.4; the numbers are quoted there. |

---

## 0. Key findings

1.  **`TareWrench` is already done (phase 3).** It runs
    `intrinsic.tare_force_torque_sensor` on `ft_sensor` with a 5 s sim-time
    timeout (`project/container/services/robot_service.py:677-687`, `:82`).
    The phase 3 and phase 4 checks passed: "force norm 5.753 N before,
    0.000 N after taring" (`project/docs/designs/phases/phase_3_notes.md:24`,
    `phase_4_notes.md:491`). `MoveToContact` and `MoveCompliant` are still
    `_unimplemented(..., "5b")`, which returns UNIMPLEMENTED
    (`robot_service.py:689-690`, `:588-592`).
2.  **Intrinsic's Python helpers for these actions cannot be imported in
    `robot_service` as it is built today.** `force_primitive_utils.py` and
    `cartesian_admittance_utils.py` are not in the vendored module list
    (`project/container/stage_vendor.sh`). They import
    `intrinsic.manipulation.skills.force.utils` (`ICON/actions/force_primitive_utils.py:17-25`).
    That module imports `directional_stiffness` and `overdamping` from
    `ICON/control/algorithms/python/`, which are **pybind11 C++ extensions**
    (`ICON/control/algorithms/python/BUILD:22-42`; only `.cc` sources exist).
    They are not built (`ls ic/bazel-bin/intrinsic_control/intrinsic/icon/control/algorithms/python/` → no such directory).
    The protos are not the problem: they compile with our include roots (X1).
    The `gen/` tree does not contain them yet (`ls project/gen/intrinsic/icon/actions`
    lists only joint_jogging, point_to_point_move, simple_gripper,
    tare_force_torque_sensor and trajectory_tracking). **Derived:** the
    parameters can be built as plain protos. `intrinsic.force_primitive`
    computes its damping on the server side (§1.3), so it needs no Python math.
3.  **`MakeContact` has no speed field.** The free-space approach speed is
    \(v = F/D\), where \(D\) comes from virtual inertia and the assumed contact
    stiffness (§1.4). For CUJ 6 (15 N, 10 mm/s) at MEDIUM stiffness
    (60000 N/m), the virtual translational inertia is about 9.19 kg (X2). The
    `max_approach_velocity` field of the skill's proto is declared but no
    code reads it (§5.1).
4.  **No real-time code enforces a force limit.** `excessive_force_threshold`
    and `excessive_torque_threshold` appear only in Python. Neither the
    `move_to_contact` skill's argument check nor `ForceControlParams` enforces
    them in real time (`rg excessive_force_threshold -g '*.cc' -g '*.h' -g '*.py' ic`
    → only `FORCE/utils.py:235-268` and `FORCE/move_to_contact.py:229,283`).
    MakeContact regulates the force *toward* `max_contact_force`; it does not
    stop there (§1.3). **Derived:** both RPCs must end through ICON reactions on
    state variables, as the skill does (§5.1).
5.  **`is_done` of both actions means "reference pose reached"**, measured on
    the controller's *commanded* pose (`ICON/actions/cartesian_admittance_info.h:45-47`;
    `ICON/control/actions/cartesian_admittance_action.cc:478-481`). For
    MakeContact the reference pose is the start pose (§1.3), so `is_done` does
    not signal contact. **Derived:** for MoveCompliant, a blocked peg may never
    come within 1 mm of the target (§2.4).
6.  **Our ICON config gives the arm every interface the admittance slot needs.**
    The slot needs CARTESIAN_LIMITS, JOINT_LIMITS, JOINT_POSITION(_SENSOR),
    JOINT_VELOCITY_ESTIMATOR and MANIPULATOR_KINEMATICS
    (`cartesian_admittance_action.cc:734-774`). The log confirms that Cartesian
    limits are registered. MANIPULATOR_KINEMATICS is inferred, not logged (§3).
    No dynamics model and no torque state are required.
7.  **Our `SetPayload` does not reach the F/T compensation.** It writes the arm's
    `full_payload` (`project/container/services/robot/icon.py:103-114`). The
    F/T part compensates with its own `mounted_payload` + `grasped_payload`
    (`ICON/control/parts/feature_interfaces/force_torque_sensor.cc:553-556`),
    and we set neither (§4.3).
8.  **The admittance controller runs 4 substeps per 250 Hz cycle on one sensor
    sample.** It integrates the commanded state, not the sensed state. It never
    calls IK. Details and other real-time assumptions are in §6.

---

## 1. `intrinsic.force_primitive`

### 1.1 Registration and wiring

- Type name `"intrinsic.force_primitive"`. Its fixed params are
  `ForcePrimitiveFixedParams`. Its streaming params are
  `CartesianAdmittanceActionFixedParams` and its streaming output is
  `CartesianAdmittanceStatus` (`ICON/actions/force_primitive_info.h:24-37`).
- The factory builds `CartesianImpedanceParameters` from the fixed params. It
  then creates a `CartesianAdmittanceAction`, or the impedance action if
  `force_control_settings.use_impedance_control` is set
  (`ICON/control/actions/force_primitive.cc:34-48`). The registration copies
  the admittance action's signature, so slots and state variables are the same
  (`force_primitive.cc:26-36`).
- The slots are `"arm"` and `"ft_sensor"` (`ICON/actions/compliance_info.h:28-34`).
  The Python helper maps them the same way (`ICON/actions/force_primitive_utils.py:80-90`).

### 1.2 Parameter protos

`ForcePrimitiveFixedParams` (`ICON/actions/force_primitive.proto:54-69`):

| Field | # | Notes |
|---|---|---|
| `oneof primitive { ForcePrimitive force_primitive }` | 1 | |
| `ForceControlSettings force_control_settings` | 2 | see below |
| `TaskParams task_params` | 3 | |
| `bool disable_reference_lowpass_filter` | 4 | disabled → filter constant 1.0 (`action_builder.cc:74`, `:459-472`) |

`TaskParams` (`force_primitive.proto:31-51`): `robot_tip_t_robot_tool`=1 (Pose),
`robot_base_t_task`=4, `environment_stiffness`=3 (`ContactStiffnessParams`),
`optional joint_limits`=5, `optional cartesian_limits`=6 ("Action will fail if a
motion target violates Cartesian limits"), reserved 2. If
`environment_stiffness` is unset, `BuildParams` returns an error
(`ICON/control/primitives/force_control/force_primitive_utils.cc:120-122`).

`ForcePrimitive` oneof (`ICON/control/primitives/force_control/proto/force_primitives.proto:188-197`):
`make_contact`=1, `apply_force`=2, `align_rotation_with_frame`=4, `hold`=5,
reserved 3. `Push` is defined (`:136-179`) but **is not in the oneof**.

- `MakeContact` (`:94-102`): `motion_direction`=1 (`Direction`),
  `max_contact_force`=2 (N), `optional controller_params`=3.
- `Direction` (`:27-42`): either `axis` (X, Y, Z, MINUS_X, …) or `vector`. A
  zero-norm vector is an error (`force_primitive_utils.cc:84-89`).
- `ApplyForce` (`:82-91`): up to 3 `DirectionAndForce` and 3 `AxisAndTorque`,
  plus controller_params. `AlignRotationWithFrame` (`:108-128`) has a default
  rotational stiffness of 100. `Hold` (`:182-185`).
- `ControllerParams` (`.../proto/controller_params.proto:27-57`):
  - translational oneof: `translational_stiffness_orthogonal_to_direction`=1,
    or `no_translational_compliance`=4;
  - rotational oneof: `rotational_stiffness_orthogonal_to_direction`=2, or
    `no_rotational_compliance`=5.
  - The code defaults are 300 N/m and 300 Nm/rad
    (`ICON/control/primitives/force_control/action_builder.cc:66-69`;
    `force_primitive_utils.cc:37-39`). The proto comment says 100 N/m; the
    code wins.

`ForceControlSettings` (`APIS_ICON/equipment/force_control_settings.proto:23-66`):
`excessive_force_threshold`=1, `excessive_torque_threshold`=2,
`virtual_translational_inertia`=3 (kg), `virtual_rotational_inertia`=4,
`sensed_wrench_deadband`=5 (Wrench), `optional enforce_cartesian_and_joint_limits`=6,
`optional use_impedance_control`=7 (UR only),
`spectral_stability_monitor_settings`=9 (`:73-93`: crossover 4 Hz, force rms
10 N, torque rms 4 Nm). The stability monitor only feeds
`part_status.wrench_stability_index`
(`ICON/control/parts/feature_interfaces/force_torque_sensor.cc:475`,
`ICON/control/realtime_part_status_helpers.cc:164-165`). No action reads it
(`rg` over `control/actions/*.cc` finds nothing).

`ContactStiffnessParams` (`ic/intrinsic/manipulation/skills/force/contact_stiffness.proto:23-55`):
either `mechanical_stiffness_at_contact`=1 (N/m), or `default_contact_stiffness`=2
(LOW 5000, MEDIUM 60000, HIGH 100000).

### 1.3 What `BuildParams` and `ApplyMakeContact` produce

- `ApplyMakeContact` (`action_builder.cc:191-242`):
  - normalizes the direction (task frame);
  - sets `tool_reference_wrench.force = dir · max_contact_force`;
  - sets the wrench selection matrix to all six directions by default, only
    the motion direction if `no_translational_compliance`, and no rotation if
    `no_rotational_compliance`;
  - sets directional stiffness to **0 along the direction** and the orthogonal
    defaults elsewhere;
  - sets `force_direction_1_in_task = dir`.
  - It sets no reference pose. An unset reference pose becomes the last
    commanded pose
    (`ICON/control/algorithms/cartesian_impedance_commons.cc:263-284`).
- `BuildParams` (`action_builder.cc:476-583`):
  - copies `tip_t_tool`, `base_t_task`, joint and Cartesian limits, and the
    deadband;
  - sets a 1.0 s window;
  - sets inertia = diag(m_trans ×3, m_rot ×3) (`:543-546`);
  - sets damping = `ComputeOverdamping(inertia, environment stiffness)`
    (`:146-159`, `:576-580`).
- The controller drives the force error \(-(F_{sensed} - F_{goal})\) in the
  tip frame, with a deadband
  (`ICON/control/algorithms/cartesian_admittance_controller.cc:191-366`).
  **Derived:** with zero stiffness along the direction, the steady state is
  \(F_{sensed} = F_{max}\) along it. The robot keeps pushing at `max_contact_force`
  until another action replaces this one.

### 1.4 Mapping direction, force and speed

- **direction** → `MakeContact.motion_direction.vector` in the task frame. With
  `robot_base_t_task` = identity, that is the arm base frame, which is what the
  skill does (`FORCE/move_to_contact.py:332-346`). Our API gives the direction
  "in the base frame" (`project/robot_service/robot_service.proto:162-170`).
- **max_force** → `MakeContact.max_contact_force`.
- **speed** has no field. Overdamping is `kMockStiffness = 1234` and Benady
  \(\sqrt{1+\kappa}\) with \(\kappa = k_{env}/1234\)
  (`ICON/control/algorithms/overdamping.cc:96-97`, `:110-129`;
  `overdamping.h:38-45`). Critical damping is \(2\sqrt{m k}\)
  (`compute_critical_damping.cc:27-37`). Together, per translational axis:
  \[ D = 2\sqrt{m\,(1234 + k_{env})}, \qquad v_{free} = F/D \]
  The tutorial states the same relation, \(v = D^{-1}F\)
  (`ic/developer_resources/learn/tutorials/interactive_tutorials/icon/client_api/move_to_contact_tutorial.md:232-234`).
  The Python helper rejects \(F/D >\) max translational velocity
  (`ICON/actions/cartesian_admittance_utils.py:214-279`).
  X2 numbers:

  | m (kg) | k_env (N/m) | F (N) | D (N·s/m) | v (mm/s) |
  |---|---|---|---|---|
  | 15 | 60000 | 10 | 1917 | 5.22 |
  | 15 | 60000 | 15 | 1917 | 7.83 |
  | 15 | 5000 | 15 | 612 | 24.53 |
  | 1 | 60000 | 15 | 495 | 30.31 |
  | **9.19** | **60000** | **15** | **1500** | **10.00** |
  | 15 | 100000 | 15 | 2465 | 6.09 |

  **Derived:** for a requested speed, \(m = (F/2v)^2 / (1234 + k_{env})\). CUJ 6
  with MEDIUM gives 9.19 kg; with LOW it gives 90.2 kg. The tutorial says
  "virtual inertia must always be higher than the actual physical payload
  inertia" (`move_to_contact_tutorial.md:81`).
- The reference wrench is ramped by the lowpass filter. It is filtered with
  alpha·speed_override, but only the twist, not the wrench, is scaled by the
  override (`ICON/control/algorithms/simple_cartesian_impedance_reference_generator.cc:68-145`).
  **Derived:** `SetSpeed` slows the ramp-up but not the MakeContact speed.

### 1.5 Python helpers (Intrinsic; not importable in our stack, finding 2)

- `build_task_params(robot_tip_t_robot_tool: Pose3, robot_base_t_task: Pose3, contact_stiffness)`.
  It does not set joint or Cartesian limits (`force_primitive_utils.py:34-46`).
- `create_make_contact_action(action_id, joint_position_part_name, ft_sensor_part_name, primitive: MakeContact, force_control_settings: utils.ForceControlParams, task_params, disable_reference_lowpass_filter=False)`
  (`force_primitive_utils.py:49-67`).
- `ForceControlParams` has `sensed_wrench_deadband`, `excessive_force/torque_threshold`
  and `virtual_translational/rotational_inertia`. `.proto()` omits fields 6–9
  (`FORCE/utils.py:230-271`). `from_resource_handle` reads
  `Icon2ForceTorqueSensorPart.force_control_settings` (`:273-297`).
  `StateVariables` is re-exported from `cartesian_admittance_utils`
  (`force_primitive_utils.py:29`).

### 1.6 State variables and completion

Names are in `ICON/actions/compliance_info.h:35-136` and the Python aliases in
`cartesian_admittance_utils.py:37-45`:

| Variable | Meaning |
|---|---|
| `intrinsic.sensed_force` | Euclidean norm of the sensed force |
| `intrinsic.sensed_torque` | Euclidean norm of the sensed torque |
| `intrinsic.sensed_force_in_direction_1..3` | signed projection on `force_direction_*` (`ICON/control/actions/admittance_state_variables.cc:302-307`) |
| `intrinsic.reference_pose_reached`, `reference_position_error` | |
| `intrinsic.settled_for_seconds` | |
| `elapsed_time_seconds` | |
| `intrinsic.translational_distance_traveled` | |
| `maximum_translational_displacement_in_time_window` | |
| `intrinsic.is_settled` | |
| `manipulability` | |

`intrinsic.is_done` (`ICON/common/builtins.h:42`) = reference pose reached
(`cartesian_admittance_action.cc:478-481`), with the defaults position 1e-3 m
and orientation 1e-2 rad (`ICON/actions/cartesian_impedance_parameters.h:74-91`).

`is_settled` is fed with `force_reached` (wrench error norm < `force_settled_band`,
default 3 N) and the previous *commanded* joint velocities
(`cartesian_admittance_action.cc:500-507`). It does not use the sensed ones.

Our client already has the building blocks:
- the vendored `_session.py` is identical to `ic`'s (`diff` printed nothing);
- `Session` has `add_action_sequence` (`:465`), `add_actions` (`:433`),
  `add_transition` (`:591`), `add_reaction` (`:627`) and `get_latest_output`
  (`:827`);
- `reactions.py` has `is_done` (`:121`), `is_true` (`:134`),
  `is_greater_than` (`:314`), `is_less_than` (`:380`), `any_of` (`:464`) and
  `all_of` (`:484`).

`Icon.run_action` runs **one** action with one condition. It polls a
non-real-time event flag every 50 ms of wall time and ends the session on
return (`project/container/services/robot/icon.py:130-169`).

---

## 2. `intrinsic.cartesian_admittance_action`

### 2.1 Parameters

- `CartesianAdmittanceActionFixedParams{cartesian_impedance_parameters=1}`
  (`ICON/actions/cartesian_admittance_action.proto:27-72`, which also has the
  status fields).
- `CartesianTarget` (`ICON/actions/cartesian_impedance.proto:33-82`). All
  fields are in the task frame:
  - `robot_tip_t_robot_tool`=9;
  - `optional robot_base_t_task`=11 (identity = base);
  - `optional task_t_tool_reference_pose`=1 (unset = current pose);
  - `tool_reference_twist`=2 and acceleration=3;
  - `tool_reference_wrench`=4;
  - `virtual_cartesian_inertia`=10;
  - `cartesian_stiffness`=6;
  - `optional cartesian_damping`=7 (unset = critical);
  - `optional wrench_selection_matrix`=8 (must be a projection).
- Also in `cartesian_impedance.proto`: `NullspaceTarget` (`:84-93`),
  `Constraints` (`:95-108`), `AlgorithmConfiguration` (`:110-138`: pinv
  damping, deadband, integrator, post-sensor load multiplier),
  `StateVariableConfiguration` (`:140-191`) and `CartesianImpedanceParameters`
  (`:193-201`).
- Validation in `Preprocess` (`cartesian_impedance_commons.cc:60-143`):
  - inertia must be positive definite and symmetric;
  - stiffness must be positive semi-definite;
  - unset damping becomes `ComputeCriticalDamping` (`:113-118`).
  - The nullspace default is stiffness 10 with damping \(2\sqrt{k}\)
    (`:145-164`, `cartesian_impedance_commons.h:42`).
- Reference generators (`ICON/actions/cartesian_impedance_reference_generators.proto:21-89`):
  - `simple_lowpass` (default alpha 0.005 per substep,
    `cartesian_impedance_parameters.h:100,110`);
  - `quintic_spline(movement_duration_seconds)` (default 10 s, `:103`);
  - position and velocity Reflexxes (empty messages).
  - The speed override range is [0, 1] for all of them
    (`simple_cartesian_impedance_reference_generator.h:42-43` and the
    siblings).
- Python helper (not importable, finding 2):
  `create_cartesian_admittance_action(action_id, joint_position_part_name, ft_sensor_part_name, *, virtual_cartesian_inertia, cartesian_stiffness, cartesian_damping=None, robot_tip_t_robot_tool=None, task_t_tool_reference_pose=None, tool_reference_twist=None, tool_reference_wrench=None, joint_limits=None, cartesian_limits=None, sensed_wrench_deadband=None, state_variable_config=None, lowpass_filter_constant=None)`
  (`cartesian_admittance_utils.py:82-211`). Also
  `create_approach_contact_action` (`:282-355`) and
  `create_stabilize_contact_action` (lowpass 1.0, `:358-431`).

### 2.2 Moving toward a pose while compliant

- Set `task_t_tool_reference_pose` = target, `cartesian_stiffness` = diag(k, k,
  k, k_rot…) and inertia, and leave damping unset to get critical damping.
- `SetReference` rejects targets outside the Cartesian limits:
  - "Cartesian goal violates Cartesian limits."
    (`simple_cartesian_impedance_reference_generator.cc:53-66`;
    `cartesian_admittance_action.cc:335-405`, `:374-377`);
  - the controller faults on "base_t_target violates Cartesian position box
    limits" and on nominal twist outside the velocity box
    (`cartesian_admittance_controller.cc:350-363`).
  - Our limits are ±2.0 m position and ±1.7 m/s
    (`project/robot_model/franka_scene_object.py:49-61`).
- The lowpass reference moves from the start pose to the target with
  τ ≈ 1/(0.005·1000) ≈ 0.2 s at the effective 1 kHz (X2: 0.199 s).
  **Derived:** a 3 cm step peaks at about 0.15 m/s of reference speed. The
  quintic spline bounds this with an explicit duration.
- **Derived:** with stiffness 200 N/m, a target 3 cm below a blocked peg
  produces at most about 6 N of spring force. CUJ 6's `max_force` of 20 N has
  no counterpart in the action.

### 2.3 "Tare immediately before"

The action description says: "requires the 'TareAction' to be executed
immediately before control starts… It also requires the post-sensor inertia to
be configured correctly… PayloadEstimation skill"
(`ICON/actions/cartesian_admittance_info.h:26-43`). `OnEnter` uses the current
sensed `WrenchAtTip` as the initial reference wrench, and the previously
commanded state as the initial target (`cartesian_admittance_action.cc:213-333`).
The skill tares as the first action of its own sequence
(`FORCE/move_to_contact.py:411-447`). CUJ 6 instead calls `TareWrench` as a
separate RPC before `MoveToContact` (`project/docs/designs/robot_platform.md:231-241`).

### 2.4 Completion

- `is_done` means the commanded pose is within 1 mm and 0.01 rad of the
  reference (§1.6).
- The robot keeps running admittance after `is_done` until another action or
  the end of the session.
- **Derived:** a peg that jams before 1 mm of the target never sets `is_done`.
  That case needs a condition on `is_settled`, `settled_for_seconds`,
  `elapsed_time_seconds` or `sensed_force`.

---

## 3. Arm and F/T part features with our config

- Slot requirements (`cartesian_admittance_action.cc:734-774`):
  - arm: JOINT_POSITION, JOINT_POSITION_SENSOR, JOINT_VELOCITY_ESTIMATOR,
    CARTESIAN_LIMITS, JOINT_LIMITS, MANIPULATOR_KINEMATICS;
  - ft_sensor: FORCE_TORQUE_SENSOR.
  - The arm chain must not branch (`:118-124`).
- **CARTESIAN_LIMITS: yes.** The log shows "hal_arm_part.cc:548] Registering
  Cartesian limits interface from world."
  (`project/logs/phase4_container_restarted.log`; same in `logs/phase1_run.log:137`).
  Limits come from the world when `hardware_resource_name` is set
  (`ICON/control/parts/hal/arm_part/hal_arm_part.cc:522-555`).
- **MANIPULATOR_KINEMATICS: inferred, not logged.** `RegisterManipulatorInterface`
  runs when both an IK solver and a kinematics model exist (`hal_arm_part.cc:569-633`).
  The log shows that the IK solver was accepted: "Solver 'kinematic_chain' is
  not realtime safe, but we're allowing it because allow_non_real_time_inverse_kinematics_
  is true" (`phase4_container_restarted.log`; `logs/phase1_run.log:143`;
  `ICON/control/services/kinematics_from_world_service.cc:136-170`). Our config
  sets `kinematics_from_world_service: true` and
  `allow_non_real_time_inverse_kinematics: true`
  (`project/container/config/icon.txtpb:8-10`).
- **Not needed:**
  - dynamics model (`dynamics_model_name` is optional, `hal_arm_part.cc:215-244`);
  - `process_wrench_command` (optional, `:355-364`; the action uses it only if
    present, `cartesian_admittance_action.cc:715-724`);
  - `joint_torque_state`, which is configured anyway (`icon.txtpb:20-28`;
    `hal_arm_part.cc:509-520`).
- **FORCE_TORQUE_SENSOR: yes.** The tare action, which needs it, passes (finding 1).
- **Not in our F/T config:** `force_control_settings` (`icon.txtpb:56-76`).
  **Derived:** `ForceControlParams.from_resource_handle` would have nothing to
  read, and phase 5b must supply the numbers itself.
- Logged fallbacks: "No payload set in world…" (`hal_arm_part.cc:1033`) and
  "Registered PayloadStateFeature" (`:1070`). The joint and Cartesian limit
  fallback warnings are at `joint_limits.cc:183` and `cartesian_limits.cc:134`
  (`phase4_container_restarted.log`).

---

## 4. Wrench reporting, frames and payload

### 4.1 Computation

- In `ForceTorqueSensorFeature` (`ICON/control/parts/feature_interfaces/force_torque_sensor.cc`):
  - a non-Ok status code is an error (`:576-597`);
  - joint positions give the gravity direction (`:599-642`);
  - \(W_{support}\) = `ComputeWrenchFtSupportMass(mounted + grasped)` (`:644-651`, `:553-556`);
  - post-sensor dynamics only if `estimate_post_sensor_dynamic_load` (`:653-667`);
  - **`wrench_at_ft = −raw + W_support(now) − bias`** (`:729-738`);
  - `wrench_at_target = TransformWrench(target_t_ft, wrench_at_ft)` (`:740-745`);
  - Tare stores `bias = W_support(at tare)` and raises `retare` (`:762-792`).
- Gravity is (0, 0, −9.81) in the world frame, rotated into the F/T frame
  (`ICON/control/parts/hal/force_torque_sensor_part/force_sensor_controller_migration_utils.cc:45-70`,
  `ICON/utils/constants.h:30-31`).
- `WrenchAtTip()` returns `wrench_at_target` (`force_torque_sensor.h:99-100`)
  and becomes `part_status.wrench_at_tip` (`realtime_part_status_helpers.cc:163`).
  Our `GetWrench`/`GetState` read `part_status[ft_sensor].wrench_at_ft`
  (`robot_service.py:611`, `:673-675`).

### 4.2 Frames

- The F/T part uses `target_link_name "panda_link8"` and `ft_sensor_link_name
  "panda_hand"` (`icon.txtpb:56-76`). The log shows "Force torque transform
  target_t_ft: 0 0 0 0i + 0j + 0k + 1" (identity, `phase4_container_restarted.log`).
- The Intrinsic `panda_hand` entity is at the flange. The flange has identity
  relative to `panda_link8` (`franka_scene_object.py:353-360`, `:382-387`).
  **Derived:** `wrench_at_ft` and `wrench_at_tip` share the flange/`panda_link8`
  axes.
- `ros2_hwm` rotates Isaac's `panda_hand` axes into the flange frame by Rz(−π/4)
  and keeps the sign (`project/ext/ros2_hwm/ros2_hwm_logic.h:42-44`,
  `ros2_hwm_logic.cc:12-21`). Isaac reports the load on the sensor
  (`project/sim/run_scene.py:166-171`; `phase_2.md:76-81`).
- The admittance controller treats `WrenchAtTip` as acting at the chain tip
  (`cartesian_admittance_controller.cc:191-366`). The TCP is 0.1 m along
  flange z, rotated by Rz(135°): the URDF joint `right_gripper` has
  `xyz="0 0 0.1" rpy="0 0 2.356"` with parent `panda_link8`
  (`project/assets/robots/franka/lula_franka_gen.urdf:321-326`). It is used as
  the `tcp` entity under `panda_hand` (`franka_scene_object.py:38`, `:401-406`).
  **Derived:** to control the TCP, `robot_tip_t_robot_tool` must be that pose.
- **Derived, sign:** pressing down at home loads the sensor along −z of the
  hand, so `wrench_at_ft` has +z, along the push. This matches MakeContact,
  whose goal wrench is `+dir·F`. Not verified by experiment.
- `GetWrench` reports the flange frame, not the base frame. **Derived:** G8's
  "force between 12 and 18 N" must be measured along the direction, or as the
  norm.

### 4.3 Payload

- The default `mounted_payload` comes from `support_mass` and `ft_t_cog`; ours
  are 0 and [0,0,0] (`icon.txtpb:56-76`). `grasped_payload` is empty
  (`hal_force_torque_sensor_part.cc:295-313`). The property names are
  `mounted_payload` and `grasped_payload` (`hal_force_torque_sensor_part.h:74-75`).
  They are read in ApplyCommand (`.cc:425-447`). `FromRobotPayload` uses only
  the mass and the `tip_t_cog` translation (`.cc:62-68`).
- ICON `SetPayload` (`ICON/server/service.cc:1304-1379`):
  - fails with open sessions;
  - writes the property named by `payload_name` on `part_name`;
  - requires enabled and not faulted;
  - disables and re-enables.
- Our `SetPayload` writes only arm/`full_payload` with zero inertia
  (`icon.py:103-114`). It rejects calls while a motion runs
  (`robot_service.py:809-818`). Recovery re-applies payload and speed but not
  the tare (`robot_service.py:447-455`).
- **Derived:** since bias = \(W_{support}\)(at tare), the payload term only
  corrects *orientation changes after the tare*. Device taring already zeroes
  the static load (`ros2_hwm_logic.cc:80-100`). CUJ 6 keeps the orientation
  between `TareWrench` and the end of `MoveCompliant`, so the held peg is
  compensated by the tare alone.

---

## 5. Example usages and typical values

### 5.1 `move_to_contact` skill (`FORCE/move_to_contact.py`)

- Constants (`:48-50`): `STOP_FORCE_RATIO` 0.5, `MIN_APPROACH_TIME_S` 0.05,
  `SETTLING_TIMEOUT_S` 2.0.
- Checks: `0 < contact_force ≤ excessive_force_threshold` (`:225-237`).
  `min_contact_force = 2 ×` the projected deadband norm (`:307-321`).
  `switch = max(0.5·F, min)` (`:323-326`).
- Actions:
  - tare;
  - `build_task_params` with `base_t_task` = identity (`:332-336`);
  - MakeContact with the direction vector in the base frame (`:338-346`);
  - an approach action and a stabilize action
    (`disable_reference_lowpass_filter=True`);
  - stop (`:348-370`).
- Sequence (`:411-447`):
  1. tare;
  2. approach until `elapsed_time_seconds > 0.05` AND `sensed_force > switch`;
  3. stabilize until `sensed_force > switch` AND `is_settled`;
  4. stop.

  Stabilize also goes to stop after 2 s (`:459-467`). Success is when the
  stop action `is_settled` (`:469-480`). Afterwards it reads `wrench_at_tip`
  (`:533-545`).
- Manifest defaults (`FORCE/move_to_contact_manifest.textproto:48-63`): timeout
  60 s, translational and rotational stiffness 300, MEDIUM, `contact_force` 10,
  direction z 0.1. Error codes 10201, 10301, 10302, 10303.
- `max_approach_velocity`=14 is declared in `move_to_contact.proto:25-107`.
  `rg` finds no use in `.py`, `.cc` or `.proto` outside the declaration.

### 5.2 Tutorial (`move_to_contact_tutorial.md`)

- UR F/T config example (`:30-71`): excessive 85 N / 8.5 Nm, virtual inertia
  15 kg / 1.0, deadband 1.5 N / 0.2 Nm.
- Forces used: 15, 8, 15 and 5 N (`:14-18`).
- Content: MakeContact speed \(v = D^{-1}F\) (`:232-234`), the rule for
  chaining lowpass filters (`:235`), patterns A and B (`:300-361`), and a
  state machine (`:480-488`).
- The tutorial says the config is in `hardware_config.parts_by_name`. Our
  config puts parts under `realtime_control_config` (`icon.txtpb`).
- No other code in `ic` uses these actions: `rg -l` finds only the files cited
  here and `ICON/python/create_action_utils.py`. There are no tests
  (`rg --files | rg -i '(admittance|force_primitive|action_builder|move_to_contact).*test'`
  → nothing).

### 5.3 CUJ 6 numbers (`robot_platform.md:231-241`, G8 `:113-114`)

The sequence is:
1. `MoveToPose` 2 cm above the hole;
2. `TareWrench`;
3. `MoveToContact(down, 15 N, 10 mm/s)`;
4. `MoveCompliant(3 cm lower, 200 N/m, 20 N)`.

Pass criteria: force 12–18 N, peg ≥ 25 mm inside, no fault. The listed risk is
"2 mm clearance at 1 kHz PhysX" (`:598`).

---

## 6. Real-time assumptions our non-real-time 250 Hz lockstep may violate

1.  **Substeps reuse one sample.** The admittance action runs
    ceil(1000/f) substeps; at 250 Hz that is 4, for an effective 1 kHz
    (`cartesian_admittance_action.cc:155-167`). The extra substeps use the same
    sensor data (`:674-727`). The wrench is refreshed at 250 Hz, while PhysX
    steps at 1 kHz (`project/config/timing.yaml`: `control_hz: 250`,
    `substeps: 4`).
2.  **The controller integrates the commanded state.** The initial target is the
    previous command (`:213-333`), and `ComputeControl` integrates the nominal
    state (`cartesian_admittance_controller.cc:369-452`). Overdamping assumes a
    stiff, position-controlled robot (infinite joint stiffness in the
    series-stiffness term, `overdamping.cc:110-129`). Isaac's joint drives are
    finite PD drives. Their deflection under contact load adds compliance that
    the controller cannot see. The size of that compliance was not measured;
    the only joint-error number on record is the 0.00090 rad after a scene
    reset (`phase_2_notes.md:245`).
3.  **IK is not called in the loop.** `ComputeControl` ignores its `/*ik*/`
    argument (`cartesian_admittance_controller.cc:369-371`), so the non-real-time
    `kinematic_chain` solver is needed only to register MANIPULATOR_KINEMATICS (§3).
4.  **Malloc on target updates.** `ToRealTimeCartesianTarget` carries
    "TODO(b/394004292): Avoid malloc" inside `INTRINSIC_SUPPRESS_REALTIME_CHECK`
    (`cartesian_impedance_commons.cc:293-298`). It runs when a target or
    streaming input is set. It is harmless in `--non_realtime` (`ICON/server/icon_main.cc:83`, `:157-159`).
5.  **Cycle timeout.** A force action active while Isaac stalls hits the 20 s
    BeginCycle timeout (`phase_3_inventory.md` §0 item 3).
6.  **Constant readings.** A steady contact can produce bit-identical wrenches.
    The constant-readings fault (`force_torque_sensor.cc:700-727`) is already
    disabled with `num_acceptable_constant_readings 2147483647` (`icon.txtpb:73`).
7.  **Short time windows in cycles.** `MIN_APPROACH_TIME_S` 0.05 s is about 12
    cycles at 250 Hz. The 1.0 s displacement window is 250 cycles
    (`cartesian_impedance_parameters.h:35-36`).
8.  **Stops are only real time inside ICON.** `run_action` ends on a
    non-real-time Python flag, and ends the session (§1.6). Conditions such as
    "force > limit → stop" are evaluated per cycle only when they are reactions
    or transitions inside ICON (`add_transition`, `add_action_sequence`). **Derived:**
    a Python-side stop lets MakeContact keep pushing for the gRPC latency.
9.  **Freshness.** `ros2_hwm` pairs the wrench with the joint states by stamp
    (`ros2_hwm.cc:401-406`; `stamp_match_tolerance_s: 1.0e-6`, `timing.yaml`).
    The wrench belongs to the same physics step as the joints. **Derived:**
    one cycle (4 ms) passes from sample to applied command.
10. **Velocity and position box faults.** A large lowpass step or a fast
    MakeContact can exceed the 1.7 m/s limit
    (`cartesian_admittance_controller.cc:350-363`). This is not likely at CUJ 6
    values.

---

## 7. Not verified / unknowns

1.  No force action was run. Nothing in §1–§6 is backed by an ICON or Isaac run
    of `force_primitive` or `cartesian_admittance_action`. That includes
    stability of the loop with PhysX contact, and the "Admittance Controller
    running with…" log line.
2.  MANIPULATOR_KINEMATICS registration is inferred from the IK-solver log line,
    not from a log of `RegisterManipulatorInterface` or `GetActionSignatures`.
3.  The sign of `wrench_at_ft` under contact (§4.2) is derived. The checks so
    far report only the norm (`phase_3_notes.md:24`). `phase_4_notes.md:474`
    prints `[0.006, 0.0, 5.753]`; I did not confirm whether that is the raw
    topic or ICON's value.
4.  The effective PhysX contact stiffness of peg and block, compared with the
    assumed `k_env` (5000/60000/100000), is unknown. So is the real approach
    speed and overshoot it gives.
5.  Whether the arm chain tip is `panda_link8`, or whether flange counts as the
    tip, is not checked. Either way the offset is identity (§4.2).
6.  Whether ICON's "robot base" (chain base) equals our API's base frame
    (`franka` object, root `panda_link0`, `franka_scene_object.py:33`) is
    assumed, not checked.
7.  What the arm does when a session ends while admittance is active: does it
    hold the last nominal command, and with what residual force? Not checked.
    `rg` in `hal_arm_part.cc` finds no "hold" path.
8.  Whether the ICON `SetPayload` disable/enable (§4.3) clears the F/T part's
    bias or `ros2_hwm`'s device tare is not checked.
9.  Whether the vendored client plus newly generated pb2 modules import cleanly
    in the container image was not checked: X1 used the host venv in `/tmp`.
    The `gen/` tree was not regenerated.
10. The behaviour of `is_settled` (commanded velocities, §1.6) after a stop
    action under PhysX contact jitter is unknown.
11. The 2 mm-clearance insertion in Isaac (contact offsets, substeps, friction)
    is untested. That is the design's own listed risk (`robot_platform.md:598`).
