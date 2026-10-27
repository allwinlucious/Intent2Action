# Phase 1 inventory: a Franka robot in the world service, planner and ICON

Factual inventory for [robot_platform.md](../robot_platform.md) (sections
"Franka model", "Timing" and "RobotService API") and
[phase_0_notes.md](phase_0_notes.md). It covers what it takes to turn Isaac
Sim's Franka URDF into an Intrinsic robot SceneObject, load it into the
running world service, drive it with ICON on the loopback hardware module, and
check forward kinematics against Isaac.

**Citation conventions.** Paths without a prefix are relative to
`/home/mew/intrinsic-core`. `project/` means `/home/mew/untitled-project/`.
`$ISAAC/` means `/mnt/c/isaac-sim/exts/` (Isaac Sim 5.1). `$MG/` means
`$ISAAC/isaacsim.robot_motion.motion_generation/`. `FRANKA_URDF` means
`$MG/motion_policy_configs/franka/lula_franka_gen.urdf`. `$FD/` means
`$ISAAC/isaacsim.asset.importer.urdf/data/urdf/robots/franka_description/`.
"Observed" means the output of a command run while writing this document
(nothing was built; no file other than this one was written).

Running services (container on the host network): world `localhost:18000`,
geometry `18001`, planner `18002`, ICON `18003`, localcas `18020`.

---

## 1. Robot SceneObject from SDF

### 1.1 The Bazel rule `sdf_scene_object`

Defined in `intrinsic/scene/build_defs/sdf_scene_object.bzl`.

- Macro arguments: `name`, `src` (the SDF), `sdf_assets` (meshes and other
  files the SDF references), `updates_pbtxts` (list of `SceneObjectUpdates`
  text protos), `deps` (proto deps needed to parse `intrinsic:user_data`),
  `large_mesh_checks_max_mesh_diagonal`, `skip_validate_referenced_geos`
  (`sdf_scene_object.bzl:133-168`).
- Step 1 runs `//intrinsic_sdk/intrinsic/scene/sdf:sdf_to_scene_object`
  (`sdf_scene_object.bzl:118-123`) with `--scene_object_name=<name>`,
  `--input_sdf_file`, `--input_file_descriptor_sets`,
  `--output_scene_object_gzf_file`, `--output_scene_object_pbtxt_file`
  (`sdf_scene_object.bzl:45-70`).
- Step 2, only if `updates_pbtxts` is non-empty, runs
  `//intrinsic_sdk/intrinsic/scene/tools:update_scene_object` with
  `--input_scene_object_gzf_file`, `--input_updates_proto_filenames`
  (comma-separated) and the same output flags (`sdf_scene_object.bzl:72-87`).
- Outputs: `<name>.gzf` and `<name>.pbtxt`; when updates are applied the
  converter's outputs are named `<name>__raw.gzf` / `<name>__raw.pbtxt`
  (`sdf_scene_object.bzl:21-38`). The rule also returns a `SceneObjectInfo`
  provider (`sdf_scene_object.bzl:89-99`).
- A separate asset rule exists in
  `intrinsic/assets/scene_objects/build_defs/scene_object.bzl` (not needed to
  produce the gzf; not read in detail).

Converter binary (`intrinsic_sdk/intrinsic/scene/sdf/sdf_to_scene_object.cc`):

- Flags at `:57-74`; default outputs are `/tmp/scene_object.pbtxt` and
  `/tmp/scene_object.gzf`.
- Validates the result with `ValidateSceneObject`, `ValidateReferencedGeos`
  and parses `user_data` (`:81-98`).
- Runs a large-mesh check, default maximum mesh diagonal 100 (`:223-230`).
- Writes the text proto and calls `AddSceneObjectToGzf` (`:235-245`).

Updater binary (`intrinsic_sdk/intrinsic/scene/tools/update_scene_object.cc`):
flags `--input_scene_object_gzf_file` / pbtxt, `--input_updates_proto_filenames`,
output gzf / pbtxt, `--file_descriptor_sets` (`:47-62`).

### 1.2 UR definitions: the pattern to copy

`intrinsic_control/intrinsic/models/robot_definitions/ur/BUILD`:

- Loads `xacro_file` from `//bazel:xacro.bzl` and `sdf_scene_object`
  (`:15-16`).
- UR5e (`:64-98`):

```starlark
filegroup(name = "ur5e", ...)                                  # :64-71
xacro_file(
    name = "ur5e_xacro",
    src = "ur5e/ur5e_world.sdf.xacro",
    out = "ur5e_world.sdf",
    data = [":ur5e"],
)                                                              # :73-79
sdf_scene_object(
    name = "ur5e_scene_object",
    src = "ur5e_world.sdf",
    sdf_assets = ["//intrinsic/robot_definitions/ur:ur5e"],
    updates_pbtxts = [
        "ur.named_configurations.pbtxt",
        "ur5e/ur5e_limits.updates.pbtxt",
    ],
    deps = ["@intrinsic_apis//intrinsic/icon/hal/proto/v1:digital_input_output_proto"],
)                                                              # :81-98
```

- UR3e and UR10e follow the same shape (`:35-62`, `:109-133`).

Directory layout (`ur/ur5e/`): `ur5e.xacro`, `ur5e_base.xacro`,
`ur5e_limits.updates.pbtxt`, `ur5e_world.sdf.xacro`; shared files
`ur/simulation_plugins.xacro` and `ur/ur.named_configurations.pbtxt`.

- `ur5e_world.sdf.xacro:18-27`: `<world name="ur5e">` containing exactly one
  model, `<xacro:ur5e_sdf name="arm">`.
- `ur5e.xacro:16-42`: a macro wrapping the base model and the simulation
  plugins.
- `ur/simulation_plugins.xacro:18-42`: a DigitalInputOutput plugin (the reason
  for the `deps` entry above; a Franka without I/O does not need it).
- `ur5e_base.xacro`:
  - `:18` declares `xmlns:intrinsic='https://intrinsic.ai/'`.
  - `:24` `<intrinsic:ik_solver>ur</intrinsic:ik_solver>`.
  - `:25-28` `<intrinsic:cartesian_limits>` with
    `min_translational_position -5 -5 -5` / `max_translational_position 5 5 5`.
  - Links reference collision meshes
    `model://third_party/universal_robots/ur5e/collision/*.wrapped.1cm.stl`
    and visual meshes `.../visual/*.dae` (e.g. `:43-60`).
  - Joints use `<limit><lower><upper><effort><velocity>` (`:75-92`).
  - `:432` `<frame name='flange' attached_to='wrist_3_link'
    intrinsic:create_attachment_entity='true'/>`.
  - `:433` `<xacro:insert_block name='model_options'/>`.
- Mesh filegroups: `intrinsic/robot_definitions/ur/BUILD:17-30`
  (ur5e = `glob(["ur5e/**"]) + ["//third_party/universal_robots:ur5e"]`);
  `third_party/universal_robots/BUILD:22-35`. The third_party directory holds
  collision `*.stl` / `*.wrapped.1cm.stl` and visual `*.dae`, plus
  `ur5e/ur5e.urdf` and `model.config` (the URDF is not used by the BUILD; the
  SDF is hand-written xacro).

Other robot definitions: KUKA kr6/kr10/kr16/kr20/kr50, ABB irb1300, FANUC
(crx_*, lr_mate, m_*, r_2000ic). **There is no Franka, iiwa or other 7-DoF
robot definition in the tree.** `ik_solver` values in use: `ur` (UR) and
`spherical_wrist` (the rest); FANUC CRX models use `kinematic_chain`
(`third_party/fanuc/crx_*/…_base.xacro:4`).

### 1.3 SDF requirements and Intrinsic tags

Structure (`intrinsic_sdk/intrinsic/scene/sdf/scene_object_from_sdf.cc`):

- A file holds at most one `<world>`; a world holds exactly one `<model>`; a
  model has no child models and at least one link (`:96-145`).
- `<frame>` elements become entities (`:150-160`) **only** if they carry
  `intrinsic:create_entity='true'` or `intrinsic:create_attachment_entity='true'`;
  the latter also sets `Frame.is_attachment_frame`
  (`intrinsic_sdk/intrinsic/scene/sdf/entity_from_sdf.cc:74-110`).
- Fixed joints are bypassed (links merged) only if the model has **no**
  non-fixed joints (`:174-243`). In a robot, every fixed-joint child link stays
  a separate Link entity.
- Exactly one root entity is required (`:256-270`).
- `cartesian_limits` parsed at `:272-279`; `ik_solver` at `:281-289`; more than
  one `ik_solver` is an error (`:398-432`); simulation spec at `:291-300`.
- `model://` and `file://` URIs resolve relative to the SDF's directory
  (`:350-354`); the file is loaded as an `sdf::Root` (`:366-370`).
- Meshes load through assimp (`intrinsic_sdk/intrinsic/scene/sdf/sdf_util.cc:338-352`,
  `:401-418`; `intrinsic_sdk/intrinsic/geometry/internal/mesh/io/load_mesh_from_file.cc:22`),
  so `.dae` and `.stl` are both accepted (UR uses exactly these).

Tag constants (`intrinsic_sdk/intrinsic/scene/sdf/custom_tags.h`):

| Tag / attribute | Where | Line |
|---|---|---|
| `intrinsic:cartesian_limits` | model | 24 (children `min/max_translational_position/velocity/acceleration/jerk`, `max_rotational_velocity/acceleration/jerk`: 27-48) |
| `intrinsic:control_frequency_hz` | model | 51-52 |
| `intrinsic:ik_solver` | model | 56-57; attribute `tip_link_name` 64 |
| `intrinsic:user_data` | model | 60 |
| `intrinsic:acceleration`, `intrinsic:jerk` | joint `<limit>` | 67-69 (parsed in `intrinsic_sdk/intrinsic/world/conversion/sdf/kinematics_component_from_sdf.cc:184-205`) |
| `intrinsic:create_entity` | `<frame>` | 74 |
| `intrinsic:create_attachment_entity` | `<frame>` | 81 |

There is no explicit "kinematic chain" tag: the chain is the tree of links and
joints from the single root. What matters is how many leaves it has (1.7, 2.4).

URDF input: sdformat is a Bazel dependency at version 16.1.0
(`MODULE.bazel:552`) and `sdf::Root` can load URDF, but no Intrinsic code or
BUILD file feeds a URDF to the converter (the only "URDF" mention under
`intrinsic_sdk/intrinsic/scene` is a comment at `sdf_util.cc:350`), and the
sdformat source was not available locally to check its URDF behaviour (fixed
joint lumping, links without `<inertial>`). FRANKA_URDF has no `<inertial>` on
any arm link (only `camera_link` has one, `FRANKA_URDF:37-42`). Writing the
SDF by hand, as UR does, avoids that unknown.

### 1.4 `updates.pbtxt` format

Header (`ur/ur5e/ur5e_limits.updates.pbtxt:1-2`):

```
# proto-file: intrinsic/scene/proto/v1/scene_object_updates.proto
# proto-message: intrinsic_proto.scene_object.v1.SceneObjectUpdates
```

Joint limits (`ur5e_limits.updates.pbtxt:19-183`):

```textproto
updates: {
  update_joints {
    joint_positions { key: "shoulder_pan_joint" value: 0 } ...
    joint_system_limits: {
      key: "shoulder_pan_joint"
      value: { min_position: … max_position: … max_velocity: 1.0
               max_acceleration: 15 max_jerk: 9000 max_effort: 100 }
    } ...
    joint_application_limits: { … }   # UR uses 95 % of system limits
  }
}
```

Named configurations (`ur/ur.named_configurations.pbtxt:19-49`):

```textproto
updates {
  set_named_configurations {
    named_configurations_to_set {
      name: "home"
      joint_positions { key: "shoulder_pan_joint" value: -1.5707 } ...
    }
  }
}
```

Proto definitions:

- `SceneObjectUpdates` = `repeated SceneObjectUpdate updates`
  (`intrinsic_apis/intrinsic/scene/proto/v1/scene_object_updates.proto:76-79`).
- `SceneObjectUpdate` oneof (`:31-51`): `entity_pose`, `create_frame`,
  `delete_entity`, `set_named_configurations`, `update_joints`,
  `cartesian_limits`, `rename_entity`, `update_collision_rules`,
  `update_user_data`, `set_ik_solvers`, `reparent_entity`,
  `update_frame_properties`, `update_geometry`, `update_physics_properties`,
  `create_joint`.
- `CreateFrameUpdate {parent_name, new_frame_name, parent_t_new_frame,
  designate_as_attachment_frame}` (`:96-110`); `DeleteEntityUpdate` with a
  `ChildPolicy` (`:112-133`); `SetNamedConfigurationsUpdate` (`:136-146`);
  `SetIKSolversUpdate` (`:148-151`); `ReparentEntityUpdate` (`:318-324`).
- `UpdateJointsRequest` (`:154-177`): `joint_positions`,
  `joint_application_limits`, `joint_system_limits` (maps of
  `JointLimitUpdate`), `parent_t_inboard`. The comment says "degrees" but the
  code stores radians (next bullet).
- `JointLimitUpdate` (`intrinsic_apis/intrinsic/kinematics/types/joint_limits.proto:77-100`):
  optional `min_position`, `max_position`, `max_velocity`, `max_acceleration`,
  `max_jerk`, `max_effort`.

Validation (`intrinsic_sdk/intrinsic/scene/util/scene_object_updates_internal.cc`):

- A named configuration must list **exactly** the movable (non-fixed) joints
  (`:554-566`, `:613-633`).
- `ApplyJointLimits` writes `fixed_limits.lower/upper`, velocity,
  acceleration, jerk and effort; all must be non-negative where applicable
  (`:649-690`).
- `update_joints.joint_positions` must cover all movable joints; values are
  written to `raw_value` and applied via `CreateAngleAxisPose`, i.e. radians
  (`:694-808`).

The same acceleration/jerk values can instead go directly into the SDF as
`<limit><intrinsic:acceleration>…</intrinsic:acceleration><intrinsic:jerk>…</intrinsic:jerk></limit>`
(1.3).

### 1.5 Output artifact and mesh storage

SceneObject proto (`intrinsic_apis/intrinsic/scene/proto/v1/scene_object.proto:30-74`):
`name=2`, `properties=3` (ObjectProperties), `entities=4`,
`simulation_spec=5`, `collision_rules=6`, `user_data=7`; validity rules at
`:30-38`.

- `ObjectProperties.kinematics` holds `limits` (CartesianLimits),
  `ik_solvers` and `named_configurations`
  (`intrinsic_apis/intrinsic/scene/proto/v1/object_properties.proto:22-89`).
  The IkSolver comment says that with no solver, `kinematic_chain` is used as a
  non-real-time fallback; `tip_link_name` is optional (`:64-74`).
  `NamedConfiguration = {name, map<string,double> joint_positions}`.
- Entities (`intrinsic_apis/intrinsic/scene/proto/v1/entity.proto`):
  `Entity{name=1, parent_name=2, parent_t_this=3, oneof frame=4 | link=5 |
  joint=6 | sensor=7}`; `Frame{is_attachment_frame=1}` (`:30-41`);
  `Link{geometry_component=1, physics_component=2}`;
  `Joint{kinematics_component=1}`.
- `KinematicsComponent` (`intrinsic_apis/intrinsic/world/proto/kinematics_component.proto:29-116`):
  `parent_t_inboard=10`, `outboard_t_child=13`, `motion_type`
  (REVOLUTE=1, PRISMATIC=2, FIXED=3), `axis=11` (default +z), `raw_value=3`,
  `Limits{velocity, acceleration, jerk, effort, fixed_limits{lower, upper}}`,
  `system_limits=5`, `application_limits=12`;
  `parent_t_this = parent_t_inboard · actuation · outboard_t_child`.
- `GeometryComponent.named_geometries`: `map<string, GeometrySet>`;
  `GeometrySet` has a deprecated `repeated Geometry geometries` (v0
  `GeometryStorageRefs`) and `map<string, v1.TransformedGeometry>
  named_geometries`
  (`intrinsic_apis/intrinsic/world/proto/geometry_component.proto:43-100`).
  The set names are `"Intrinsic_Collision"` and `"Intrinsic_Visual"`
  (`intrinsic_sdk/intrinsic/world/geometry_types.h:22,26`).
  v1 `TransformedGeometry{geometry=1, ref_t_shape=2}`; v1 `Geometry` oneof
  `{geo_ref=2, inline_geometry_data=3}`, `material_overrides=4`; v1
  `GeometryStorageRefs{exact_geometry_ref=1, renderable_ref=2, keep_renderable=3}`.

The `.gzf` is a zip (`intrinsic/assets/scene_objects/gzffile.go:35-113`):

| Entry | Content |
|---|---|
| `SOBJ/0` | the serialized SceneObject proto |
| `GEOM/<decimal>` | v1 exact geometry |
| `GLTF/<decimal>` | v1 renderable (glTF) |
| `EXPO/<ref>` | v0 geometry |

Inside the gzf the geometry refs point at zip entries; before the SceneObject
can be sent to the world service each file must be uploaded to CAS and the refs
rewritten. Only Go does this:
`intrinsic/assets/scene_objects/gzfprocessor.go:105-145, 273-364`,
`intrinsic/tools/inctl/util/cas_geometry_uploader.go:80-139`, and
`intrinsic/storage/content_addressable_storage/pkg/filetocas/filetocas.go:42-49, 138-185`
(refs use the `intcas://` prefix; `Upload` returns the object id).

### 1.6 Doing it without Bazel

- **No non-Bazel converter exists.** Nothing in Python or Go calls
  `SceneObjectFromSdf` (searched with `rg`), and the local `bazel-bin` has no
  built `sdf_to_scene_object`, `update_scene_object` or
  `scene_object_import_main`.
- A `SceneObjectImport` service also converts SDF (and USD)
  (`intrinsic/scene/service/scene_object_import.cc:155,170,976-1020`); it too
  needs a Bazel build.
- **Path A (no Bazel):** build the `SceneObject` proto directly in Python with
  the project's `gen/` stubs (`gen/intrinsic/scene/proto/v1/*`,
  `gen/intrinsic/world/proto/*` exist). Links carry v1 `named_geometries` whose
  `geo_ref` comes from `GeometryService.CreateGeometry` (2.2); joints carry
  `KinematicsComponent`; the flange is a Frame entity with
  `is_attachment_frame: true`; limits, named configs and ik solver go into
  `properties.kinematics`. This skips the SDF entirely.
- **Path B (Bazel):** copy the UR pattern of 1.2. A target for Franka would
  look like:

```starlark
xacro_file(
    name = "franka_xacro",
    src = "franka/franka_world.sdf.xacro",
    out = "franka_world.sdf",
    data = [":franka"],                 # filegroup with the meshes
)
sdf_scene_object(
    name = "franka_scene_object",
    src = "franka_world.sdf",
    sdf_assets = [":franka"],
    updates_pbtxts = [
        "franka/franka.named_configurations.pbtxt",
        "franka/franka_limits.updates.pbtxt",
    ],
)
```

  The resulting gzf still has to be unpacked (zip `SOBJ/0`, `GEOM/*`,
  `GLTF/*`), uploaded to CAS, and its refs rewritten (1.5) before
  `CreateObject`.

### 1.7 Consequences for the Franka model

These follow from the rules above and from FRANKA_URDF (section 5).

- **The robot must be a single non-branching chain** for Cartesian planning
  and IK:
  - the planner rejects Cartesian targets on branching structures (3.2);
  - `AddIkSolverKey` requires `tip_link_name` when there is more than one
    leaf (2.4);
  - ICON picks the IK solver for "the" chain (4.1).
- Leaves are counted over links/joints; coordinate and attachment frames are
  excluded (`intrinsic/world/world.cc:2761-2800`). So every extra fixed URDF
  link (`camera_*`, `panda_face_*`, fingertips, forearm/wrist points,
  `right_gripper`) must be dropped or written as an SDF
  `<frame … intrinsic:create_entity='true'>`. Kept as links, each becomes an
  extra leaf (1.3: fixed joints are not merged in a robot).
- The two finger prismatic joints would add two DoF (9 in total), which
  mismatches the 7-DoF loopback HWM, and every named configuration would have
  to list them (1.4). Options: drop the fingers, or put the finger geometry
  on `panda_hand` as fixed shapes.
- Name a frame `flange` and make it an attachment frame on `panda_link8`
  (flange frames are frames named "flange":
  `intrinsic/world/objects/kinematic_object_internal.cc:132-174`,
  `intrinsic_sdk/intrinsic/world/objects/object_world_ids.cc:37`).
- IK solver: only `kinematic_chain` fits 7 DoF (3.3). It is also the default
  when none is given (`intrinsic/world/objects/world_object_internal.cc:115`).

---

## 2. Creating the SceneObject in the running world

### 2.1 RPCs and request fields

The protos are `intrinsic/world/proto/object_world_service.proto` and
`intrinsic/world/proto/object_world_updates.proto` (not under `intrinsic_apis`);
`object_world_refs.proto` is under `intrinsic_apis/intrinsic/world/proto/`.
The project already has Python stubs for all three in `project/gen/intrinsic/world/proto/`.

RPC lines in `object_world_service.proto`: `CloneWorld` 88, `GetObject` 202,
`ListObjects` 213, `CreateObject` 224, `DeleteObject` 236, `UpdateObject` 254
(marked experimental), `UpdateObjectJoints` 288,
`UpdateKinematicObjectProperties` 301, `CreateFrame` 373, `GetTransform` 439,
`UpdateWorldResources` 468, `SyncObject` 553.

`CreateObjectRequest` (`object_world_updates.proto:199-248`):

| Field | # | Notes |
|---|---|---|
| `world_id` | 1 | `"world"` |
| `name` | 2 | must not be in use |
| `name_is_global_alias` | 9 | |
| `parent_object` | 3 | `ObjectReferenceWithEntityFilter` |
| `parent_object_t_created_object` | 4 | Pose |
| `action_for_object_entities` | 11 | |
| oneof `object_spec` | 5 / 10 / 12 | `create_single_entity_object`, `clone_from_existing_object`, `create_from_scene_object` |
| `user_data` | 13 | |
| `view` | 7 | |

`ObjectSpecFromSceneObject {scene_object=1, map<string,string> user_data=2}`
(`:182-193`). Server side:
`intrinsic/world/service/objects/object_world_updates_utils.cc:257-276`
(`HandleCreateObjectRequestFromSceneObject`), dispatch at `:376-448`.

`DeleteObjectRequest {world_id, object, force, reparent_child_objects}`
(`:251-278`).

### 2.2 SDK path and geometry upload

`intrinsic_sdk/intrinsic/world/python/object_world_client.py`:

- `create_object(*, object_name, geometry_spec: SceneObject | GeometryComponent,
  parent=None, parent_object_t_created_object=Pose3(), user_data=None)`
  (`:1328-1385`) sets `name_is_global_alias=True`, parent = root with
  `INCLUDE_FINAL_ENTITY`, and raises `CreateObjectError` on failure.
- `delete_object(world_object, force, reparent_child_objects)` (`:1192-1218`);
  `get_object` `:383`; `get_kinematic_object` `:404`; `list_objects` `:242`;
  `reset` → `CloneWorld` `:1489`.
- `register_geometry` / `register_geometry_v1(geometry=CreateGeometryRequest)`
  call `GeometryService.CreateGeometry` (`:1387-1439`).
- The module imports much of the SDK (`intrinsic.icon.equipment`,
  `robot_payload`, error handling, …; `:21-51`), so outside Bazel it is easier
  to call the stubs directly.

GeometryService:

- `CreateGeometry` RPC: `intrinsic/geometry/proto/geometry_service.proto:62-71`;
  request at `:134-137`.
- `GeometryData` oneof
  (`intrinsic_apis/intrinsic/geometry/proto/geometry_service_types.proto:95-126`):
  `geometry_v0`, `inline_geometry=9`, `primitive_set_v0`, `primitive_set=8`,
  `obj_data=2` (OBJ text), `gltf_bytes=3` (binary glTF), `stl_bytes=5`,
  `pts_bytes=6`. **There is no COLLADA (`.dae`) input**, so the Franka visual
  meshes need converting to glb/obj (or the STL is reused as the visual).
  `GeometryWithMetadata.geometry_storage_refs` (v1) is field 4 (`:31-58`).
- Implementation: `intrinsic/geometry/service/geometry_service_impl.cc:122-141`
  (`CreateGeometry`) → `SaveAndUploadGeometry` → `SaveGeometryV1`
  (`:310-316`), backed by the CAS library
  `intrinsic/geometry/storage/cas_storage.cc` (`SaveGeometryV1` `:264-279`;
  v1 `GetGeometry` needs `exact_geometry_ref`, `:186-230`).
- World and geometry share the same localcas (`project/container/entrypoint.sh:19,22-24`),
  so refs returned by `CreateGeometry` are readable by the world service.
- Stubs: `project/gen/intrinsic/geometry/proto/` has storage-ref and v1
  geometry modules but **no `geometry_service_pb2`**; it must be generated with
  `project/container/gen_protos.sh intrinsic/geometry/proto/geometry_service.proto`
  (the script generates the whole import closure: `gen_protos.sh:12-20`).

What the SceneObject holds: per link, a `GeometryComponent` whose
`named_geometries["Intrinsic_Collision"/"Intrinsic_Visual"].named_geometries[...]`
is a v1 `TransformedGeometry` with `geo_ref.exact_geometry_ref` (and optionally
`renderable_ref`) set to the CAS refs (1.5). The world service deserializes
geometry eagerly only for v0-only sets (`intrinsic/world/objects/world_object_internal.cc:377-386`;
override `--always_deserialize_geometries`, `:100`; deserialization at
`:1755-1771`).

### 2.3 Making loading idempotent

- `UpdateObjectRequest {world_id, object, update_from_scene_object=4, view}`
  (`object_world_updates.proto:293-306`) with
  `UpdateFromSceneObject {object_spec_from_scene_object, update_policy
  (UPDATE_IF_LEAF_OBJECT), allow_missing=3}` (`:308-323`).
- With `allow_missing: true` and a **by-name** object reference, a missing
  object is created under root with `name_is_global_alias`
  (`object_world_updates_utils.cc:291-329`). This is create-or-replace in one
  call.
- Alternative: `GetObject` / `ListObjects`, then `DeleteObject` and
  `CreateObject`. `CreateObject` alone fails on a duplicate name
  (`object_world_updates.proto:199-248`).
- `CloneWorld` (`reset`) restores a whole world (`object_world_client.py:1489`).

### 2.4 How the world turns the object into a robot

- An object is kinematic if it has any non-fixed joint
  (`intrinsic/world/objects/world_object_internal.cc:136-144`).
- Frame entities go to `kCoordinateFrames`, and also to `kAttachmentFrames`
  when `is_attachment_frame` (`:1736-1743`).
- Creating the `KinematicObject` sets cartesian limits and named configs
  (`:1950-1991`). The IK solver defaults to `kinematic_chain` (`:115`). A
  solver given explicitly must build; the default only logs a warning on
  failure (`:1993-2036`).
- `AddIkSolverKey`: with more than one final entity, `tip_link_name` must
  name one of them, else `FailedPrecondition`
  (`intrinsic/world/objects/kinematic_object_internal.cc:240-300`).
- Final entities = leaves of the robot parts, excluding attachment and
  coordinate frames (`intrinsic/world/world.cc:2761-2800`);
  `GetFinalEntityOfRobotKinematicChain` is invalid unless there is exactly one
  (`:2804-2818`); `IsChainKinematicChain` = exactly one leaf (`:2820-2824`).
- `KinematicObjectComponent` as returned by `GetObject`
  (`object_world_service.proto:647-700`): `joint_positions` (radians/metres),
  `joint_entity_ids=5`, `iso_flange_frames=3`,
  `named_joint_configurations=4`, `cartesian_limits=6`,
  `joint_system_limits=8`, `joint_application_limits=9` (`max_effort`
  unsupported), `IkSolver{kinematic_solver_key, base_entity_id, tip_entity_id}`.

---

## 3. Planner: ComputeIk, ComputeFk, PlanTrajectory

### 3.1 RPCs and messages

`intrinsic_motion_planning/intrinsic/motion_planning/proto/v1/motion_planner_service.proto`:
`PlanTrajectory` 352, `PlanPath` 370, `ComputeIk` 379, `ComputeFk` 389,
`CheckCollisions` 397. Stubs already exist in
`project/gen/intrinsic/motion_planning/proto/v1/`.

- `MotionPlanningRequest {world_id=1, robot_specification=2,
  motion_specification=3, motion_planner_config=4 (default timeout 90 s),
  compute_swept_volume=5, …}` (`:136-170`); the response carries
  `discretized` `JointTrajectoryPVA` (`:171-185`).
- `IkRequest {world_id=1, target=2 (GeometricConstraint), starting_joints=3
  (icon.JointVec), robot_reference=4, max_num_solutions=5,
  collision_settings=6, ensure_same_branch=7, prefer_same_branch=9,
  disable_error_on_collisions=10, collision_checker_config=11, …}`
  (`:198-253`); `IkResponse.solutions` (`:255-270`).
- `FkRequest {world_id=1, joints=2 (icon.JointVec), robot_reference=3,
  reference=4, target=5 (TransformNodeReference)}` (`:272-300`);
  `FkResponse.reference_t_target=3` (`:302-305`).
- `RobotReference` oneof `object_id` (`ObjectReference`);
  `RobotSpecification {robot_reference=1, start_configuration=2}`
  (`intrinsic_apis/intrinsic/motion_planning/proto/v1/robot_specification.proto:24-43`).
- References (`intrinsic_apis/intrinsic/world/proto/object_world_refs.proto`):
  `ObjectReferenceByName{object_name}` `:22`;
  `FrameReferenceByName{object_name, frame_name}` `:29`;
  `TransformNodeReferenceByName` oneof object/frame `:53`;
  `ObjectReference {id | by_name}` `:65`;
  `TransformNodeReference {id | by_name}` `:131`.
- `GeometricConstraint` oneof includes `joint_position=13` and
  `cartesian_pose=8` (`PoseEquality`)
  (`intrinsic_apis/intrinsic/motion_planning/proto/v1/geometric_constraints.proto:441-462`);
  `PoseEquality {moving_frame, target_frame, target_frame_offset}` (`:343-358`).
- `MotionSegment {target=5, motion_type=15 (ANY, LINEAR, JOINT),
  collision_settings=16, joint_limits=17}` (`motion_specification.proto:46`);
  `MotionSpecification {motion_segments=1, curve_parameters=3}` (`:92`).

### 3.2 What the robot must look like

- `ComputeFk` downloads the world, sets the joints if given
  (`robot->SetJointPositions`, joint order = `joint_entity_ids`), and returns
  `reference->GetTransform(target)`. It needs no IK solver and no chain check
  (`intrinsic_motion_planning/intrinsic/motion_planning/service/motion_planner_service.cc:930-969`).
- `ComputeIk` builds a validation proxy with `require_ik_and_fk=true`
  (`motion_planner_service.cc:894-900`), which fails for branching robots:
  "We currently do not support resolving Cartesian motion targets for
  branching kinematic structures" (`:397-417`; see also
  `path_planning/kinematics_system_proxy_util.cc:183-207`).
- Trajectory segments look up the single ISO flange frame
  (`trajectory_segment.cc:286`, `GetSingleIsoFlangeFrame`).

### 3.3 IK solver selection

- `SelectIkSolverKeyForChain` takes the object's first key, defaulting to
  `kinematic_chain` (`world_kinematics_system_proxy.cc:72-84`); the planner
  applies no real-time restriction (`:566-584`).
- Registered solvers (`intrinsic_kinematics/intrinsic/kinematics/ik/register_*.cc`):
  `ur`, `spherical_wrist` (6 DoF only:
  `spherical_wrist_inverse_kinematics.h:59`, check in `.cc:53-56`),
  `kinematic_chain` (non-RT, `register_chain_inverse_kinematics.cc:25-26`),
  `calibrated_ur*`, `rt_calibrated_ur*`.
- **Only `kinematic_chain` fits a 7-DoF Franka.** Its numerical method is set
  by the planner flag `--chain_ik_solver`, default NewtonRaphson
  (`chain_inverse_kinematics.cc:39`; listed in the phase 0 inventory).

### 3.4 Example requests (text format)

Object name `franka`, flange frame `flange`, world `world`.

```textproto
# FkRequest: flange pose in the robot base frame at the Lula default_q
world_id: "world"
robot_reference { object_id { by_name { object_name: "franka" } } }
joints { joints: [0.0, -1.3, 0.0, -2.87, 0.0, 2.0, 0.75] }
reference { by_name { object { object_name: "franka" } } }
target    { by_name { frame  { object_name: "franka" frame_name: "flange" } } }
```

```textproto
# IkRequest
world_id: "world"
robot_reference { object_id { by_name { object_name: "franka" } } }
starting_joints { joints: [0.0, -1.3, 0.0, -2.87, 0.0, 2.0, 0.75] }
max_num_solutions: 1
target {
  cartesian_pose {
    moving_frame { by_name { frame { object_name: "franka" frame_name: "flange" } } }
    target_frame { by_name { object { object_name: "franka" } } }
    target_frame_offset { position { x: 0.3 y: 0.0 z: 0.5 } orientation { x: 1 y: 0 z: 0 w: 0 } }
  }
}
```

```textproto
# MotionPlanningRequest (PlanTrajectory), joint-space goal
world_id: "world"
robot_specification {
  robot_reference { object_id { by_name { object_name: "franka" } } }
  start_configuration { joints: [0.0, -1.3, 0.0, -2.87, 0.0, 2.0, 0.75] }
}
motion_specification {
  motion_segments {
    target { joint_position { joints: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785] } }
  }
}
```

The `reference` / `target` / `moving_frame` field nesting follows
`object_world_refs.proto:29,53,131`. `joint_position` is an
`intrinsic_proto.icon.JointVec` (`geometric_constraints.proto:444`), and
`PoseEquality.target_frame_offset` is an `intrinsic_proto.Pose {position,
orientation}` (`geometric_constraints.proto:357`;
`intrinsic_apis/intrinsic/math/proto/pose.proto:25-28`).

---

## 4. ICON

### 4.1 IconMainConfig with a HalArmPart on loopback

Current project config (`project/container/config/icon.txtpb`):

```textproto
control_frequency_hz: 250
hardware_module_read_write_timeout_seconds: 10
hardware_module_names: "robot_hwm"
hardware_module_that_drives_clock: "robot_hwm"
```

HWM config (`project/container/config/loopback_hwm.txtpb`):
`name: "robot_hwm"`, `LoopbackConfig { num_dof: 7 }`,
`drives_realtime_clock: true`, `control_frequency_hz: 250`.

Arm config (from `project/docs/designs/phases/phase_0_inventory.md:540-581`,
plus the non-RT IK switch):

```textproto
control_frequency_hz: 250
hardware_module_read_write_timeout_seconds: 10
hardware_module_names: "robot_hwm"
hardware_module_that_drives_clock: "robot_hwm"
services {
  world_service_from_grpc { world_id: "world" }
  kinematics_from_world_service: true
  allow_non_real_time_inverse_kinematics: true   # only for Cartesian actions
}
realtime_control_config {
  parts_by_name {
    key: "arm"
    value {
      part_type_name: "HalArmPart"
      safety_action_type_name: "intrinsic.stop"
      hardware_resource_name: "franka"
      config {
        [type.googleapis.com/intrinsic_proto.icon.HalArmPartConfig] {
          joint_position_command   { module_name: "robot_hwm" interface_name: "joint_position_command" }
          joint_position_state     { module_name: "robot_hwm" interface_name: "joint_position_state" }
          joint_velocity_state     { module_name: "robot_hwm" interface_name: "joint_velocity_state" }
          joint_acceleration_state { module_name: "robot_hwm" interface_name: "joint_acceleration_state" }
        }
      }
    }
  }
}
intrinsic_runtime {
  name: "intrinsic_runtime"
  interfaces {
    key: "grpc://intrinsic_proto.world.ObjectWorldService"
    value { grpc { connection { address: "localhost:18000" } } }
  }
  interfaces {
    key: "grpc://intrinsic_proto.geometry.GeometryService"
    value { grpc { connection { address: "localhost:18001" } } }
  }
}
```

Field sources:

- `ServicesConfig`
  (`intrinsic_apis/intrinsic/icon/server/config/services_config.proto:31-66`):
  `WorldServiceFromGrpcServiceConfig{world_id}`, `world_service_from_grpc=2`,
  `kinematics_from_world_service=3`, `assembly_from_world_service=4`,
  `dynamics_from_kinematics_service_config=5`,
  `allow_non_real_time_inverse_kinematics=6`; consumed at
  `create_icon_main_loop.cc:436`. The world service is created only if the
  runtime interface exists or the fallback is on, and kinematics needs world
  (phase 0 inventory §3.4: `create_icon_main_loop.cc:371-373`, `:426-447`).
- `KinematicsFromWorldService`
  (`intrinsic_control/intrinsic/icon/control/services/kinematics_from_world_service.cc`):
  snapshots all kinematic objects once at `Initialize` (`:48-77`); per part it
  picks the first solvable chain (`:89-123`); a non-RT solver such as
  `kinematic_chain` is skipped unless `allow_non_real_time_inverse_kinematics`
  is set, and then `nullptr` is returned (`:132-172`).
- `HalArmPartConfig`
  (`intrinsic_apis/intrinsic/icon/control/parts/hal/arm_part/hal_arm_part_config.proto:30-125`):
  interfaces `joint_position_command=1`, `joint_commanded_position=25`,
  `joint_velocity_command=2`, `joint_torque_command=3`,
  `joint_acceleration_command=27`, `joint_position_state=4`,
  `joint_velocity_state=5`, `joint_acceleration_state=6`,
  `joint_torque_state=7`, `process_wrench_command=18`,
  `hand_guiding_command=19`, `control_mode_state=20`, `payload_command=21`,
  `payload_state=22`, `joint_system_limits=23`; other fields `homing=26`,
  `kinematics_model_name=10`, `dynamics_model_name=11`,
  `linear_joint_acceleration_filter_config=12`,
  `arm_position_pid_torque_controller_config=13`, `check_collisions=14`,
  `mode_of_safe_operation_limits_config=15`,
  `velocity_filter_cutoff_frequency=16`,
  `calculate_velocity_state_from_position=24`.
- `hal_arm_part.cc`:
  - Joint limits come from world `GetKinematicObject(hardware_resource_name)`
    when the name is set and a command interface exists; `NotFound` without a
    WorldService (`:366-451`).
  - Position command needs position state and `JointLimits` (phase 0
    inventory §3.5: `:685-694`).
  - `joint_acceleration_state` and the acceleration filter are mutually
    exclusive (`:453-458`).
  - Cartesian limits come from the world, else from safety config, else only
    a `LOG(ERROR)` (`:522-567`).
  - Kinematics come from `KinematicsService` via `hardware_resource_name` and
    are optional (the part builds without them); `kinematics_model_name`
    makes them required (`:569-633`).
  - No explicit check that the world robot's joint count equals the HWM's
    `num_dof` was found in `hal_arm_part.cc`. The Franka SceneObject must have
    exactly 7 movable joints (1.7).

### 4.2 Loopback interfaces

`intrinsic_control/intrinsic/icon/hardware_modules/loopback/loopback_hardware_module.cc`
(`num_dof` defaults to 6, `:102-160`):

| Interface name | Kind | Line |
|---|---|---|
| `joint_position_command` | strict | 122-123 |
| `joint_position_state` | mutable | 126-127 |
| `joint_velocity_state` | mutable | 130-131 |
| `joint_acceleration_state` | mutable | 134-135 |
| `payload_command` | | 138-139 |
| `payload_state` | mutable | 142-143 |
| `joint_system_limits` | strict, only if `advertise_system_limits_interface` | 148-149 |
| `safety_status` | mutable (AUTOMATIC) | 154-155 |

It runs its own clock thread when it drives the clock (`:163-175`).
`LoopbackConfig` (`intrinsic_apis/intrinsic/icon/hardware_modules/loopback/loopback_config.proto:21-34`):
`num_dof=1`, `sensed_joint_position_noise_stddev_rad=2`,
`advertise_system_limits_interface=4`.

### 4.3 Rebuilding the main loop

`intrinsic_control/intrinsic/icon/server/grpc_envelope.cc`:

- `ClearFaults` (`:238-257`) rebuilds **only if the envelope is latching a
  fatal error** (`IconService().status()` not OK, i.e. main-loop creation
  failed). Otherwise it forwards to the operational
  `IconApiService::ClearFaults` (`intrinsic_control/intrinsic/icon/server/service.cc:1160-1165`),
  which does not re-read the world.
- `RestartServer` (`:278-284`) always calls `RebuildIconImpl`.
- `Enable` while in fatal fault also rebuilds (`:450-454`).
- `RebuildIconImpl` re-runs the factory (`:682-~735`).

`intrinsic_apis/intrinsic/icon/proto/v1/service.proto`: `GetStatus` 503,
`ListParts` 539, `OpenSession` 600, `Enable` 679, `ClearFaults` 705 (returns
`RESOURCE_EXHAUSTED` while a fatal-fault clear is in progress; retry),
`GetOperationalStatus` 719, `RestartServer` 733 ("Requests restarting the
entire server. Clears faults and enables as a side effect."),
`SetSpeedOverride` 753, `SetPayload` 808.

Ordering consequence: the kinematics snapshot (4.1) and the arm's limits are
read when the main loop is built. Either start ICON after the robot exists in
the world, or call `RestartServer` (works in any state) or `ClearFaults`
(only if creation failed) after loading it.

### 4.4 Starting a point-to-point or trajectory action on part "arm"

Python client (`intrinsic_control/intrinsic/icon/python/`):

- `icon_api.Client.connect(grpc_host="localhost", grpc_port=8128,
  insecure=True, connect_timeout=20, rpc_timeout=None)` uses no ingress
  (`icon_api.py:168-197`); `start_session(parts, context)` returns
  `_session.Session(self._stub, parts, context)` (`:351-389`); `enable` `:391`;
  `clear_faults` `:443-464`; `list_parts` `:341`.
- `_session.Session(stub, parts, context=None)` opens the `OpenSession`
  bidi stream and allocates the parts via
  `initial_session_data.allocate_parts.part` (`_session.py:79-131`), then
  opens `WatchReactions` (`:137-138`). `add_action` `:415`, `start_action`
  `:686`, `start_action_and_wait(action, wait_for=None, timeout_s)`
  (default condition `is_done`) `:744-772`, `end` `:237`.
- `actions.Action(id, type_name, part, params)` (`actions.py:33`);
  `Condition.is_done` (`reactions.py:121`).
- Point-to-point
  (`intrinsic_control/intrinsic/icon/actions/point_to_point_move_utils.py:20-76`):
  `ACTION_TYPE_NAME = "intrinsic.point_to_point_move"`;
  `create_point_to_point_move_action(action_id, joint_position_part_name,
  goal_position, goal_velocity=None, joint_limits=None)`. Params
  `PointToPointMoveFixedParams {goal_position=1 (JointVec), goal_velocity=2
  (JointVec), optional joint_limits=3}`
  (`intrinsic_apis/intrinsic/icon/actions/point_to_point_move.proto:27`;
  package `intrinsic_proto.icon.actions.proto`).
- Trajectory
  (`intrinsic_control/intrinsic/icon/actions/trajectory_tracking_action_utils.py:17-63`):
  `"intrinsic.trajectory_tracking"`;
  `create_trajectory_tracking_action(action_id, part, trajectory:
  JointTrajectoryPVA)`; the trajectory's start must be within 0.1 rad and
  0.15 rad/s of the current state. Params `TrajectoryTrackingActionFixedParams
  {trajectory=1}` (`trajectory_tracking_action.proto:23`; package
  `intrinsic_proto.icon.actions`). A planner `discretized` trajectory (3.1) is
  this type.
- Joint point-to-point needs no IK; Cartesian actions need an IK solver in
  ICON (4.1).

Sketch, with ICON at `localhost:18003`:

```python
stub = service_pb2_grpc.IconApiStub(grpc.insecure_channel("localhost:18003"))
with _session.Session(stub, ["arm"]) as s:
    a = s.add_action(point_to_point_move_utils.create_point_to_point_move_action(
        1, "arm", goal_position=[0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]))
    s.start_action_and_wait(a, timeout_s=30)
```

### 4.5 Running the Python client outside Bazel

- There are no `__init__.py` files in `intrinsic_control/intrinsic`,
  `intrinsic_sdk/intrinsic` or `project/gen/`, so they all work as implicit
  namespace packages on `sys.path`.
- `icon_api.py` imports `from intrinsic.solutions import deployments`
  (`icon_api.py:41`), which pulls in the whole solutions SDK
  (`intrinsic_sdk/intrinsic/solutions/deployments.py:38-66`), plus
  `intrinsic.util.grpc.connection/interceptor` and `robot_payload` (numpy).
  **Using `icon_api.Client` outside Bazel is heavy.**
- `_session.py` imports (`:30-53`): `absl.logging`, `google.rpc.code_pb2`,
  `google.rpc.status_pb2`, `grpc`, `intrinsic.icon.proto.streaming_output_pb2`,
  `intrinsic.icon.proto.v1.{service_pb2, service_pb2_grpc, types_pb2}`,
  `intrinsic.icon.python.{actions, errors, reactions}`,
  `intrinsic.logging.proto.context_pb2`. `actions.py:26-30` and
  `reactions.py:31-33` import only protos and each other; `errors.py` has no
  intrinsic imports. **Constructing `_session.Session` directly with a stub is
  feasible outside Bazel.**
- Missing pieces, observed against `project/.venv` and `project/gen/`:
  - `gen/intrinsic/icon/actions/` does not exist. Generate it with
    `gen_protos.sh intrinsic/icon/actions/point_to_point_move.proto
    intrinsic/icon/actions/trajectory_tracking_action.proto`.
  - `gen/google/rpc/` has `status_pb2` but not `code_pb2` (generate
    `google/rpc/code.proto`).
  - `absl-py` and `numpy` are not installed in `.venv`.
  - Present: `icon/proto/{streaming_output, joint_space}_pb2`,
    `icon/proto/v1/{service, types, condition_types}_pb2`,
    `logging/proto/context_pb2`, `kinematics/types/joint_limits_pb2`.
- Raw gRPC alternative: drive `OpenSession` directly with
  `OpenSessionRequest` messages, in the same sequence `_session.py:107-131`
  and `:686-772` use.

---

## 5. The Franka URDF (FRANKA_URDF)

`<robot name="franka">`, generated by xacro from `lula_franka.urdf.xacro`
(`FRANKA_URDF:1-6`).

### 5.1 Joints

Movable joints (all have `<origin>`, `<axis xyz="0 0 1">` and a
`safety_controller` with `k_position=100`, `k_velocity=40` and soft limits
equal to the hard limits):

| Joint | Type | Parent → child | Origin xyz / rpy | lower | upper | effort | velocity | Line |
|---|---|---|---|---|---|---|---|---|
| `panda_joint1` | revolute | link0 → link1 | 0 0 0.333 / 0 0 0 | -2.8973 | 2.8973 | 87 | 2.1750 | 120-127 |
| `panda_joint2` | revolute | link1 → link2 | 0 0 0 / -π/2 0 0 | -1.7628 | 1.7628 | 87 | 2.1750 | 140-147 |
| `panda_joint3` | revolute | link2 → link3 | 0 -0.316 0 / π/2 0 0 | -2.8973 | 2.8973 | 87 | 2.1750 | 160-167 |
| `panda_joint4` | revolute | link3 → link4 | 0.0825 0 0 / π/2 0 0 | -3.0718 | -0.0698 | 87 | 2.1750 | 180-187 |
| `panda_joint5` | revolute | link4 → link5 | -0.0825 0.384 0 / -π/2 0 0 | -2.8973 | 2.8973 | 12 | 2.6100 | 200-207 |
| `panda_joint6` | revolute | link5 → link6 | 0 0 0 / π/2 0 0 | -0.0175 | 3.7525 | 12 | 2.6100 | 220-227 |
| `panda_joint7` | revolute | link6 → link7 | 0.088 0 0 / π/2 0 0 | -2.8973 | 2.8973 | 12 | 2.6100 | 240-247 |
| `panda_finger_joint1` | prismatic, axis 0 1 0 | hand → leftfinger | 0 0 0.0584 | 0.0 | 0.04 | 20 | 0.2 | 298-304 |
| `panda_finger_joint2` | prismatic, axis 0 -1 0, `<mimic joint="panda_finger_joint1"/>` | hand → rightfinger | 0 0 0.0584 | 0.0 | 0.04 | 20 | 0.2 | 305-312 |

Note `panda_joint4`'s range excludes 0 (`:186`), so the all-zeros pose is
outside the joint limits.

Fixed joints on the arm's main chain:

| Joint | Parent → child | Origin | Line |
|---|---|---|---|
| `base_link_robot` | base_link → panda_link0 | identity | 314-319 |
| `panda_joint8` | panda_link7 → panda_link8 | xyz 0 0 0.107 | 248-254 |
| `panda_hand_joint` | panda_link8 → panda_hand | rpy 0 0 -π/4 | 255-259 |

Links with geometry: `panda_link0` … `panda_link7`, `panda_hand`,
`panda_leftfinger`, `panda_rightfinger` (the right finger's visual and
collision have `rpy 0 0 π`, `:284-297`), and `camera_link` (box primitives,
`:22-43`). `base_link` (`:313`) and `panda_link8` (`:248`) are empty. Root
link: `base_link`, matching Lula's `root_link: base_link`
(`$MG/motion_policy_configs/franka/rmpflow/robot_descriptor.yaml:33`). No
arm link has an `<inertial>` block.

### 5.2 Extra fixed frames

All are empty links on fixed joints:

| Link | Parent | Origin xyz / rpy | Line |
|---|---|---|---|
| `camera_bottom_screw_frame` | panda_hand | 0.0358116 0 0.0360562 / 0 -π/2 π | 11-16 |
| `camera_link` (box geometry, mass 0.564) | camera_bottom_screw_frame | 0 0.02 0.0115 | 17-43 |
| `camera_depth_frame` | camera_link | identity | 45-50 |
| `camera_depth_optical_frame` | camera_depth_frame | rpy -π/2 0 -π/2 | 51-56 |
| `camera_left_ir_frame` / `_optical_frame` | camera_depth_frame | 0 0 0 | 58-69 |
| `camera_right_ir_frame` / `_optical_frame` | camera_depth_frame | 0 -0.055 0 | 71-82 |
| `camera_color_frame` / `_optical_frame` | camera_depth_frame | 0 0.015 0 | 84-95 |
| `right_gripper` | panda_link8 | 0 0 0.1 / 0 0 3π/4 | 320-326 |
| `panda_rightfingertip` | panda_rightfinger | 0 0 0.045 / 0 0 π | 327-333 |
| `panda_leftfingertip` | panda_leftfinger | 0 0 0.045 / 0 0 π | 334-340 |
| `panda_forearm_end_pt` | panda_link4 | -0.09 0.02 0 | 341-347 |
| `panda_forearm_mid_pt` | panda_forearm_end_pt | 0 0.175 0 | 348-354 |
| `panda_forearm_mid_pt_shifted` | panda_forearm_mid_pt | 0 0 -0.05 | 355-361 |
| `panda_forearm_distal` | panda_link5 | 0 0.1 0 | 362-368 |
| `panda_wrist_end_pt` | panda_link7 | 0 0 -0.07 | 369-375 |
| `panda_face_left` / `_right` | panda_hand | 0 ±0.07 0.04 | 376-389 |
| `panda_face_back_left` / `_right` | panda_hand | 0 ±0.07 0 | 390-403 |

A `left_gripper` frame is commented out (`:404-411`).

### 5.3 Meshes

- References use `package://franka_description/meshes/visual/<name>.dae` and
  `package://franka_description/meshes/collision/<name>.stl` for `link0`–`link7`,
  `hand` and `finger` (e.g. `FRANKA_URDF:96-107`, `:260-271`, `:272-283`).
- On disk, the package is `$FD` (it has a `package.xml`):
  - `$FD/meshes/visual/{link0..link7,hand,finger}.dae`: COLLADA 1.4.1,
    exported from Blender 2.79, `<unit name="meter" meter="1"/>`, `Z_UP`
    (`$FD/meshes/visual/link1.dae:1-11`), 51 KB–1.7 MB each (11 MB in total).
  - `$FD/meshes/collision/{link0..link7,hand,finger}.stl`: binary STL.
    Observed: `link1.stl` has 300 triangles and 15084 bytes = 84 + 50·300;
    `finger.stl` has 56 triangles. 2.9–15 KB each (132 KB in total).
- The same package also has `robots/panda_arm.urdf`,
  `robots/panda_arm_hand.urdf` and their xacros.
- Formats versus Intrinsic: the SDF converter accepts both through assimp
  (1.3). `GeometryService.CreateGeometry` accepts STL, OBJ and glTF but not
  DAE (2.2).

### 5.4 Flange and tool frame

- `panda_link8` is 0.107 m along `panda_link7`'s z (`:248-254`); this is the
  Franka flange.
- `panda_hand` is the same origin rotated −45° about z (`:255-259`).
- Isaac's RMPflow end effector is `right_gripper`
  (`$MG/motion_policy_configs/franka/rmpflow/config.json:2`), which is 0.1 m
  beyond `panda_link8`, rotated +135° about z (`:320-326`).
- The Isaac `Franka` manipulator class uses the USD prim `panda_rightfinger`
  as its default end effector
  (`$ISAAC/isaacsim.robot.manipulators.examples/isaacsim/robot/manipulators/examples/franka/franka.py:68-69,79-80`).

### 5.5 Limits and defaults in Isaac's configs

Lula robot description (`$MG/motion_policy_configs/franka/rmpflow/robot_descriptor.yaml`):

- `cspace: panda_joint1 … panda_joint7` (`:24-31`).
- `default_q: [0.00, -1.3, 0.00, -2.87, 0.00, 2.00, 0.75]` (`:35-41`).
- `acceleration_limits: [15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0]` (`:43`).
- `jerk_limits: [7500.0, 3750.0, 5000.0, 6250.0, 7500.0, 10000.0, 10000.0]` (`:44`).
- Finger joints are held fixed at 0.025 (`cspace_to_urdf_rules`, `:49-51`).
- Collision spheres from `:53` on.

Other copies and configs:

- An identical acceleration/jerk set is in
  `$MG/isaacsim/robot_motion/motion_generation/tests/test_assets/franka_conservative_spheres_robot_description.yaml:40-41`.
- FR3, a different robot, uses `acceleration_limits [20,20,20,20,20,10,10]`
  and `jerk_limits` all 10000
  (`$MG/motion_policy_configs/FR3/rmpflow/fr3_robot_description.yaml:36-37`).
- `franka_rmpflow_common.yaml` has only RMP gains (e.g.
  `max_acceleration_norm: 50.` `:85`), no joint limits.
- No cuMotion or XRDF Franka config was found under `$ISAAC` (searched for
  `jerk_limits|acceleration_limits` in yaml/xrdf/json).

Observed from Lula (6.2): velocity limits `[2.175 ×4, 2.61 ×3]` (from the
URDF); acceleration and jerk exactly as in the YAML above.

These are Isaac's limits, not certified Franka datasheet values; Franka's own
published limits were not checked.

---

## 6. FK in Isaac Sim 5.1

### 6.1 APIs

- `lula` is a native module: `$ISAAC/isaacsim.robot_motion.lula/pip_prebundle/lula.cp311-win_amd64.pyd`
  plus `lula_*.dll`, `nvidia_lula_no_cuda-0.10.1` (the extension adds
  `pip_prebundle` to the path: `$ISAAC/isaacsim.robot_motion.lula/config/extension.toml`,
  `[[python.module]] path = "pip_prebundle"`).
- Raw Lula (no Kit needed): `rd = lula.load_robot(robot_description_path,
  urdf_path)`; `k = rd.kinematics()`; `pose = k.pose(q_column_vector,
  frame_name)`; `pose.translation`, `pose.rotation.matrix()`;
  `k.frame_names()`, `k.num_c_space_coords()`, `k.c_space_coord_name(i)`,
  `k.c_space_coord_velocity_limit(i)`, `…_acceleration_limit(i)`,
  `…_jerk_limit(i)`. These are the calls `LulaKinematicsSolver` makes
  (`$MG/isaacsim/robot_motion/motion_generation/lula/kinematics.py:50-54,205-206,387-428`;
  `lula/interface_helper.py:97-101`).
- `LulaKinematicsSolver(robot_description_path, urdf_path,
  robot_description=None)` (`lula/kinematics.py:45-58`).
  `compute_forward_kinematics(frame_name, joint_positions,
  position_only=False) -> (position(3,), rotation(3,3))` (`:208-225`) returns
  `robot_rot @ pose.translation + robot_pos`, divided by stage units
  (`interface_helper.py:97-101`). `set_robot_base_pose(position,
  orientation)` (`kinematics.py:199-200`; `interface_helper.py:47-63`).
  `get_joint_names()` `:202`, `get_all_frame_names()` `:205`,
  `get_cspace_acceleration_limits()` / `get_cspace_jerk_limits()`
  `:394-428`. It imports `isaacsim.core.utils.stage.get_stage_units`
  (`kinematics.py:21,58`), so it needs a running Kit app (SimulationApp).
- Config loader:
  `interface_config_loader.load_supported_lula_kinematics_solver_config("Franka")`
  returns `{robot_description_path, urdf_path}`
  (`$MG/isaacsim/robot_motion/motion_generation/interface_config_loader.py:75-113`),
  resolved through `$MG/motion_policy_configs/policy_map.json:2-5` →
  `./franka/rmpflow/config.json` → `robot_descriptor.yaml` and
  `../lula_franka_gen.urdf` (`config.json:5-9`).
- USD articulation: `ArticulationKinematicsSolver(robot_articulation,
  kinematics_solver, end_effector_frame_name)`; `compute_end_effector_pose()`
  reads the articulation's joint positions and calls the Lula solver
  (`$MG/isaacsim/robot_motion/motion_generation/articulation_kinematics_solver.py:38-67`).
  The USD's own pose comes from
  `SingleXFormPrim(robot_prim_path + "/" + frame).get_world_pose()`. Isaac's
  test compares the two for every Lula frame that exists as a USD prim, on
  `/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd`
  (`$MG/isaacsim/robot_motion/motion_generation/tests/test_kinematics.py:128-202`).
  It notes "a known bug with the kinematics not matching on the Franka finger
  frames and whatever is in frame 0" and asserts < 0.005 only on the other
  frames (`:140-144`). The asset path also appears in `franka.py:66`.

### 6.2 Verified standalone run

Command (script on stdin; nothing written to disk). `cmd.exe` warns that the
UNC working directory is not supported and falls back to `C:\Windows`, which
is harmless:

```bash
cat script.py | cmd.exe /c "cd /d C:\isaac-sim && python.bat -"
```

```python
import sys
sys.path.insert(0, r"C:\isaac-sim\exts\isaacsim.robot_motion.lula\pip_prebundle")
import numpy as np, lula
d = r"C:\isaac-sim\exts\isaacsim.robot_motion.motion_generation\motion_policy_configs\franka"
rd = lula.load_robot(d + r"\rmpflow\robot_descriptor.yaml", d + r"\lula_franka_gen.urdf")
k = rd.kinematics()
p = k.pose(np.array(q, dtype=np.float64).reshape(-1, 1), "panda_link8")
p.translation, p.rotation.matrix()
```

Observed:

- It runs in about 1 s, with no SimulationApp.
- Lula 0.10.1 reports 7 c-space coordinates, `panda_joint1` … `panda_joint7`.
- It prints one warning: the mimic attribute of `panda_finger_joint2` is
  ignored because `panda_finger_joint1` is not a c-space coordinate.
- Frames: `base_link`, `panda_link0`–`panda_link8`, `panda_hand`, all camera
  frames, `panda_face_*`, both fingers and fingertips, `right_gripper`, and
  the forearm/wrist points (35 in total).

Reference FK values, base frame = `base_link`, metres. Rotation matrices are
row-major:

| q | frame | translation | rotation |
|---|---|---|---|
| all zeros (outside the joint4 limit) | `panda_link8` | [0.088, 0, 0.926] | [[1,0,0],[0,-1,0],[0,0,-1]] |
| all zeros | `panda_hand` | [0.088, 0, 0.926] | [[0.7071,0.7071,0],[0.7071,-0.7071,0],[0,0,-1]] |
| all zeros | `right_gripper` | [0.088, 0, 0.826] | [[-0.7071,-0.7071,0],[-0.7071,0.7071,0],[0,0,-1]] |
| `default_q` [0,-1.3,0,-2.87,0,2.0,0.75] | `panda_link8` | [0.226113, 0, 0.519254] | [[0.66508,-0.619586,0.416871],[-0.681639,-0.731689,0],[0.30502,-0.284155,-0.908966]] |
| `default_q` | `panda_hand` | [0.226113, 0, 0.519254] | [[0.908396,0.032169,0.416871],[0.035391,-0.999374,0],[0.41661,0.014753,-0.908966]] |
| `default_q` | `right_gripper` | [0.2678, 0, 0.428358] | [[-0.908396,-0.032169,0.416871],[-0.035391,0.999374,0],[-0.41661,-0.014753,-0.908966]] |
| "ready" [0,-0.785,0,-2.356,0,1.571,0.785] | `panda_link8` | [0.30702, 0, 0.59027] | [[0.707388,-0.706825,0],[-0.706825,-0.707388,0],[0,0,-1]] |
| "ready" | `panda_hand` | [0.30702, 0, 0.59027] | [[1,0.000398,0],[0.000398,-1,0],[0,0,-1]] |
| "ready" | `right_gripper` | [0.30702, 0, 0.49027] | [[-1,-0.000398,0],[-0.000398,1,0],[0,0,-1]] |

For comparison with Intrinsic's `ComputeFk` (3.4):

- With `reference` = the robot object and `target` = a `flange` frame placed
  with identity on `panda_link8`, the result should equal the `panda_link8`
  rows above, as long as the SceneObject's root is at `base_link`.
- Lula's `kinematics.pose` is relative to the URDF root with no base offset;
  `LulaKinematicsSolver` adds `set_robot_base_pose` and stage units on top
  (6.1).

---

I did not build anything with Bazel, check sdformat's URDF import behaviour,
or run a USD-articulation FK comparison in Kit. None of the example requests
were sent to the running services.
