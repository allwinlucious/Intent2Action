# Phase 4 inventory: camera-to-robot calibration via `CalibrationService`

This is a factual inventory for [robot_platform.md](../robot_platform.md)
("Calibration flow", CUJ 3, G6, Q3). It covers:

- which binary serves `CalibrationService`, and its flags and runtime config
- the platform services it needs at start-up and on each call
- what each RPC does
- how it gets images and robot poses
- the calibration targets it supports, eye-to-hand and eye-in-hand
- what `Save` writes and where
- the `pose_sampling` library
- the minimal stand-ins needed to run it without k3s

**Citation conventions.**

| Shorthand | Path |
|---|---|
| `ic/` | `/home/mew/intrinsic-core/` |
| `project/` | `/home/mew/untitled-project/` |
| `CAL/` | `ic/intrinsic_perception/intrinsic/perception/calibration/` |
| `SVC/` | `CAL/services/v1/` |
| `SK/` | `ic/intrinsic_perception/intrinsic/perception/skills/calibration/` |
| `PROTO` | `ic/intrinsic_apis/intrinsic/perception/proto/v1/calibration_service.proto` |
| `IMPL` | `SVC/calibration_service_impl.cc` |
| `MAIN` | `SVC/calibration_service_main.cc` |

**What was run.** Source reading only, plus these read-only commands:

- `ls ic/bazel-bin/intrinsic_perception/intrinsic/perception/calibration/services/v1/`
  gives "No such file or directory". The binary has never been built in
  this checkout.
- `docker images` lists `intrinsic-core:dev`, `hello-world` and the zenohd
  image `zenohd@sha256:9a8195c9…db01d`.
- `docker image save <zenohd image> | tar -t` (a listing of every layer):
  the zenohd image's root contains `zenohd`, `libzenoh_plugin_storage_manager.so`,
  `libzenoh_backend_fs.so`, `libzenoh_plugin_rest.so` and `dumb-init`. The
  image config entrypoint is `/dumb-init -- /zenohd -c /zenohd_config.json5`
  (`docker image inspect`).
- `docker run --rm --network none --entrypoint /bin/ls intrinsic-core:dev -la /opt/zenoh /opt/intrinsic/bin`:
  `/opt/zenoh` holds only `zenohd`. `/opt/intrinsic/bin` holds
  `generic_icon_machine_server`, `geometry_service`, `localcas`,
  `loopback_hardware_module_main`, `motion_planner_service_assetized_main`
  and `world_binary`. There is no calibration binary.

Isaac Sim was not started, Bazel was not run, no image was built and
nothing was committed. This file is the only repository file written.

---

## 1. Binary, target, flags, runtime config

### 1.1 Target

| Item | Value | Source |
|---|---|---|
| Library | `//intrinsic_perception/intrinsic/perception/calibration/services/v1:calibration_service_impl` | `SVC/BUILD:23-102` |
| Binary | `//intrinsic_perception/intrinsic/perception/calibration/services/v1:calibration_service_main` (`cc_binary`) | `SVC/BUILD:104-131` |
| OCI image | `:calibration-service-image` (from the binary) | `SVC/BUILD:133-137` |
| Service asset | `intrinsic_service(name = "calibration_service")`, default config `calibration_service_default_config.textproto` | `SVC/BUILD:139-149` |
| Manifest id | `ai.intrinsic` / `calibration_service`; the same image for `real_spec` and `sim_spec`; config message `intrinsic_proto.perception.v1.CalibrationServiceConfig`; routed prefix `/intrinsic_proto.perception.v1.CalibrationService/` | `SVC/calibration_service_manifest.textproto:19-45` |
| gRPC service | `intrinsic_proto.perception.v1.CalibrationService` | `PROTO:47-73` |

The ChArUco detector is linked through a registrar library marked
`alwayslink` (`SVC/BUILD:36`, `CAL/BUILD:47-55`).

### 1.2 Flags defined by the binary

| Flag | Default | Use | Source |
|---|---|---|---|
| `--runtime_context_file` | `/etc/intrinsic/runtime_config.pb` | binary `intrinsic_proto.config.RuntimeContext` | `MAIN:45-47` |
| `--grpc_client_connection_timeout` | `kGrpcClientConnectDefaultTimeout` = 5 s | wait for each dependency channel | `MAIN:49-51`; `ic/intrinsic_sdk/intrinsic/connect/cc/grpc/channel.h:35-36` |

Transitive flags: the impl links `//intrinsic_sdk/intrinsic/platform/pubsub:kvstore`
(`SVC/BUILD:66`), and it uses `PubSub` and `KeyValueStore` (`SVC/calibration_service_impl.h:113`, `:117`).
So `--zenoh_router` (`ic/intrinsic_sdk/intrinsic/platform/pubsub/zenoh_util/zenoh_config.cc:23-24`)
and `--use_replicated_kv_store` (`ic/intrinsic_sdk/intrinsic/platform/pubsub/kvstore.cc:53`)
should be present, as for the other services (`project/docs/designs/phases/phase_0_inventory.md:70-72`).
`main` calls `InitIntrinsic(argv[0], argc, argv)` (`MAIN:157-158`).
`--helpfull` was not run because there is no binary (see "Not verified").

### 1.3 Runtime config (RuntimeContext)

`MainImpl` reads the RuntimeContext file (`MAIN:56-61`) and unpacks
`context.config()` as `CalibrationServiceConfig` (`MAIN:63-67`). It listens
on `0.0.0.0:<context.port()>` with insecure credentials (`MAIN:127-142`).
No other RuntimeContext field is read in `MAIN`.

`CalibrationServiceConfig` has one field (`PROTO:34-43`):

```
intrinsic_proto.assets.v1.ResolvedDependency intrinsic_runtime = 1
  requires: "grpc://google.longrunning.Operations"
  requires: "grpc://intrinsic_proto.assets.v1.AssetInstances"
  requires: "grpc://intrinsic_proto.data_logger.DataLogger"
  requires: "grpc://intrinsic_proto.world.ObjectWorldService"
```

The default config sets only `intrinsic_runtime { name: "intrinsic_runtime" }`,
with no interfaces (`SVC/calibration_service_default_config.textproto:18-22`).
In k3s the asset system fills the interfaces in.

Each interface is resolved by `dependencies::Connect`
(`ic/intrinsic/assets/dependencies/utils.cc:85-98`):

- a missing key returns `NotFound` (`:54-73`)
- an interface without `grpc.connection` returns `InvalidArgument` (`:88-93`)
- otherwise it creates a channel to `connection.address`, with each
  `connection.metadata` entry added as a header on every call
  (`ic/intrinsic/assets/instances/connect/connect.cc:62-97`)

The project already writes a RuntimeContext in this form for the planner
(`project/container/config/motion_planner.runtime.txtpb:5-14`) and encodes
it with `project/container/encode_runtime.sh:1-12`.

---

## 2. Platform services needed at start-up

`MainImpl` connects in this order. Every `WaitForChannelReady` uses the
5 s flag above.

| # | Interface | What failure does | Source |
|---|---|---|---|
| 1 | `grpc://google.longrunning.Operations` | missing key or not ready within the timeout: `MainImpl` returns the error and `QCHECK_OK` aborts | `MAIN:69-77`, `:159` |
| 2 | `grpc://intrinsic_proto.assets.v1.AssetInstances` | same (fatal) | `MAIN:79-89` |
| 3 | `grpc://intrinsic_proto.world.ObjectWorldService` | same (fatal) | `MAIN:91-100` |
| 4 | `grpc://intrinsic_proto.data_logger.DataLogger` | **missing key: fatal** (`INTR_ASSIGN_OR_RETURN` on `Connect`). Key present but not ready: only `LOG(ERROR)`, and start-up continues | `MAIN:102-120` |

"Ready" means:

- the channel reaches `GRPC_CHANNEL_READY`
- a `grpc.health.v1.Health/Check` call returns OK **or `UNIMPLEMENTED`**

(`ic/intrinsic_sdk/intrinsic/connect/cc/grpc/channel.cc:136-161`, `:61-96`.)
Any gRPC server on the address passes, even one that does not implement
the dependency's service.

The binary constructs the impl with an Operations stub, an AssetInstances
stub and a world stub. It passes **`nullptr` for the AssetDeploymentService
stub** and no KV store (`MAIN:122-124`). The constructor's last two
parameters default to `nullptr` (`SVC/calibration_service_impl.h:64-75`).

The service does **not** use these at start-up or per call:

- the ResourceRegistry (no include or stub in `MAIN` or `IMPL`; the caller passes resource handles)
- ICON (no ICON dependency in `SVC/BUILD:23-131`)
- CAS
- the motion planner

### 2.1 Hard-coded cluster address

`kIngressAddress = "istio-ingressgateway.app-ingress.svc.cluster.local:80"`
(`IMPL:101-102`). It is used only to create the `AssetDeploymentService`
stub on first use (`IMPL:1938-1958`), and that stub is used only by
`SaveIntrinsicCalibrationResults` (`IMPL:1830-1834`).
`GetOrCreateIngressChannel` calls `CHECK_OK` on
`connect::CreateClientChannel(kIngressAddress, now + 5 s)` (`IMPL:1939-1946`),
which connects with a deadline (`channel.cc:206-224`). Outside k8s this
address does not resolve, so a `Save` with intrinsic results would most
likely abort the process (inference; not run).

---

## 3. RPCs: request fields and behaviour

All RPCs take one mutex (`mutex_`), so calls are serialized
(`IMPL:403`, `:574`, `:840`, `:1572`, `:1598`, `:1655`).
The service is **stateful and in-memory**: captures live in
`calibration_data_` (`SVC/calibration_service_impl.h:122-133`). A restart
loses them.

### 3.1 `Initialize(InitializeRequest) → InitializeResponse`

Request fields (`PROTO:75-101`):

| Field | Meaning |
|---|---|
| `pattern_detection_config` (required) | the pattern detector config (§6) |
| `camera_resource_handles` (≥ 1, required) | one `ResourceHandle` per camera |
| `arm_part` (optional `ObjectReference`) | the robot object the camera pose is expressed against |
| `calibration_object` (optional `ObjectReference`) | the board object, whose pose is updated in the world after `Calibrate` |
| `belief_world_id` | default `"world"` |
| `edit_world_id` | default `"init_world"` |

Response: `coverage_map_topic_names` (`PROTO:103-108`).

Behaviour, in order:

1. It rejects a missing config or empty handles with `INVALID_ARGUMENT` (`IMPL:393-401`).
2. It sets `execute_world_id_` and `initial_world_id_` from the request or
   the defaults (`IMPL:405-410`; defaults `IMPL:105-106`).
3. KV store: it opens `pubsub_.KeyValueStore("kv_store")` once
   (`IMPL:412-417`; `kDefaultKeyPrefix = "kv_store"`,
   `ic/intrinsic_sdk/intrinsic/platform/pubsub/kvstore.h:50`). It then runs
   `GetAll("calibration/sessions/**")` and deletes every key returned, waiting for the
   query's on-done callback (`IMPL:418-430`).
4. It sets a new `session_id_` (UTC `%Y%m%d_%H%M%S`) and clears all captures
   and cached intrinsics (`IMPL:432-438`).
5. It builds the `PatternDetector` from the config (`IMPL:440-442`).
6. For each camera handle (`IMPL:444-497`):
   - `ConnectionParams` from `handle.connection_info.grpc`: `address`,
     `server_instance`, `header` (`IMPL:446-449`;
     `ic/intrinsic_sdk/intrinsic/skills/cc/skill_utils.cc:38-53`)
   - `CameraConfig` from `handle.resource_data["CameraConfig"].contents`
     (`IMPL:451-453`; `ic/intrinsic_perception/intrinsic/perception/cameras/camera.cc:62`, `:78-88`)
   - **`CameraService.DescribeCamera`** with `camera_config.identifier` (`IMPL:456-462`)
   - It needs one sensor with `PIXEL_INTENSITY` in `supported_pixel_types`,
     otherwise `FAILED_PRECONDITION` (`IMPL:468-480`). That sensor is the
     default sensor, and its `dimensions` are the image size (`IMPL:481`, `:494-496`).
   - Post-processing for every sensor is `skip_undistortion = true`, with no crop or resize (`IMPL:482-489`).
7. If `arm_part` or `calibration_object` is set, it uses `ObjectWorldService`
   on the **execute** world (`IMPL:503-529`):
   - `GetKinematicObject(arm_part)`, then `GetSingleIsoFlangeFrame()`. A
     failure returns `INVALID_ARGUMENT` (`IMPL:507-517`).
   - `GetObject(calibration_object)` (`IMPL:519-527`)
8. It waits for coverage-map tasks to finish. Then, per camera, it creates
   PubSub publishers `calibration/sessions/<session>/cameras/<i>/coverage_map`
   and `…/sharpness`, and publishes an empty coverage map
   (`IMPL:534-564`).

### 3.2 `CaptureData(CaptureDataRequest) → CaptureDataResponse`

Request fields (`PROTO:110-123`):

- `base_t_flange` (optional `Pose`)
- `detect_redundant_pattern` (bool)
- `detect_blurred_image` (bool)

Response fields (`PROTO:125-145`):

- `capture_data_status[]`, one per camera: `NO_ERRORS`, `NO_DETECTIONS`,
  `MULTIPLE_DETECTIONS`, `INSUFFICIENT_MARKER_POINTS`,
  `REDUNDANT_PATTERN_DETECTION` or `BLURRED_IMAGE`
- `annotated_image_locations[]` (KV `StorageLocation`)
- `capture_id`

Behaviour:

1. Preconditions: `Initialize` was called, and fewer than
   `kMaxCalibrationDataPoints = 100` captures are stored
   (`IMPL:579-593`, `:109`).
2. Robot pose (`IMPL:595-629`):
   - If `Initialize` set `arm_part`, it reads
     `world.GetTransform(robot, flange)` from the execute world.
   - If the request also has `base_t_flange`, the two must agree within
     `isApprox(…, 1e-4)`, otherwise `INVALID_ARGUMENT`.
   - If only one of the two is set, that one is used.
   - It stores the pose with the capture (`IMPL:631-634`).
3. Per camera (`IMPL:640-786`):
   - **`AssetInstances.GetAssetInstance(name = handle.name)`**. The
     `CameraConfig` comes from `asset_instance.config.hardware_device.service.service_config`
     (an `Any`). If that path is missing: `NotFound` (`IMPL:653-656`, `:1982-2017`).
   - **`CameraService.Capture`** with that `camera_config`, timeout 5 s and
     the post-processing map (`IMPL:658-667`; `kDefaultImageGrabbingTimeout`, `IMPL:97`).
   - The first sensor image of type `Rgb8u`, `Gray8u` or `Gray32f` is used.
     Its `camera_params` (from `SensorImage.sensor_config.camera_params`,
     `ic/intrinsic_perception/intrinsic/perception/proto_conversion/v1/sensor_image.cc:50-58`)
     are kept per capture (`IMPL:675-679`).
   - It runs `pattern_detector_->Run` (`IMPL:681-684`).
   - Classification (`IMPL:716-741`):
     - 0 detections: `NO_DETECTIONS`
     - more than 1 detection: `MULTIPLE_DETECTIONS`
     - fewer than 10 points or a low-rank detection: `INSUFFICIENT_MARKER_POINTS`
       (`kMinNumberPatternImagePoints = 10`, `IMPL:111`)
   - Optional blur check (`IMPL:743-763`) and redundancy check (`IMPL:764-780`).
   - A camera with any non-OK status still yields a capture. Its
     `PatternDetection` is empty (`IMPL:644-645`, `:782`).
4. It assigns `capture_id = "%04d"` (an increasing counter) (`IMPL:788`).
5. **KV writes** (`IMPL:794-828`):
   - the annotated JPEG goes to `calibration/sessions/<session>/captures/<id>/cameras/<i>/annotated_image`
   - the sharpness score goes to `…/sharpness`
   - both use `kvstore_->Set(key, value, /*high_consistency=*/true)`, and a
     failure fails the RPC

### 3.3 `DeleteCapture(DeleteCaptureRequest{capture_id}) → Empty`

It removes one capture and updates the coverage map. An unknown id returns
`NOT_FOUND` (`PROTO:147-149`; `IMPL:836-858`).

### 3.4 `Calibrate(CalibrationRequest) → CalibrationResult`

Request fields (`PROTO:151-177`):

| Field | Meaning |
|---|---|
| `calibrate_intrinsics` | run intrinsic calibration |
| `calibrate_camera_to_robot` | run camera-to-robot calibration |
| `camera_to_robot_calibration_type` | `CAMERA_SETUP_STATIONARY` or `CAMERA_SETUP_MOVING` |
| `calibrate_camera_to_camera` | run camera-to-camera calibration |
| `calibrate_capture_to_capture` | compute transforms between captures |
| `exclude_capture_ids[]` | captures left out of this run |

Result (`PROTO:294-307`) has:

- `intrinsic_calibration_results[]`
- `camera_to_robot_calibration_results[]` (`CameraToRobotCalibrationResult`)
- `camera_to_camera_calibration_result`
- `capture_to_capture_calibration_result`

Behaviour: it filters out the excluded ids, then runs the requested parts
in the order intrinsics → camera-to-robot → camera-to-camera →
capture-to-capture (`IMPL:1593-1639`).

Camera-to-robot (`IMPL:990-1133`):

1. It needs `arm_part` from `Initialize`, otherwise `FAILED_PRECONDITION`
   (`IMPL:995-1001`).
2. Camera intrinsics: it uses the cached results of a previous
   `calibrate_intrinsics`, otherwise the `camera_params` of the **last**
   capture. If neither exists: `FAILED_PRECONDITION` (`IMPL:1007-1009`, `:956-986`, `:1027-1032`).
3. `camera_t_sensor` is always identity. The local `default_sensor_config`
   is built with only `id` and `camera_params`, so the `has_camera_t_sensor()`
   branch is never taken (`IMPL:1017-1040`).
4. For each capture with a detection, it computes the board pose with
   `ComputePoseFromPatternDetection`. It then shifts the pose from the
   OpenCV board corner to the **board centre, rotated π about x**
   (`corner_t_center`, `IMPL:1062-1077`), and adds the pair
   `(camera_t_object, base_t_flange)` (`IMPL:1078-1081`).
5. It calls `CalibrateCameraToRobot(request)` (`IMPL:1105-1107`). The
   service never sets `algorithm`, so it is `AUTO`, which means
   `NONLINEAR`: OpenCV SHAH refined with Ceres
   (`ic/intrinsic_apis/intrinsic/perception/proto/v1/camera_to_robot_calibration.proto:49-59`;
   `CAL/camera_to_robot_calibration.cc:282-293`, `:320-321`). At least 3
   pose pairs are needed (`CAL/camera_to_robot_calibration.cc:415-418`).
6. **It writes into the execute world** (`IMPL:1098-1131`):
   - `GetObject(camera_resource_handle)`, which is a `GetObjectRequest.resource_handle_name`
     lookup (`ic/intrinsic_sdk/intrinsic/world/objects/object_world_client.cc:301-307`)
   - frame `"sensor"` (`ic/intrinsic_sdk/intrinsic/world/objects/object_world_ids.cc:41-43`)
   - Stationary: `UpdateTransform(robot, camera "sensor" frame, node_to_update = camera, base_t_camera)`.
     Moving: `UpdateTransform(flange, camera_frame, camera, flange_t_camera)` (`IMPL:230-256`).
   - If `calibration_object` was given, its pose is updated too.
     Stationary: `flange_t_object`. Moving: `base_t_object` (`IMPL:279-308`).

### 3.5 `Validate(ValidateRequest) → ValidationResult`

Request fields (`PROTO:179-191`):

- `validate_intrinsics`
- `validate_camera_to_robot`
- `camera_to_robot_calibration_type`
- `validate_camera_to_camera`

It runs **no solver**. Camera-to-robot (`IMPL:1407-1450`):

1. It builds the same pose pairs as `Calibrate`, from all stored captures.
2. It reads the camera pose **currently in the execute world**
   (`base_t_camera` or `flange_t_camera`).
3. `ComputeCameraToRobotValidationMetrics` returns RMS and max translation
   and rotation errors (`CameraToRobotValidationResult`,
   `camera_to_robot_calibration.proto:114-124`).

Captures are not split into calibration and validation sets: `Validate`
uses `calibration_data_` (`IMPL:1574-1589`). A separate validation set
needs a fresh `Initialize` and new captures.

### 3.6 `Save(CalibrationResult) → Empty`

The request is a `CalibrationResult`, normally the one `Calibrate`
returned (`PROTO:72`). Behaviour (`IMPL:1641-1665`):

- An empty result returns `INVALID_ARGUMENT`.
- A capture-to-capture result only triggers a warning.
- If `intrinsic_calibration_results` is non-empty, it runs `SaveIntrinsicCalibrationResults` (§8.2).
- If `camera_to_robot_calibration_results` or a camera-to-camera result is
  present, it runs `SaveCameraPosesToInitialWorld` (§8.1).

---

## 4. How images are obtained

- Camera access is **CameraService over gRPC at the address in the
  caller's `ResourceHandle`** (`handle.connection_info.grpc.address`,
  optional `server_instance` and `header`). The calls go through
  `GrpcCamera::Call` (`IMPL:446-449`, `:456-462`, `:658-667`;
  `ic/intrinsic_perception/intrinsic/perception/cameras/camera.h:236-281`).
  There is no registry lookup.
- `DescribeCamera` at `Initialize` uses the `CameraConfig` in the handle's
  `resource_data["CameraConfig"]` (`camera.cc:78-88`).
- `Capture` at every `CaptureData` uses the `CameraConfig` from
  **AssetInstances** (`IMPL:653-656`).
- Request: `camera_config`, `timeout = 5 s`, `post_processing_by_sensor_id`
  with `skip_undistortion = true` (`IMPL:658-663`, `:484-488`).
  `sensor_ids` and `capture_result_location` are left unset (`camera_service.proto:168-199`).
- Response requirements:
  - `capture_result.sensor_images` is non-empty (`IMPL:671-674`)
  - one image is `Rgb8u`, `Gray8u` or `Gray32f` (`IMPL:675-677`)
  - `sensor_config.camera_params` is set. This is needed by
    camera-to-robot unless intrinsics were calibrated first (`IMPL:975-981`).
    It is also needed by the annotation step: `PatternDetector::Run`
    `INTR_RET_CHECK`s `camera_params` when `publish_annotated_image` is true
    (`CAL/pattern_detector.cc:54-60`). The skill always sets that flag
    (`SK/initialize_calibration.cc:160-161`).
- Annotated images also go to PubSub topic `/perception/annotated_frame`
  (`CAL/pattern_detector.cc:76-77`) and to the DataLogger
  (`CAL/pattern_detector.cc:125-131`). The DataLogger write is asynchronous,
  and its errors are only logged
  (`ic/intrinsic_perception/intrinsic/perception/logging/image.cc:61-85`).
- The project's planned CameraService is `camera_sim`
  (`project/docs/designs/robot_platform.md:516-518`). No implementation
  exists in the repo yet (rg for `CameraService` outside `docs/` and `gen/`
  finds only docs and `architecture.svg`).

## 5. How robot poses are obtained

- The service **never calls ICON**. `base_t_flange` comes from:
  - the caller (`CaptureDataRequest.base_t_flange`)
  - and/or the execute world, `GetTransform(arm_part object, its single ISO flange frame)`
    (`IMPL:600-629`)
- If `arm_part` is set, the world value is always read. A request value
  that differs by more than 1e-4 (Eigen `isApprox`) is rejected
  (`IMPL:616-623`). So the world's joint state must match the real arm at
  capture time.
- The reference skill moves the arm, then reads
  `world.GetTransform(robot, flange)` and sends it as `base_t_flange`
  (`SK/collect_calibration_data.cc:131-155`). It updates the world joints
  from ICON only once, at the start (`SK/collect_calibration_data.cc:218-227`).
  For each later capture it relies on something else keeping `world` current.
- The world's own ICON → joints sync is `WorldUpdater`. It subscribes only
  to robots that the **ResourceRegistry** lists with a position-controlled
  ICON part (`ic/intrinsic/world/service/updater/world_updater.cc:1050-1110`).
  If none match, it subscribes to nothing (`:1057-1061`). The project's
  registry stand-in has no instances (`project/container/services/platform_stubs.py:24-35`).
  So in today's container nothing copies ICON joints into `world`
  (inference from the code; not observed).
- `arm_part` must be a kinematic object with exactly one ISO flange frame
  (`ic/intrinsic_sdk/intrinsic/world/objects/kinematic_object.cc:146-164`).
  The project's `franka` has a frame named `flange`, which counts as the
  flange (`project/robot_model/franka_scene_object.py:353-360`;
  `project/docs/designs/phases/phase_1_inventory.md:364-367`).
- "Base" in every result is the `arm_part` object's own transform node:
  `base_t_camera` is set by `UpdateTransform(robot, …)` (`IMPL:246-250`).

## 6. Calibration targets

- `PatternDetectionConfig` (`ic/intrinsic_apis/intrinsic/perception/proto/v1/pattern_detection_config.proto:23-42`):
  - `name` (required)
  - `oneof Type { CharucoPatternDetectionConfig charuco_pattern_detection_config = 2; }`.
    **ChArUco is the only detector type.**
  - `publish_annotated_image`
- `CharucoPattern` (`ic/intrinsic_apis/intrinsic/perception/proto/v1/charuco_pattern.proto:22-34`):
  - `squares_x`, `squares_y`
  - `square_length` (m), `marker_length` (m)
  - `dictionary`: `DICT_ARUCO_ORIGINAL`, or `DICT_{4,5,6,7}X{4,5,6,7}_{50,100,250,1000}` (`:36-54`)
- The board is built as
  `cv::aruco::CharucoBoard(Size(squares_x, squares_y), square_length, marker_length, dict)`,
  **without `setLegacyPattern`** (`CAL/charuco_pattern_utils.cc:75-109`).
  Intrinsic's own board generator calls `setLegacyPattern(True)`
  (`CAL/charuco_boards/generate_charuco_board.py:180-185`).
  The two layouts differ only for an even number of rows (OpenCV issue
  23152, cited at `:183`; general OpenCV knowledge, not checked here).
- The project board is 7 × 5 squares, 30 mm squares, 22 mm markers,
  `DICT_5X5_100`, no legacy flag (`project/tools/make_charuco.py:1-24`).
  5 rows is odd, so the legacy question does not arise. The board is
  0.21 × 0.15 × 0.005 m, fixed to the hand 0.12 m along +z
  (`project/sim/scene.py:22-33`, `:83-115`).
- The OpenCV module is `opencv 4.12.0.bcr.5` (`ic/MODULE.bazel:525`).
- Object frame used by the solver: the board centre, rotated π about x
  from OpenCV's corner frame (`IMPL:1070-1077`).
- The skill can also derive the config from a data asset `PerceptionModel`
  whose first target is a ChArUco marker (`SK/initialize_calibration.cc:61-104`).
  That path needs a DataAssets service and is not needed here.

## 7. Eye-to-hand and eye-in-hand

Both are supported through `CameraSetup` (`ic/intrinsic_apis/intrinsic/perception/proto/v1/camera_setup.proto:22-42`):

| Setup | Unknowns solved | Written to the world | Source |
|---|---|---|---|
| `CAMERA_SETUP_STATIONARY` (eye-to-hand; the project's case) | `base_t_camera`, `flange_t_object` | camera "sensor" frame relative to the robot; optionally the board relative to the flange | `camera_to_robot_calibration.proto:86-92`; `IMPL:243-250`, `:295-302` |
| `CAMERA_SETUP_MOVING` (eye-in-hand) | `flange_t_camera`, `base_t_object` | camera relative to the flange; optionally the board relative to the robot | `:94-98`; `IMPL:235-242`, `:287-294` |

One setup applies to all cameras in a request (`PROTO:156-165`).
The solver's algorithm choices are listed at
`camera_to_robot_calibration.proto:48-72`. The service always uses
`AUTO` (§3.4).

## 8. What `Save` writes and where

### 8.1 Camera poses go to the initial world (camera-to-robot and camera-to-camera)

`SaveCameraPosesToInitialWorld` (`IMPL:1667-1767`):

1. It opens an `ObjectWorldClient` on `initial_world_id_` (default `"init_world"`) (`IMPL:1669-1671`).
2. The number of results must equal the number of cameras (`IMPL:1674-1680`).
3. For each camera it calls `GetObject(WorldObjectName(handle.name))`. This
   is a lookup **by object name** in the initial world, unlike `Calibrate`,
   which looks up the resource name in the execute world. It then takes the
   object's `"sensor"` frame (`IMPL:1684-1696`).
4. It applies the same `UpdateTransform` as §3.4, but with `robot` and
   `flange` = **the `TransformNode`s fetched from the execute world at
   `Initialize`** (`IMPL:1697-1703`). `UpdateTransform` sends these as
   **object IDs** (`object_world_client.cc:110-140`, `:606-612`). So the
   robot and flange in `init_world` must have the same IDs as in `world`.
5. It logs `"Saving the initial world via the conductor service is unavailable. You need to copy camera poses manually."`
   and returns OK (`IMPL:1760-1766`).

`robot_object_.value()` and `robot_flange_.value()` are dereferenced
without a check (`IMPL:1700-1701`). A `Save` of camera-to-robot results
after an `Initialize` without `arm_part` would hit an empty `std::optional`.

Where the initial world comes from in this container:

- `world_binary` creates **two empty worlds at start-up, `world` and
  `init_world`** (`ic/intrinsic/world/binary/world_binary.cc:143-147`,
  `:163-168`). It loads any saved worlds from `--worlds_path` first
  (`:150-154`).
- `CloneWorld` accepts `init_world` as a destination
  (`ic/intrinsic/world/service/world_storage.cc:272-280`). It copies with
  `world.Clone()` (`ic/intrinsic/world/service/object_world_service.cc:1719-1723`).
  Whether the clone keeps object IDs was not checked.
- Persistence: `--save_worlds_to_disk` defaults to `true`
  (`world_storage.cc:60`). Changed worlds are marked
  (`world_storage.cc:430-450`) and written by `FlushImpl` (`:452-470`).
  The project passes `--worlds_path=/var/lib/intrinsic/worlds`
  (`project/container/entrypoint.sh:23-24`) on the persistent volume
  (`project/docs/designs/phases/phase_0_notes.md:54-56`). So a changed
  `init_world` should survive a restart. This is an inference: the flush
  trigger was not traced and no restart was tested.
- The loader deletes and recreates `franka` and `franka_hand` in `world` at
  every start (`project/robot_model/load_franka.py:73-116`). Their IDs in
  `world` can then differ from those in a saved `init_world`. This matters
  for point 4 above (inference).

### 8.2 Intrinsics go to the camera resource config (not used for eye-to-hand)

`SaveIntrinsicCalibrationResults` (`IMPL:1769-1857`):

1. It reads the camera config from AssetInstances (`IMPL:1793-1796`).
2. It writes `intrinsic_params` and `distortion_params` into the default
   sensor's `SensorConfig.camera_params` (`IMPL:1798-1817`).
3. It calls **`AssetDeploymentService.UpdateResource`** through the
   hard-coded ingress (§2.1), then **`Operations.WaitOperation`** on the
   returned operation (`IMPL:1819-1851`).

Multi-sensor cameras are rejected (`IMPL:1786-1791`).

## 9. `pose_sampling` library

- Target: `//intrinsic_perception/intrinsic/perception/calibration:pose_sampling`.
  It is a `cc_library` depending on eigenmath, `math:pose3` and
  `absl/random` (`CAL/BUILD:237-246`).
- It is header-only. `pose_sampling.cc` only includes the header
  (`CAL/pose_sampling.cc:15`). All code is templates in
  `namespace intrinsic::perception` (`CAL/pose_sampling.h:22-126`).
- API (`CAL/pose_sampling.h`):
  - `GenerateRandomVector3d(double min = 0, double max = 1, URBG&& = URBG())`
    returns a uniform `Vector3d` (`:24-30`).
  - `struct SampleRandomPoseParams { Vector3d sample_box_halfsize; double rotation_randomization_angle_radians = 0; double rotation_randomization_roll_angle_radians = 0; }` (`:32-36`)
  - `Pose3d SampleRandomPose(const Pose3d& base_t_flange, const Pose3d& flange_t_attached_object, const Pose3d& base_t_stationary_object, const SampleRandomPoseParams&, URBG&& = URBG())`
    (`:46-124`). It returns the **sampled `base_t_flange`**. The steps:
    1. Translate the attached object uniformly within ±halfsize, on the
       base axes (`:56-65`).
    2. Point its z axis at the stationary object's origin, flipping z if
       the result faces away from the current z (`:71-91`).
    3. Tilt uniformly within ±angle about its x and y axes, and roll within ±roll about z (`:93-114`).
    4. Return `sampled_attached * flange_t_attached_object⁻¹` (`:118-123`).
- It does **no** reachability or collision check. The only caller is the
  `SampleCalibrationPoses` skill (rg for `pose_sampling` and
  `SampleRandomPose` in BUILD, `.py` and `.cc` files; `SK/BUILD:371`).
  That skill:
  - for `CAMERA_SETUP_STATIONARY`, uses the camera's `"sensor"` frame as
    the stationary object and the board as the attached object
    (`SK/sample_calibration_poses.cc:333-346`)
  - tries `10 × num_samples` poses (`:85`, `:348-349`)
  - keeps poses with a collision-free same-branch IK solution
    (`MotionPlannerService.ComputeIk`) **and** a successful `PlanTrajectory`
    to it (`:243-308`, `:385-414`)
  - needs `num_samples ≥ 4` (`SK/sample_calibration_poses.h:40`; `:170-175`)
  - skill parameters: `num_samples`, `sample_box_halfsize` (3 values),
    `rotation_randomization_angle_degrees`, `rotation_randomization_roll_angle_degrees`,
    `ensure_same_branch_ik`, `seed` (`SK/sample_calibration_poses.proto:28-53`, `:110`)
- **There are no Python bindings.** `CAL/BUILD` has only `cc_library`
  targets (`CAL/BUILD:15`, `:21-300`). rg for `pybind11` in
  `ic/intrinsic_perception` finds nothing. rg for `pose_sampling` and
  `SampleRandomPose` in `*.py` finds nothing.

## 10. Reference call sequence (Intrinsic's skills)

The skills are the in-repo clients. They reach the service through
`dependencies::Connect(params.calibration_service, "grpc://intrinsic_proto.perception.v1.CalibrationService")`
(`SK/initialize_calibration.cc:140-145`; `SK/collect_calibration_data.cc:237-249`;
`SK/save_calibration_result.cc:40`, `:56-68`; `SK/calibrate_camera_to_robot.cc:43`, `:94-107`).
The sequence:

1. `InitializeCalibration`:
   - sets `publish_annotated_image = true`
   - takes camera handles from the equipment slots
   - sets `arm_part` from the ICON position part's world object
   - sets `calibration_object`, `belief_world_id` and `edit_world_id` if given
   - calls `Initialize`

   (`SK/initialize_calibration.cc:147-219`)
2. `SampleCalibrationPoses` (§9).
3. `CollectCalibrationData`: for each waypoint, move, read `base_t_flange`
   from the world, then call `CaptureData`. A 
   `CaptureDataStatus` is not checked (TODO at `:159`). By default it
   returns to the start pose between waypoints (`SK/collect_calibration_data.cc:126-169`).
4. `CalibrateCameraToRobot`: `Calibrate` (`SK/calibrate_camera_to_robot.cc:88-107`).
5. `SaveCalibrationResult`: `Save` (`SK/save_calibration_result.cc:56-68`).

The skills need the executive and skill runtime, which the project
excludes (`project/docs/designs/robot_platform.md:126-127`, D4 at `:141`).
`robot_service` would make the same gRPC calls itself. No Python
`CalibrationService` stubs exist yet: `project/gen/intrinsic/perception/proto/v1/`
does not exist (ls printed nothing). `project/container/gen_protos.sh:1-21`
can generate them from `PROTO` (not run, because it writes into `gen/`).

---

## 11. Minimal stand-ins to run it without k3s

| Need | When | What the stand-in must do | Existing piece | Source |
|---|---|---|---|---|
| RuntimeContext file | start-up | binary `RuntimeContext` with `port` and `config = Any(CalibrationServiceConfig{intrinsic_runtime{interfaces{…4 keys…}}})`, each `grpc.connection.address` set | `encode_runtime.sh`; planner example | §1.3 |
| `google.longrunning.Operations` | start-up (fatal); per call only for intrinsic `Save` | any gRPC server on the address (Health `UNIMPLEMENTED` passes). `WaitOperation` only if intrinsic `Save` is used | `platform_stubs.py` on 18010 would pass the readiness check | §2; `IMPL:1842-1845` |
| `AssetInstances` | start-up (fatal); **every `CaptureData`**; intrinsic `Save` | `GetAssetInstance(name = camera handle name)` returning `AssetInstance.config.hardware_device.service.service_config = Any(CameraConfig)` | none; new RPC on the stub server | `IMPL:1982-2017`; `ic/intrinsic_apis/intrinsic/assets/proto/v1/asset_instances.proto:40`, `:112-153`; `ic/intrinsic_apis/intrinsic/assets/proto/v1/instance_config.proto:26-55` |
| `DataLogger` | start-up | the interface key must exist. The address may be unreachable (costs the 5 s wait, then `LOG(ERROR)`) | — | `MAIN:102-120` |
| `ObjectWorldService` | start-up (fatal); `Initialize`, `CaptureData`, `Calibrate`, `Validate`, `Save` | the existing `world_binary` on 18000, with the world contents listed below | `world_binary` | `project/container/entrypoint.sh:23-24` |
| Zenoh router | `Initialize` (PubSub, KV `GetAll`), `CaptureData` (KV `Set`) | reachable via `--zenoh_router=tcp/127.0.0.1:7447` | zenohd 7447 | `project/docs/designs/phases/phase_0.md:133` |
| **Zenoh KV storage for `kv_store/**`** | **every `CaptureData`** | answer queries for keys written under `kv_store/calibration/sessions/**` (see below) | **missing**: the project zenohd runs with only `--listen` and `--no-multicast-scouting` (`entrypoint.sh:18`), and the image copies only `/zenohd` (`project/container/Dockerfile:15`) | below |
| CameraService | `Initialize` (`DescribeCamera`), `CaptureData` (`Capture`) | §4 requirements: ≥ 1 `PIXEL_INTENSITY` sensor with dimensions, and a `CaptureResult` with an RGB8 / Gray8 / Gray32f image and `sensor_config.camera_params` | `camera_sim` (planned, not written) | §4 |
| Camera `ResourceHandle` (built by the caller) | `Initialize` | `name`, `connection_info.grpc.address` (+ optional `server_instance` / `header`), `resource_data["CameraConfig"].contents = Any(CameraConfig{identifier…})` | — | `ic/intrinsic_apis/intrinsic/resources/proto/resource_handle.proto:23-59`; `camera.cc:62`, `:78-88` |
| Up-to-date robot joints in `world` | every `CaptureData` (if `arm_part` is set) | `world`'s `franka` joints equal the arm's at capture time | none (`WorldUpdater` sees no robot, §5). Options: give the registry stub an ICON position-part resource, or have the caller send `UpdateObjectJoints` before each capture | §5 |
| AssetDeploymentService | intrinsic `Save` only | not reachable without k8s (hard-coded ingress) | — | §2.1 |

**Why the KV storage is required.** `Set(key, value, true)` is
`SetWithVerification(kHighConsistency, timeout = 30 s)`
(`kvstore.cc:224-240`). It first does a `Get` (up to 10 s, `:263-287`),
then `imw_set`, then polls `Get` until the stored bytes match
(`:364-411`). A `Get` with no replying storage returns `NotFound`
(`kvstore.cc:437`, `:476-483`). `NotFound` leaves the "unchanged" state
equal to the initial state, so the loop keeps polling until
`DeadlineExceeded` after 30 s (`:376-379`, `:386-410`). `CaptureData` then
fails (`IMPL:804-805`). The constructor's `kvstore` injection exists only
for tests, and `main` does not use it (`SVC/calibration_service_impl.h:116`;
`MAIN:122-124`).

In k8s the zenoh router loads `storage_manager` with a storage on
`kv_store/**` (`ic/intrinsic/platform/pubsub/fds_core.yaml:243-280`). That
storage is file-backed and needs `/libzenoh_backend_fs.so` and
`ZENOH_BACKEND_FS_ROOT`. The same file uses `"volume": "memory"` for other
key spaces (`:300-307`). The zenohd image contains
`/libzenoh_plugin_storage_manager.so` and `/libzenoh_backend_fs.so`
(`docker image save` listing above). A memory or fs storage on
`kv_store/**` therefore looks possible with files the project already
pulls. This was not run.

**World contents the service expects.**

- `world` (execute world):
  - the robot object named in `arm_part` (`franka`, one ISO flange)
    (`IMPL:507-517`)
  - a **camera object found by `resource_handle_name == camera handle name`**,
    with a frame named `"sensor"` (`IMPL:1111-1115`)
  - optionally the board object (`calibration_object`)
- How the world resolves `resource_handle_name`
  (`ic/intrinsic/world/objects/object_world.cc:126-150`, called from
  `ic/intrinsic/world/service/object_world_service.cc:1882-1885`). It
  matches either:
  - the object's PPR resource name (`ic/intrinsic/world/objects/world_object_internal.cc:467-484`), or
  - for compatibility, any member entity whose **local name** equals the instance name (`object_world.cc:135-144`)
- `CreateObjectRequest` has no resource-name field
  (`ic/intrinsic/world/proto/object_world_updates.proto:200-245`). PPR
  resource names are set by:
  - the resource-instance path (`world_object_internal.cc:803-835`; RPCs
    `CreateWorldFromResourceInstances` and `CreateWorldFromResourceSetData`,
    `ic/intrinsic/world/proto/object_world_service.proto:1229-1260`)
  - SDF conversion (`ic/intrinsic/world/conversion/sdf/sdf_to_world.cc:156`, `:166`)
  - `CreateFrame.match_object_asset_name`, which copies an existing name
    and does not create one (`ic/intrinsic/world/service/objects/object_world_updates_utils.cc:892-905`)

  Which route the project uses (for example an entity named after the
  handle) is open. It needs an experiment.
- `init_world` (initial world), for `Save`:
  - a camera object **named** exactly the handle name, with a `"sensor"` frame
  - the robot and its flange with **the same object IDs as in `world`** (§8.1)
  - the camera's parent chain must let `UpdateTransform` change
    robot→sensor by moving the camera (`node_to_update` must not be an
    ancestor of both nodes, `object_world_updates.proto:707-711`)

---

## 12. Facts that bear on the design text

- `Calibrate` already writes the new camera pose into `world` (`IMPL:1125-1130`).
  `Save` writes only into `init_world`, and logs that it cannot persist
  through the conductor (`IMPL:1760-1765`). The design's "Save writes the
  camera pose into the world's initial state and the camera's configuration
  through the asset-instances stand-in"
  (`project/docs/designs/robot_platform.md:500-502`) has two problems.
  First, the camera configuration is written only for **intrinsic**
  results. Second, it goes through **AssetDeploymentService.UpdateResource**
  via the hard-coded ingress, not AssetInstances (§8.2). For eye-to-hand,
  `Save` touches only `init_world`.
- `CaptureData` needs a Zenoh KV storage. The current zenohd has none (§11).
- `CaptureData` compares the world's flange pose with the request's
  (1e-4). With `arm_part` set, the world's joints must be current, and
  nothing in today's container keeps them current (§5).
- `Validate` reuses the calibration captures (§3.5).
- The service always uses `AUTO` (`NONLINEAR`) and identity `camera_t_sensor`.
  The solved pose belongs to whatever world frame is named `"sensor"`
  (§3.4). The D455's optical frame convention therefore has to match that
  frame. How Isaac's camera frame maps to it is not covered here.

## 13. Not verified

- That `calibration_service_main` builds and links in this checkout, and
  its full flag list (`--helpfull`). No Bazel run; the binary is absent
  from `bazel-bin`.
- The runtime behaviour of everything above. Nothing was run, and the
  service was never started.
- Whether zenohd 1.7.2 with `storage_manager` and a memory (or fs) volume
  on `kv_store/**` satisfies the high-consistency `Set`. Also not checked:
  whether `/zenohd_config.json5` (the image's default config) exists in
  the image (it did not match the listing filter).
- Whether `CloneWorld` keeps object IDs, and so whether
  `CloneWorld(world → init_world)` meets `Save`'s same-ID requirement.
- Which world-API route gives the camera object a resource name, or a
  member entity whose local name matches the handle.
- Whether `GrpcCamera`'s connection cache runs a health check with the
  `x-resource-instance-name` header against the camera stand-in
  (`camera.h:268-269`, implementation not read).
- When `WorldStorage::FlushImpl` runs (periodic or on shutdown), and so
  how soon a `Save`d `init_world` reaches disk.
- The behaviour of `GetOrCreateIngressChannel` outside k8s. The process
  is expected to abort (§2.1); not run.
- OpenCV's legacy ChArUco layout rule (even rows only). This comes from
  OpenCV knowledge and the cited issue, not from code read here.
- Isaac-side facts: the D455 frame convention against the `"sensor"`
  frame, and the board's rendered appearance. Isaac was not started (parallel-safety rule).
