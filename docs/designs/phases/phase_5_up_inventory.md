# Phase 5 inventory (upstream path): Intrinsic `nvidia_pose_estimator`

Status: research only. Written 2026-10-28. Nothing was built, and nothing ran on the GPU.

Scope: owner decision D9 (`robot_platform.md:150`) selects SAM 3, FoundationStereo and FoundationPose
on TensorRT through Intrinsic's upstream `nvidia_pose_estimator`. This file records what that
service is, what it needs, and what is not known yet. It answers six questions: the service
itself, its inputs, its models, GPU in Docker on WSL2, licences, and how intrinsic-omts wires it.

Method: I cloned the upstream repositories into `/home/mew/src/` and read them. I also read local
files, ran read-only commands (`docker info`, `nvidia-smi --query-gpu`, `ldd --version`, file
listings) and fetched vendor pages into `/tmp/p5up/`. The rules for this task forbade running Bazel,
building images, starting Isaac Sim and using the GPU. So every memory and timing figure below comes
from upstream text and none was measured here.

## 0. Reference prefixes

| Prefix | Meaning |
|---|---|
| UP | `github.com/intrinsic-ai/intrinsic-core` main `757cd2c000ac52a256a766e42898d88795ff5bb2` (2026-10-27), clone in `/home/mew/src/intrinsic-core` |
| UPpin | the same repo at `50927e35c5c85992b2a4d0b91dabb9cf4172b2d5` (2026-10-22, "Bump to 20261022.0 (#3921)"). This is the commit intrinsic-omts pins (OMTS `MODULE.bazel:14`) |
| PKG | `intrinsic_perception/intrinsic/perception/service/nvidia_pose_estimator` inside UP |
| PIPE | `github.com/NVlabs/foundationpose_perception_pipeline`, at the commit `cd4965a1…` that UP pins (`UP bazel/non_module_deps.bzl:70-76`) |
| FPIL | `github.com/NVlabs/foundation-pose-inference-library` `32d1122bf819b4db5b6d823aaa4092e27a562091` (pinned at `UP bazel/non_module_deps.bzl:62-68`) |
| OMTS | `github.com/intrinsic-ai/intrinsic-omts` `bd814610b58c295025a7294ebe96b42e0c7ebc2f` (2026-10-23) |
| IC | local `/home/mew/intrinsic-core`, branch `fix/so3-from-matrix-reject-reflections`, HEAD `b2f9336` |
| P | this project, `/home/mew/untitled-project`, HEAD `2a4ca0f` |
| NV-WSL, NV-CTK, NV-CDI | NVIDIA CUDA-on-WSL user guide v13.4, Container Toolkit install guide, and CDI support page (saved as `/tmp/p5up/{wsl,ctk,cdi}.html`) |
| MS-WSL | Microsoft Learn "Enable NVIDIA CUDA on WSL" (`/tmp/p5up/ms_gpu.html`, `ms_gpu2.html`) |

Between UPpin and UP the estimator changed in only four files (`git diff 50927e3..757cd2c -- PKG`):

- The README gained §3 Configuration, which includes the GPU memory table.
- One proto comment changed.
- The SAM 3 script pin moved from PIPE `816eaee5…` to `cd4965a1…`.
- The service gained `retain_pose_context=True` (`service.py:607-611`), a log of how long estimator creation takes, and a fix to the stereo disparity resize (`service.py:912-927`).

`bazel/non_module_deps.bzl` also moved the pipeline pin, from `4d79cf04…` to `cd4965a1…`. Every line
number below refers to UP unless the prefix says otherwise.

## 1. Headline findings

1. **An IR stereo pair is not required.** The default is `depth_source: DEPTH_SOURCE_HARDWARE`
   (`PKG/configs/default_config.textproto:74-81`). In that mode the service reads a `PIXEL_DEPTH`
   image and never loads FoundationStereo (`PKG/service/nvidia_pose_estimator_service.py:617-625`).
   The default sensor names are `"1"` for detection and `"4"` for depth. Our camera already
   publishes exactly those ids: 1 is RGB8 and 4 is float32 depth in metres (`P container/services/camera_sim.py:67-70`).
   OMTS also captures only `[1, 4]` (`OMTS configs/omts/app_config.yaml:33-41`).
2. **GPU memory is the blocker.** Upstream says HARDWARE mode needs **16 GB** and STEREO or AUTO
   needs **24 GB** (`PKG/README.md:318-340`). This machine has an RTX 3080 with 10240 MiB, shared
   with Isaac Sim (`nvidia-smi`). Our design budgets 3.5 GB for perception (`P docs/designs/robot_platform.md:313-319`).
   No upstream source gives a figure for a 10 GB card. Engines build in FP32 with TF32 by default,
   not FP16 (§4.4).
3. **The service only runs from a Bazel-built image.** It imports Bazel-built C++ extensions:
   - `intrinsic.platform.pubsub.python.pubsub`, a pybind module over Zenoh (`UP intrinsic_sdk/intrinsic/platform/pubsub/python/BUILD:23-35`);
   - `multi_view_pose_utils`, which depends on SDK geometry and world modules (`UP intrinsic_perception/intrinsic/perception/skills/multi_view/multi_view_pose_utils.py:25-38`).

   The image comes from `python_oci_image` on the `@tensorrt` base (`PKG/BUILD:311-321`). IC has
   no `nvidia_pose_estimator` package (`ls IC/intrinsic_perception/intrinsic/perception/service/`
   lists only `ioc_pose_estimator ioc_train_service multi_view python`).

   Once built, the image runs with plain `docker run` outside k3s (§2.4).
4. **The API is the same `intrinsic_proto.perception.v1.PoseEstimationService`** (`PKG/service_manifest.textproto:34`).
   The UP proto matches IC except that `CaptureDataList` moved into `capture_data.proto`, with the
   same package and fields (`diff` of the two files; `UP intrinsic_apis/.../capture_data.proto:17,35-37`).
   It is therefore wire-compatible with our generated stubs.
5. **Docker in WSL has no NVIDIA runtime.** `docker info` reports only `runc`, there is no
   `/etc/docker/daemon.json`, and `nvidia-ctk` is not installed. NVIDIA documents that the
   Container Toolkit with `--gpus all` is supported on WSL2 (NV-WSL §4, §5.1). The README's
   plain-runc recipe needs `/dev/nvidia*`, and those devices do not exist on WSL (§5).
6. **Licences.**
   - SAM 3 uses the Meta "SAM License". Its grant has no commercial-use exclusion, but it adds trade-control and military-use bans, a no-reverse-engineering clause, an indemnity, and terms Meta can change unilaterally. The Hugging Face checkpoint is gated and approved manually.
   - FoundationStereo and FoundationPose use the NVIDIA Open Model License and are marked "ready for commercial use".
   - The TensorRT container uses the NVIDIA Software License Agreement plus the AI Product Terms. Under those terms TensorRT may run only on NVIDIA platforms, and distribution as part of a product needs a notice and use reporting (§6).

## 2. Q1: the estimator itself

### 2.1 Source location

`PKG` is in UP. It was added after IC's base (IC does not have it), and OMTS uses it through UPpin.
The upstream TODO b/572956581 says the estimator will be removed from this source tree
(`UP MODULE.bazel:687,695`). Files and line counts:

| File | Lines |
|---|---|
| `service/nvidia_pose_estimator_service.py` | 1494 |
| `service/nvidia_service_main.py` | 138 |
| `README.md` | 587 |
| `BUILD` | 386 |
| `model_data_asset.bzl` | 271 |
| `proto/nvidia_service_config.proto` | 239 |
| `scripts/prepare_sam3_models.sh` | 227 |
| `service_manifest.textproto` | 93 |
| `configs/default_config.textproto` | 86 |

The `util/` directory holds the modules `asset_utils`, `config_utils`, `data_classes`, `debug_utils`,
`environment`, `image_utils`, `logging_utils`, `model_asset_utils`, `model_warmup`, `path_resolver`
and `pose_utils`. The service code wraps PIPE (SAM 3 and FoundationStereo through TensorRT), plus
FPIL's `libfoundation_pose_nvidia.so` (FoundationPose). It uses pure TensorRT 10.16 with no PyTorch
at runtime (`PKG/README.md:18-26`).

### 2.2 Bazel targets and flags

| Target or flag | Definition | Notes |
|---|---|---|
| `--//PKG:build_full_image` | `bool_flag`, default False (`PKG/BUILD:28-31`) | True bases the image on `@tensorrt` and bundles `libfoundation_pose_nvidia.so` (`BUILD:53-62,250-262,311-321`). False uses the plain Python base and gives an image that "won't run" (`README.md:88-97`) |
| `--//PKG:download_weights` | `bool_flag`, default False (`BUILD:43-46`) | True downloads about 480 MB of FoundationPose and FoundationStereo ONNX from NGC (`README.md:98-103`). The gate is `bazel/non_module_deps.bzl:78-81` |
| `:nvidia_pose_estimator_service_asset` | `intrinsic_service` (`BUILD:323-336`) | Service asset bundle for `inctl asset install` |
| `:nvidia_pose_estimator_image.tar` | `python_oci_image` (`BUILD:311-321`) | Image only (`README.md:108-110`) |
| `:foundationpose_model_data_asset`, `:foundationstereo_model_data_asset`, `:sam3_model_data_asset` | `BUILD:275-308`, all tagged `manual` | Data assets `ai.intrinsic.nvidia_pose_estimator.{foundationpose,foundationstereo,sam3}_models` (`README.md:115-135`). Version strings: `tao-deployable-v1.0`, `tao-deployable-s-dynamic-v2.0`, `sam3-608457d…` (`BUILD:265-272`) |
| `:nvidia_service_main` | `py_binary` (`BUILD:234-248`) | Entry point |

`python_oci_image` bundles the hermetic Python interpreter layer (`UP bazel/python_oci_image.bzl:137-162`)
and sets `cmd=[binary_path]` (`:271-272`). `PYTHON_VERSION` is `"3.11"` (`UP MODULE.bazel`). The
SAM 3 asset needs files staged by `prepare_sam3_models.sh` in `$NVIDIA_POSE_ESTIMATOR_MODELS_DIR`.
Without them Bazel prints a warning and skips the asset (`UP bazel/nvidia_pose_estimator_models.bzl:25-41,120-153`;
`PKG/model_data_asset.bzl:229-263`).

### 2.3 Image composition

- **Base image:** `oci.pull tensorrt`, image `nvcr.io/nvidia/tensorrt`, tag `26.05-py3`, index digest
  `sha256:94aa793e6db548940168186172b5acbe413cae9e56f7a656d171cd0dd9b6a869`, platform linux/amd64 (`UP MODULE.bazel:749-755`).
  The amd64 manifest is `sha256:dae6902dd2ee694ee3a21b7ad5e05a29c9377f707385f1957daf9fd84867bd2d`,
  6,168,528,080 bytes compressed, pushed 2026-06-01 (NGC images API, `/tmp/p5up/ngc_trt.json`).
  The README calls it "~5.8 GB" (`README.md:94`).
- **What the base contains:** release notes for 26.05 list Ubuntu 24.04, CUDA 13.2.1, TensorRT
  10.16.1.11, and GPUs of compute capability 7.5 or higher (`/tmp/p5up/trt_rn.html`). Releases
  26.06 and later move to TensorRT 11.x. The README says not to upgrade, because the bindings
  `tensorrt-cu13-bindings==10.16.1.11` must match (`README.md:516-528`; `UP requirements.in:195`).
- **Layers added on top:** the hermetic Python 3.11 interpreter with pip dependencies
  (`PKG/BUILD:185-232`: ftfy, numpy, opencv-python-headless 4.11.0.86, pillow, psutil, pycocotools,
  pyyaml, regex, scipy, tqdm, trimesh 4.6.12, wcwidth, grpc, cuda-bindings 13.2.0; `UP requirements.in:45,71-72,201`),
  then `libfoundation_pose_nvidia.so` in `/usr/lib/x86_64-linux-gnu` (`BUILD:250-262`).
  nvdiffrast is excluded because of its NVIDIA Source Code License (`README.md:509-512`).
- **Library loading:** `environment.init_environment()` runs before the other imports
  (`service.py:45`). It preloads libcudart, libnvrtc, libnvinfer, libnvonnxparser and the
  libnvinfer plugins (`util/environment.py:124-158`). It searches `FP_LIBRARY`, then
  `FP_INFERENCE_LIB_PATH`, then `/usr/lib/x86_64-linux-gnu` (`:90-110,161-171`), and raises
  `RuntimeError` unless `FP_ALLOW_MISSING_LIBRARY=1` (`:175-206`).

### 2.4 Building and running outside k3s

- **Build** (needs Bazel in an upstream checkout; not run here):
  `bazel build --//PKG:build_full_image=True //PKG:nvidia_pose_estimator_image.tar` (`README.md:80-110`).
- **Run** (`README.md:218-263`). Load the image with `docker load -i …image.tar`. Mount a host
  directory with all eight weight files at `/models` and set `-e MODELS_DIR=/models`. Then use
  either option:
  - Option A: `docker run -p 50051:50051 --gpus all …`, which needs the NVIDIA Container Toolkit.
  - Option B: plain runc with `/dev/nvidia*` devices and libcuda bind mounts, which works on native Linux only.

  The service listens on 50051 and logs warmup progress.
- **Without a cluster the service degrades on its own:**
  - `_in_cluster = bool(KUBERNETES_SERVICE_HOST)` (`service.py:483`).
  - It creates the InstalledAssets client only if `intrinsic_runtime` has interfaces (`:487-496`).
  - It creates the CAS stub only in a cluster or when `CAS_ADDRESS` is set (`:499-503`).
  - Without both clients it finds models through `path_resolver` (`$MODELS_DIR`, default `/tmp/models`) instead of downloading them (`:519-540`; `util/path_resolver.py:37-59`).
  - Engine plans go to `$MODELS_DIR/engine_cache` without namespacing (`README.md:552-587`).
- **Configuration without a cluster:**
  - With no `/etc/intrinsic/runtime_config.pb` the service starts from an empty config, which is merged over `default_config.textproto` (`service/nvidia_service_main.py:47-54,69-96`; `util/config_utils.py:74-154`).
  - To override settings, mount a `RuntimeContext` binary file whose `config` Any holds `NvidiaPoseEstimatorServiceConfig` and whose `port` is above 0 (`nvidia_service_main.py:72-86,120-127`).
  - The gRPC server uses 10 threads, no message size limits and an insecure port (`:56-60,100-102,120-127`).
- **CAD without a cluster:** `asset_utils.get_local_cad_path_from_asset` falls back to
  `/tmp/cad_models/<package>.<name>.ply` or `.obj` (`util/asset_utils.py:246-253`). The id format
  is `f"{package}.{name}"` (`UP intrinsic_sdk/intrinsic/assets/id_utils.py:159-166`). The mesh
  loader accepts only binary little-endian PLY, and scales meshes whose largest extent is below
  2.0 from metres to millimetres (`service.py:167-219`). An `.obj` fallback would therefore
  probably fail to load. That is inferred from the code and was not run.
- **Prompt without a cluster:** there is no assets client, so `resolve_prompt` returns None
  (`asset_utils.py:94-140`). The prompt then comes from the request's `inference_parameters`
  or from the config default `"object"` (`service.py:702-758`; `default_config.textproto`).
- **PubSub:** the service always tries to create `pubsub.PubSub()`, and failure is only a warning
  (`service.py:505-517`). The default Zenoh endpoint is
  `tcp/zenoh-router.app-intrinsic-base.svc.cluster.local:7447`, in peer mode with multicast off
  (`UP intrinsic_sdk/intrinsic/platform/pubsub/zenoh_util/zenoh_config.h:55-57`, `zenoh_config.cc:34-61`).
  It can be overridden only by `router_override` or the `--zenoh_router` flag (`zenoh_config.cc:100-120`).
  `debug.publish_visualization` defaults to true (`default_config.textproto`; `README.md:265-306`).

### 2.5 gRPC API

- **Service:** `intrinsic_proto.perception.v1.PoseEstimationService`, the same service IC's
  `ioc_pose_estimator` implements (`PKG/service_manifest.textproto:34`;
  `UP intrinsic_apis/intrinsic/perception/proto/v1/pose_estimation_service.proto:31-37`). It has
  two RPCs, `RunPoseEstimation` and `DeletePoseEstimation`.
- **Request** (`pose_estimation_service.proto:43-61`):
  - a `oneof` of `capture_result` (=1, images inline) or `capture_data_list` (=2, images read from the KV store through pubsub);
  - `asset_id` (=3);
  - `pose_estimation_run_config` (=4), whose `inference_parameters` Any carries a per-request `NvidiaPoseEstimatorServiceConfig` (`service.py:702-758`).
- **Request fields the service ignores:** `pose_priors`, `timeout`, `suppress_logging` and
  `publish_annotated_image` do not appear in PKG (`rg` returned no matches).
- **Request handling** (`service.py:1252-1449`):
  - `asset_id` is required and at least one view is needed, otherwise the call returns `INVALID_ARGUMENT`.
  - A per-request `artifact_dir` must be under `/tmp`.
  - Requests run under a single GPU lock and models initialise lazily.
  - Errors are reported as `INVALID_ARGUMENT`, `CANCELLED` or `INTERNAL`.
- **Response:** `pose_estimates`, sorted best-first.
  - Each estimate has `id = asset name`.
  - `camera_t_target` is in the frame of the detection sensor's `camera_t_sensor` (`pose_estimation_service.proto:45-50`).
  - `score = 1/(1+exp(fp_score/10))` with the input clipped to ±1000 (`util/pose_utils.py:35-49`).
- **DeletePoseEstimation:** evicts the cached estimator and unlinks the cached CAD (`service.py:1451-1494`).

### 2.6 `nvidia_service_config.proto`, fields and defaults

The package is `intrinsic_proto.perception.nvidia`. The message is `NvidiaPoseEstimatorServiceConfig`.
Defaults come from `PKG/configs/default_config.textproto:15-86`.

| # | Field | Default | Notes |
|---|---|---|---|
| 1 | `intrinsic_runtime` (ResolvedDependency) | name `"intrinsic_runtime"` | requires `grpc://intrinsic_proto.assets.v1.InstalledAssets` (`proto:26-30`) |
| 2 | `stereo` (StereoDepthConfig) | min 0.30 m, max 0.80 m, width 640, height 416, CLAHE clip 3.0, detail boost 1.5 | width must be a multiple of 32 and "can be adjusted to suit the input image size and GPU memory" (`proto:76-102`) |
| 3 | `sam3_detection` | prompt `"object"`, confidence 0.2, resolution 1008 | `proto:104-113`. PIPE requires resolution 1008 (`PIPE sam3/trt_processor.py:54-55`) |
| 4 | `sam3_refinement` | policy `REPLACE_MID_NMS06`, low mIoU 0.2, high mIoU 0.8, NMS 0.6 | `proto:115-128,158-166`; the policy `NONE` disables refinement |
| 5 | `foundation_pose` | n_refine 5, n_hypotheses 252, batch_size 42 | batch_size must divide n_hypotheses (`proto:130-140`) |
| 6 | `selection` | `SOFT_GLOBAL_V1`, 3.0, 2.0, 4.0, 1.0, 100.0 | `proto:142-155,169-177` |
| 7 | `sensor_routing` | detection `"1"`, `DEPTH_SOURCE_HARDWARE`, hardware depth `"4"`, stereo `["2","3"]` | `proto:180-225`. `UNSPECIFIED` means AUTO |
| 8 | `debug` | `publish_visualization: true` | `proto:227-239` |
| 9-11 | `foundationpose_model`, `foundationstereo_model`, `sam3_model` | the three asset names | each requires `data://intrinsic_proto.perception.v1.PerceptionModel` (`proto:56-69`) |

Sensor names are the stringified sensor ids (`README.md:371-375`;
`UP client/v1/python/camera/data_classes.py:356-363`). The prompt is resolved in this order: the
asset's `IocPoseEstimatorParams`, then the request, then the config (`service.py:702-758`;
`README.md:371-375`). `config_utils.build_inference_config` range-checks per-request values
(`util/config_utils.py:193-263`).

### 2.7 runtime_config and platform dependencies

| Dependency | Where it is used | Default address |
|---|---|---|
| RuntimeContext `/etc/intrinsic/runtime_config.pb` | provides config and port (`nvidia_service_main.py:47-54`) | missing file means defaults and port 50051 |
| InstalledAssets (through `intrinsic_runtime`) | resolves model assets and per-request `asset_id` metadata and CAD (`service.py:487-496`; `util/model_asset_utils.py:192-306`) | resolved from the ResolvedDependency |
| CAS | downloads model and CAD bytes, 2 parallel workers with 3 attempts each (`model_asset_utils.py:192-306`) | `content-addressable-storage.app-intrinsic-base.svc.cluster.local:9747`, overridden by `CAS_ADDRESS` (`service.py:87-90`) |
| Zenoh pubsub | visualisation topics `ioc_pose_estimator/{detections,poses}` (`README.md:265-306`) | `zenoh-router.app-intrinsic-base.svc.cluster.local:7447` |
| KV store | only for `capture_data_list` requests (`util/image_utils.py:210-288`) | `zenoh-router.app-intrinsic-base.svc.cluster.local:8081` (`UP …/pubsub/kvstore.cc:56`) |
| Host volume | `/var/lib/file-storage/service_volumes/intrinsic/nvidia_pose_estimator` mounted at `/models` | `service_manifest.textproto:38-62` (real) and `:67-91` (sim) |
| GPU | `resources.limits nvidia.com/gpu: 1`; the code uses CUDA device 0 | `service_manifest.textproto`; `service.py:101` |

In a cluster the model data assets are installed before `inctl service add`. Their names are
resolved once, when the instance is created, and the service fails to start if a dependency is
unresolved (`README.md:115-159`). Model files go to `/models/weights` and plans to
`/models/engine_cache/<16-hex namespace>` (`model_asset_utils.py:309-318`).

### 2.8 Registering objects and prompts

- **In a cluster:** a SceneObject asset goes through `ioc_train_service`, using the tool
  `register_using_ioc_train_service` (`UP …/ioc_train_service/tools/register_using_ioc_train_service.py:90-91,145-146,361`).
  - It accepts `--text_prompt`, `--confidence_threshold`, `--refinement_iters` and `--visibility_threshold`.
  - The train service takes the object's renderable or exact geometry and stores it as `/poseEstimators/<asset>/<target>.glb` in a PerceptionModel data asset (`…/ioc_train_service/…:101-104,231`).
  - The params proto is `IocPoseEstimatorParams { refinement_iters=1, confidence_threshold=2, visibility_threshold=3, text_prompt=4 }` (`…/ioc_train_service/proto/ioc_pose_estimator_params.proto:22-34`).
- **At request time** the estimator:
  - reads only `text_prompt` and `confidence_threshold` from those params (`util/asset_utils.py:50-91`); `refinement_iters` and `visibility_threshold` are not read anywhere in PKG (`rg`);
  - takes the first CAS `data_reference` with a `.glb/.gltf/.obj/.ply/.stl` extension, loads it with trimesh, applies `target_t_mesh`, and exports `/tmp/cad_models/<id>.ply` (`asset_utils.py:46-47,143-255`).
- **Caching:** the FoundationPose estimator is cached per `(dataset, obj_id, width, height)`
  (`service.py:312-456`). Because of `retain_pose_context=True` the context of one object stays
  resident, and it is released when the object or the image size changes (`PIPE inference/engine.py:64-67`).

## 3. Q2: inputs

- **Depth source.** The routing is `HARDWARE`, `STEREO` or `AUTO` (`proto:180-189`). The default
  is HARDWARE (`default_config.textproto:74-81`), which uses the depth image directly. The
  README's memory table marks FoundationStereo as "not loaded" in that mode (`README.md:331`), and
  `_init_foundation_stereo` returns early (`service.py:617-620`).
- **Depth units.** `uint16` depth is divided by 1000. `float32` depth is used as metres. Intensity
  images are expanded to 3 channels (`util/image_utils.py:210-288`). Our 32FC1 metric depth is the
  float32 case.
- **Pose and intrinsics.** Both the detection view and the depth view need a pose and intrinsics:
  - The pose is `w_t_c = s_img.world_t_sensor or s_img.camera_t_sensor` (`image_utils.py:250`).
  - `world_t_sensor` is None unless `world_t_camera` is set; otherwise `camera_t_sensor` is used (`UP client/v1/python/camera/data_classes.py:276-288`).
  - If either pose is None the request fails with "No depth available" (`service.py:997-1005`).
  - Depth is reprojected into the detection camera with `point_cloud_utils.reproject_depthmap_to_target_camera` (`service.py:956-1021`).
  - K must be float64 with fx and fy above 0 (`image_utils.py:159-191`).
- **Our camera sim:**
  - Sensor 1 is `color`, PIXEL_INTENSITY rgb8. Sensor 4 is `depth`, PIXEL_DEPTH 32FC1 float32. The resolution is 1280×720 (`P container/services/camera_sim.py:42,67-70`).
  - Both images set `camera_t_sensor` to identity and no world pose (`:219-233`), so the reprojection should be the identity. That is inferred and was not run.
  - RGB and depth come from one render product (`P sim/graphs.py:184-204`).
  - `capture_result_location` is UNIMPLEMENTED (`camera_sim.py:188`), so only inline `capture_result` requests work. The service supports those (`image_utils.py:210-288`).
- **Image size.** SAM 3 resizes input to 1008×1008 (`PIPE sam3/trt_processor.py:54-55`). The
  FoundationPose estimator is sized to the request image (`service.py:312-456`). Warmup uses
  1280×800 (`util/model_warmup.py:37-42`), so the first 1280×720 request creates a new estimator.
  That creation time is logged, and its length is unknown.
- **If stereo were wanted.** The STEREO path needs two intensity views with poses and intrinsics.
  It rectifies them, applies CLAHE, pre-shifts by the depth window, and runs inference
  (`service.py:760-954`). The sim's rsd455 asset has `Camera_OmniVision_OV9782_Left` at local
  (0, −0.0475, 0) and `_Right` at (0, +0.0475, 0), a 95 mm baseline, with focal length 1.93
  (`/mnt/c/isaac-sim-work/phase2/scripts/exp2a.log:476-500`; `P docs/designs/phases/phase_2_inventory.md:451-486`).
  Using them would need two more render products, sensors `"2"` and `"3"` with `camera_t_sensor`
  set, and the 24 GB tier. These are RGB cameras, not an IR projector pair, and FoundationStereo
  takes an RGB pair (NGC model description, `/tmp/p5up/ngc_fs.json`).

## 4. Q3: models

### 4.1 SAM 3 export (`PKG/scripts/prepare_sam3_models.sh`)

- **Access:** the Hugging Face repo `facebook/sam3` is gated with manual approval (HF API
  `gated: "manual"`, sha `3c879f39…`). The access form asks for name, date of birth, country,
  affiliation and job title. The script needs `HF_TOKEN`, `HUGGING_FACE_HUB_TOKEN` or
  `~/.cache/huggingface/token` (`:115-121`; `README.md:44-72`).
- **Input file:** `sam3.pt`, sha256 `9999e2341ceef5e136daa386eecb55cb414446a00ac2b55eb2dfd2f7c3cf8c9e`
  (`:50-56`), 3.45 GB (`PIPE README.md:138`). The SAM 3 code is pinned at `608457d816009fc369a02c692f27efdd7256b442`.
- **Python environment:** the script creates a venv in `/tmp/sam3_onnx_export_venv`, using uv
  or venv. It installs `setuptools<81`, `numpy<2`, `torch`, `torchvision`, `huggingface_hub`,
  `onnx`, `onnxscript`, `einops`, `hydra-core`, `omegaconf`, `opencv-python-headless`,
  `pycocotools`, `scipy`, `pyyaml`, `psutil` and `git+…/sam3@608457d` (`:126-169`). torch is not
  pinned in the script; PIPE's own pyproject pins `torch==2.10.0` from a cu128 index (`PIPE pyproject.toml:22-28,63-94`).
- **GPU:** optional. The script falls back to CPU (`:110-113`; `README.md:44-72`). How much RAM a
  CPU export needs is not documented.
- **Steps:** clone PIPE, download and check the sha, run `tools/export_sam3_to_onnx.py`, verify
  (`:171-216`). The export is FP32 only, with no FP16 conversion (`PIPE tools/export_sam3_to_onnx.py:14-19`).
- **Output** (3.3 GB in total, `README.md:44-72`):

  | File | Size |
  |---|---|
  | `sam3_vision_encoder.onnx` | 1.73 GB |
  | `sam3_text_encoder.onnx` | 1.35 GB |
  | `sam3_mask_decoder.onnx` | 111.3 MB |
  | `sam3_box_decoder.onnx` | 124.0 MB |
  | `bpe_simple_vocab_16e6.txt.gz` | 1.3 MB |

  The BPE vocabulary file counts as SAM Material (`README.md:71-72`).

### 4.2 FoundationPose and FoundationStereo ONNX

| File | sha256 (Bazel pin) | URL | Size |
|---|---|---|---|
| `refiner_net.onnx` | `4a4445d2506b1bfc0a8d62fa80f8780a7e37bb0df13bb09aa5dd04a03bf0c954` | NGC `nvidia/tao/foundationpose/versions/deployable_v1.0/files/refiner_net.onnx`; fallback `huggingface.co/nvidia/foundationpose/resolve/18d8309afc9790cddc03a1d50bc69954dc058693/refiner_net.onnx` | 68,158,609 B |
| `score_net.onnx` | `bb7208919557560ecb0e10a30612821f0025a2578190b17677bdf3653514a48d` | same two locations | 63,910,103 B |
| `deployable_foundation_stereo_s_dynamic_v2.0.onnx` | `a001a7bc0512a0bc3b3218194e924784e58b20656c6f1ea2c151024e555cfd64` | `https://api.ngc.nvidia.com/v2/models/nvidia/tao/foundationstereo/versions/deployable_foundation_stereo_s_dynamic_v2.0/files/deployable_foundation_stereo_s_dynamic_v2.0.onnx` | 346,531,163 B |

Sources are `UP bazel/non_module_deps.bzl:83-108` and `README.md:530-541`. The NGC file API returns
the same sha256 for the FoundationStereo file (`/tmp/p5up/ngc_fs.json`). For FoundationPose it
returns no sha256, so those two pins were not checked against NGC. The file sizes do match.

### 4.3 TensorRT engine compilation

- **When:** engines compile lazily on the first cache miss, normally during warmup at boot.
  Warmup runs FoundationPose, then FoundationStereo, then SAM 3, each in isolation with
  `_unload_models` between them, followed by a resident stage (`util/model_warmup.py:354-384`;
  `README.md:161-216`). In HARDWARE mode the FoundationStereo stage is skipped (`service.py:617-625`).
- **How long:** "tens of minutes" for seven engines, about 3.2 GB of plans (`README.md:161-216`).
  FoundationStereo builds a static profile on the first miss, which "can take a long time"
  (`PIPE README.md:231-234`). There is no figure for an RTX 3080.
- **Portability:** plans are not portable across GPUs or TensorRT versions. If `deserialize`
  returns null the service raises RuntimeError and does not rebuild. Exactly one plan per model
  should be kept. Prebuilt plans can be shipped in `engine_cache_bundle/` (`README.md:161-216`;
  `PIPE README.md:287-292`). A named `{name}.plan`, for example one built with trtexec, takes
  precedence over the cache (`PIPE README.md:294-307`; `PIPE models.py:46-56`).
- **Builder memory:** FPIL's builder workspace defaults to 8 GB (`FPIL include/foundation_pose_nvidia/config.hpp:61`).
  A batch of 252 "requires at least ~6.1 GB", and lower-memory GPUs should pair a smaller
  workspace with a smaller `batch_size` (`FPIL README.md:131-137`). `tensorrt_workspace_bytes`
  is not exposed by the Python `RuntimeConfig` (`FPIL python/src/foundation_pose_nvidia/core/config.py:26-41`)
  or by the C ABI (`rg workspace` in `c_api.h` finds only frame-workspace comments). The service
  therefore cannot change it.

### 4.4 GPU memory at runtime and the knobs that reduce it

- **Upstream figures:**
  - 16 GB for HARDWARE mode and 24 GB for STEREO or AUTO (`README.md:318-340`). No measurement method is given.
  - PIPE: "Start GPU sizing at 24 GB … 32 GB was tested" (`PIPE skills/foundationpose-setup/SKILL.md:31`).
  - PIPE: FoundationStereo's TensorRT scratch is "10-13 GB depending on the profile", and 30.1 GB was seen in use on a 32 GB card when stages were not released (`PIPE inference/depth.py:268-277`).
- **This machine:** RTX 3080 with 10240 MiB, compute capability 8.6, driver KMD 616.92, UMD
  615.71.08, CUDA UMD 13.4. 741 MiB was in use when queried (`nvidia-smi`). Isaac Sim shares the
  card. Our design gives Isaac at most 6 GB and perception 3.5 GB (`P docs/designs/robot_platform.md:313-319`).
- **Precision:** SAM 3 and FoundationStereo default to FP32 with TF32 (`PIPE trt_build.py:20,198-205`;
  `PIPE README.md:258-259`). FoundationPose defaults to `kTF32` (`FPIL config.hpp:65`). The service
  sets no precision (`service.py:312-456,570-655`). PIPE lists FP16 and BF16 as options.

Knobs that exist (their effect on our card was not measured):

| Knob | Where | Effect stated upstream |
|---|---|---|
| `sensor_routing.depth_source: DEPTH_SOURCE_HARDWARE` | config field 7 | does not load FoundationStereo; the 16 GB tier (`README.md:318-340`) |
| `foundation_pose.batch_size`, `n_hypotheses` | config field 5 | a smaller RefineNet batch reduces peak memory; ScoreNet always runs the full batch (`FPIL README.md:109-118`) |
| `sam3_refinement.policy: NONE` | config field 4 | skips the second, box-prompted pass; the box decoder loads lazily (`PIPE sam3/trt_processor.py:63-71`) |
| prebuilt or FP16 named plans | `engine_cache_bundle/` or `{name}.plan` | PIPE suggests an FP16 fixed 320×736 FoundationStereo ONNX (`PIPE skills/…/engine.md:17-19`) |
| `stereo.width` and `height` | config field 2 | only matters in stereo mode (`proto:84-89`; `PIPE stereo/depth.py:150-152`) |
| request image size | input | sizes the FoundationPose workspace (`FPIL c_api.h:218-219`) |

The `workspace_mb` option exists only in PIPE's build tools and is not wired into the service
(`PIPE trt_build.py:198-205`).

## 5. Q4: GPU in Docker on WSL2

**What this machine has** (read-only commands):

- Docker CE 29.9.2 runs natively in the WSL Ubuntu distribution: context `default`, a unix socket, OS Ubuntu 24.04.4.
- `docker info` lists only the `runc` runtime. `CDISpecDirs` is set, but `/etc/cdi` and `/var/run/cdi` do not exist.
- There is no `/etc/docker/daemon.json` and no `nvidia-ctk`.
- `/dev/dxg` exists, and `/usr/lib/wsl/lib` holds `libcuda.so*`, `libnvidia-ml.so*` and `nvidia-smi`.
- There are no `/dev/nvidia*` devices. glibc is 2.39, which meets PIPE's requirement of 2.38 or later (`PIPE README.md:31-40`).
- The driver meets FoundationPose's requirement of 580 or later (`PIPE README.md:55-56`; `nvidia-smi`).

**What NVIDIA documents:**

- NV-WSL §3.1: "Do not install any Linux display driver in WSL"; the Windows driver provides `libcuda` through `/usr/lib/wsl/lib`.
- NV-WSL §4: Docker is supported through the NVIDIA Container Toolkit, minimum v2.6.0 with libnvidia-container 1.5.1 or later, for both Docker CE and Docker Desktop.
- NV-WSL §5.1: only `--gpus all` is supported; pinned memory is limited and full Unified Memory is not supported.
- NV-CTK: add the apt repository from `nvidia.github.io/libnvidia-container`, install `nvidia-container-toolkit`, run `sudo nvidia-ctk runtime configure --runtime=docker` (which writes `/etc/docker/daemon.json`), then restart Docker.
- NV-CDI: with `nvidia-container-runtime.mode=auto`, WSL2 automatically selects JIT-CDI mode, which writes no persistent spec. Otherwise run `nvidia-ctk cdi generate --output=/var/run/cdi/nvidia.yaml` and use `--device nvidia.com/gpu=all`.

**What Microsoft documents:** MS-WSL says to install the Windows NVIDIA driver, then either
Docker Desktop or Docker Engine inside WSL. If you install the engine directly, you then install
the NVIDIA Container Toolkit. Its example is `docker run --gpus all …`. The page still names the
older `nvidia-docker2` package, so the NV-CTK steps are the current ones.

**Two gaps:**

- The README's Option B (plain runc with `/dev/nvidia*`) cannot work on WSL because those devices do not exist.
- A toolkit-free alternative would pass `--device /dev/dxg` and mount `/usr/lib/wsl/lib` read-only with `LD_LIBRARY_PATH` pointing at it. That comes from general knowledge of how libnvidia-container handles WSL. No document I fetched states it, and I did not test it.

## 6. Q5: licences

No legal interpretation is given here; this section only quotes or paraphrases the texts.

- **SAM 3, Meta "SAM License".** The LICENSE at sam3 `608457d` and at main is identical (sha256
  `4dea99bf…`, "Last Updated: November 19, 2025", 61 lines).
  - 1.a grants a non-exclusive, worldwide, non-transferable, royalty-free limited licence to use, reproduce, distribute, copy, create derivative works of and modify the SAM Materials. The text has no commercial-use exclusion and no user-count threshold.
  - 1.b.i: redistribution only under this Agreement, with a copy of it included.
  - 1.b.ii: acknowledge SAM in publications.
  - 1.b.iii and 1.b.v: comply with trade controls; no ITAR, military, warfare, nuclear, espionage or weapons uses.
  - 1.b.iv: no reverse engineering or decompiling. Whether exporting to ONNX or TensorRT touches this clause has not been assessed.
  - 3 and 4: disclaimer and limitation of liability.
  - 5.a: you own your derivatives. 5.b: suing over patents or IP ends the licence, and you indemnify Meta.
  - 6: Meta may terminate on breach. 7: California law.
  - 8: Meta may modify the agreement, effective immediately.
  - Discrepancies: PIPE calls the checkpoint "gated and non-redistributable" and notes that its metadata wrongly says MIT (`PIPE README.md:796-826`; `PIPE THIRD_PARTY_NOTICES.md:64-81`). Licence 1.b.i, however, allows redistribution under its terms. PIPE's export script is itself under the SAM License (`PIPE tools/export_sam3_to_onnx.py:10-13`).
- **FoundationStereo.** The NGC `nvidia/tao/foundationstereo` page says "This model is ready for
  commercial use" under the NVIDIA Open Model License (`/tmp/p5up/ngc_fs.json`). Version
  `deployable_foundation_stereo_s_dynamic_v2.0` was created 2025-11-21 and is FP32. The Hugging
  Face repo `nvidia/c-foundationstereo-s` (sha `d776ba85…`) is not gated and is tagged
  `nvidia-open-model-license`.
- **FoundationPose TAO `deployable_v1.0`.** The Hugging Face repo `nvidia/foundationpose`
  (sha `18d8309a…`) is not gated and uses the NVIDIA Open Model License. For the licence text,
  see `P docs/designs/phases/phase_5_inventory.md` §5.
- **TensorRT container.** The NGC repository description says that by pulling and using the
  container you accept the NVIDIA Software License Agreement and the Product-Specific Terms for
  AI Products (`/tmp/p5up/ngc_trt.json`). Relevant clauses:
  - The SLA ("Last Modified: May 07, 2026") says NVIDIA proprietary software is "licensed to run only on NVIDIA Platforms", forbids reverse engineering, and forbids distribution except as expressly granted (`/tmp/p5up/nvsla.html`).
  - The AI Product Terms ("Last Modified: April 15, 2026") §B.1 add 1.1.2.1 (offer the software as a service as part of a Customer Product) and 1.1.2.2 (sublicense and distribute it as part of a Customer Product).
  - §1.7 makes those grants conditional on an NVIDIA notice in distributed derivatives, and on reporting use when NVIDIA asks by email, no more than monthly.
  - §D.2 limits CUDA, TensorRT and cuDNN to NVIDIA platforms (`/tmp/p5up/pst.html`).
- **Also excluded upstream:** nvdiffrast, because of its NVIDIA Source Code License (`README.md:509-512`).

## 7. Q6: how intrinsic-omts wires the estimator

- **Pin and flags:** OMTS pins UPpin (`OMTS MODULE.bazel:14`) and sets both flags to True
  (`OMTS .bazelrc:93-95`).
- **Assets:** the solution bundles `nvidia_pose_estimator_service_asset` and the
  `ioc_train_service_asset` (`OMTS BUILD:65-66`). It declares the instance
  `intrinsic_asset_instance nvidia_pose_estimator_service` with no config override, so the
  defaults apply, plus `train_service` (`OMTS BUILD:98-99,232-242`).
- **Model assets:** run `bazel fetch`, then the SAM 3 prepare script with
  `--output-dir ~/.cache/intrinsic/nvidia_pose_estimator_models`, then build and install the three
  data assets with `--repo_env=NVIDIA_POSE_ESTIMATOR_MODELS_DIR` (`OMTS README.md:147-189`).
  `README.md:170` contains a doubled path, `intrinsic_perception/intrinsic_perception/…`.
- **Registration:** `register_using_ioc_train_service --scene_object_id=ai.intrinsic.raw_stock_2x3x5
  --pose_estimator_id=ai.intrinsic.raw_stock_2x3x5_estimator_text --refinement_iters=5
  --confidence_threshold=0.15 --visibility_threshold=0.15 --text_prompt="cuboid"`. A check with
  `ioc_pose_estimator:run_pose_estimation --service_name=nvidia_pose_estimator_service --camera_name=orbbec_camera`
  follows (`OMTS README.md:222-243`). The workpiece geometry is `models/raw_stock_2x3x5/base_visual.glb`.
- **Runtime:** `app_config.yaml` sets camera `orbbec_camera`, `perception_service_name:
  nvidia_pose_estimator_service`, the pose estimator id, and `sensor_ids: [1, 4]`
  (`OMTS configs/omts/app_config.yaml:33-41`).
  `src/hardware/vision.py:171-207` calls `capture_images(sensor_ids)` and then
  `estimate_pose_multi_view(perception, pose_estimator=PoseEstimatorId(id, package), capture_data=[…], min_num_instances)`,
  so it goes through the KV-store `capture_data_list` path.
- **Licensing notes:** `OMTS README.md:339-352`.

## 8. Not verified or unknown

- **Actual GPU memory on a 10 GB card**, in any mode. Using the GPU was forbidden, and the 16 GB
  and 24 GB figures are README statements with no stated method.
- **Engine compile times on an RTX 3080**, and whether the 8 GB FPIL builder workspace and a batch of 252 for ScoreNet fit next to Isaac Sim.
- **FP16 or BF16 accuracy and memory savings** for these models. Upstream only lists them as options.
- **Whether `pubsub.PubSub()` blocks, times out or raises** when no Zenoh router is reachable. The code only shows that an exception becomes a warning.
- **Whether the built image runs on WSL2** with the Container Toolkit. Not built and not run.
- **The image's inherited `ENTRYPOINT`** from the TensorRT base, and how the bundled Python 3.11 coexists with the base's Python. I did not inspect the image.
- **How the `/models` volume behaves outside k3s.** Only the `MODELS_DIR` path from the README is documented.
- **The new-estimator cost of the first 1280×720 request**, after warmup at 1280×800.
- **Whether the identity `camera_t_sensor` reprojection from our camera_sim gives correct
  results**, and whether `.obj` CAD files load. Both are inferred from code and were not run.
- **Intrinsics and render cost of the rsd455 Left/Right cameras.** Only their local transforms and focal length were read.
- **FoundationPose ONNX sha256 values against NGC.** NGC returned none; only the sizes match.
- **The full SLA, AI Product Terms and Open Model License texts.** I extracted only the clauses quoted above.
- **No legal interpretation of any licence**, including SAM clause 1.b.iv against ONNX and TensorRT export.
- **The toolkit-free `/dev/dxg` plus `/usr/lib/wsl/lib` container recipe.** It is general knowledge and was not tested.
- **RAM and time needed for the SAM 3 ONNX export on CPU.**
