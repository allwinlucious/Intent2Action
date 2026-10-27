# Phase 5 inventory: perception with Intrinsic's pose estimator

Facts only, gathered 2026-10-27 for phase 5 (design doc `DetectObjects`,
G7, Q4, GPU budget). Each claim cites a file and line, a command, or a URL.
Everything I could not verify is in §11.

Method: I read the local source and the public GitHub repositories with
`gh api`, queried NGC and Hugging Face metadata with `curl`, and streamed
one release tarball into memory (§4.1); nothing was extracted to disk.
I ran one `bazel query`, one `docker run --rm intrinsic-core:dev` and
`docker images` / `docker info`. I did not start Isaac Sim, use the GPU,
build anything or start a container. This file is the only repository
file written.

## 0. Reference prefixes

| Prefix | What it is |
|---|---|
| `IC:` | local `/home/mew/intrinsic-core`, HEAD `b2f9336` (one local fix commit on top of upstream `895281d`, committed 2026-09-24; `git log -3`) |
| `UP:` | upstream `github.com/intrinsic-ai/intrinsic-core` at commit `50927e35c5c8…` (2026-10-22). This is the commit intrinsic-omts pins (`OMTS:` `MODULE.bazel:14`). Its service list is the same as `main`'s (`gh api …/contents/intrinsic_perception/intrinsic/perception/service?ref=main`) |
| `OMTSold:` | `github.com/intrinsic-ai/intrinsic-omts` at `4ad9c68f35` (2026-10-16), the last commit before "Migrate perception pipeline to nvidia_pose_estimator_service (#161)" (`29635ca5f0`, 2026-10-22) |
| `OMTS:` | intrinsic-omts HEAD `bd814610b5` (2026-10-23) |
| `P:` | this repository |

## 1. Headline findings

1. **The local tree does not contain the model weights or the code that
   runs the models.** `IC:` holds the `ioc_pose_estimator` service: a
   Python gRPC front end that calls an Open Inference Protocol (OIP)
   server. It also holds the two Triton `config.pbtxt` files. The
   RF-DETR ONNX, the FoundationPose ONNX files, the Triton Python backend
   `model.py` and its `env.tar.gz` all came from the separate intrinsic-omts
   repository (`OMTSold:`, §4).
2. **Upstream has already replaced this pipeline.** As of 2026-10-22,
   `UP:` adds `nvidia_pose_estimator`, which runs SAM 3, FoundationStereo
   and FoundationPose directly on TensorRT 10.16 without Triton
   (`UP:` `…/nvidia_pose_estimator/README.md:18-26`). OMTS switched to it
   (`OMTS:` `BUILD:65,232-236`). `ioc_pose_estimator` still exists upstream
   (§2.6).
3. **Licences (Q4):**
   * FoundationPose weights on NGC `nvidia/tao/foundationpose:deployable_v1.0`
     and on Hugging Face `nvidia/foundationpose` are under the **NVIDIA Open
     Model License** and are described as "ready for commercial use" (§5.1).
   * The weights the old `ioc` path downloaded (`nvidia/isaac/foundationpose:1.0.0_onnx`)
     are under the **TAO Toolkit 21-08 Model EULA**. That EULA restricts
     distribution and limits use to systems with NVIDIA GPUs (§5.2).
   * The original NVlabs FoundationPose code is **non-commercial**. The
     weights used here are NVIDIA's re-releases, not NVlabs' (§5.3).
   * The RF-DETR bundle ships a bare Apache-2.0 LICENSE with no copyright
     holder or model provenance (§4.1).
4. **The RF-DETR segmentor is class-agnostic.** It returns boxes, scores,
   masks and visibility but no class labels. The service runs
   FoundationPose with the *requested* object's CAD on *every* mask and
   labels every result with the requested asset name (§3.3).
5. **No training pipeline exists for RF-DETR on custom objects** in any
   repository I examined. `ioc_train_service` trains nothing: it wraps a
   SceneObject's CAS mesh reference and three thresholds into a
   `PerceptionModel` data asset (§7).
6. **Hard-coded Kubernetes DNS names** sit in the pose estimator, train
   service, inference service, client utilities and the dependency-fallback
   helper (§8).
7. **No GPU memory figures exist** in any of the sources (§10). The design
   doc assumes FoundationPose runs in FP16. The old Triton backend instead
   runs the ONNX files through onnxruntime-gpu with float32 inputs (§10).
   Docker on this machine has no NVIDIA runtime configured (§10).

## 2. Where `ioc_pose_estimator` lives and how it is built

### 2.1 Location

* Service: `IC:intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator/`
  holds `BUILD`, `service_manifest.textproto`, `configs/default_config.textproto`,
  `proto/ioc_service_config.proto`, `model_files/{pose,segmentation}/config.pbtxt`,
  `service/{ioc_service_main,ioc_pose_estimator_service,pose_estimator_model,segmentation_model}.py`
  and `run_pose_estimation.py` (`find … | xargs wc -l`).
* Companion service: `…/service/ioc_train_service/` (§7).
* Client library: `…/service/python/pose_estimation_client_utils.py`,
  `pose_estimation_service_client.py`.
* Skill: `IC:intrinsic_perception/intrinsic/perception/skills/multi_view/estimate_pose_multi_view.py`.
* OIP server: `IC:intrinsic_inference/assets/inference_service/`
  (`inference_service_main.py`, `inference_service.py`,
  `model_assets_manager.py`) plus `IC:intrinsic_inference/core/`
  (`model_controller_triton.py`, `inference_runner.py`, `triton_shm_utils.py`).
* `incode/` holds no pose-estimator code. Its only use here is the
  `//incode/util/runfiles` dependency (`IC:…/ioc_pose_estimator/BUILD:32,53`).

### 2.2 Language and Bazel targets

All of it is Python. `bazel query … --output=label_kind` (run 2026-10-27) lists:

| Target | Kind | Source |
|---|---|---|
| `…/ioc_pose_estimator:segmentation_model` | py_library | `BUILD:24-43` |
| `…/ioc_pose_estimator:pose_estimator_model` | py_library | `BUILD:45-65` |
| `…/ioc_pose_estimator:ioc_pose_estimator_service` | py_library | `BUILD:67-100` |
| `…/ioc_pose_estimator:ioc_service_main` | py_binary | `BUILD:102-115` |
| `…/ioc_pose_estimator:ioc_pose_estimator_image` | oci_image (via `python_oci_image`) | `BUILD:117-120` |
| `…/ioc_pose_estimator:ioc_pose_estimator_service_asset` | intrinsic_service | `BUILD:122-135` |
| `…/ioc_pose_estimator:run_pose_estimation` | py_binary (CLI) | `BUILD:137-152` |
| `…/ioc_train_service:ioc_train_service_main` / `_image` / `_asset` | py_binary / oci_image / intrinsic_service | `ioc_train_service/BUILD:47-76` |
| `//intrinsic_inference/assets/inference_service:inference_service_main` | py_binary | `inference_service/BUILD:73-95` |
| `…:inference_service_image` | oci_image on `@distroless_python3` | `inference_service/BUILD:116-120` |
| `…:triton_inference_server_image` | oci_image on `@tritonserver-onprem` | `inference_service/BUILD:122-128` |
| `…:inference_service_asset` | intrinsic_service (both images) | `inference_service/BUILD:100-114` |

* The Python toolchain is 3.11 (`IC:MODULE.bazel:227-238`, `PYTHON_VERSION = "3.11"`).
* `python_oci_image` defaults to the base `//intrinsic/kubernetes:python3-image-oci`
  (`IC:bazel/python_oci_image.bzl:247-248`).
* The pip dependencies are `numpy`, `opencv_python_headless`, `trimesh`,
  `grpcio`, `protobuf`, `@open_inference_protocol` and
  `@triton-inference-server-common` (`BUILD:28-99`).
* The existing project image `intrinsic-core:dev` has Python 3.10.12, with
  `numpy` and `grpc` installed. It lacks `tritonclient`, `trimesh`, `cv2`,
  `onnxruntime`, `torch` and `tensorrt` (`docker run --rm --entrypoint sh
  intrinsic-core:dev -c 'python3 -c "…find_spec…"'`). The project
  `Dockerfile` installs only `grpcio` and `protobuf` via pip
  (`P:container/Dockerfile:8-9`).

### 2.3 Service manifest and runtime

* Asset id: `ai.intrinsic.ioc_pose_estimator_service`, described as
  "Image-based Object Center Pose Estimator Service Prototype"
  (`service_manifest.textproto:20-30`).
* It serves `/intrinsic_proto.perception.v1.PoseEstimationService/`
  (`:34`). Its config message is `intrinsic_proto.perception.ioc.IocPoseEstimatorServiceConfig`
  (`:35`). `real_spec` and `sim_spec` use the same image and set no GPU
  resource of their own (`:36-46`).
* `main` reads `/etc/intrinsic/runtime_config.pb` (flag
  `--runtime_context_file`, `ioc_service_main.py:40-47`). It unpacks the
  config from `RuntimeContext.config` (`:68-90`) and serves on
  `RuntimeContext.port`. Without a runtime context it falls back to
  `[::]:50051` (`:38,107-114`). The server is insecure, uses 10 worker
  threads and has no message-size limits (`:49-55,94-96`).
* Config (`proto/ioc_service_config.proto:24-43`):
  * `inference_service`: a ResolvedDependency that requires
    `grpc://inference.GRPCInferenceService`.
  * `segmentation_model` and `foundationpose_model`: ResolvedDependencies
    that require `data://intrinsic_proto.ml.inference_service.v1.MlModelAsset`.
  * `optional int32 batch_size`: "Defaults to 240 (full batch)".
* Default config (`configs/default_config.textproto:15-25`) binds the names
  `inference_service`, `ai.intrinsic.ioc_pose_estimation.segmentor.rfdetr`
  and `ai.intrinsic.ioc_pose_estimation.pose_estimator.foundationpose`.
  OMTSold overrode it with the same names plus `batch_size: 128`
  (`OMTSold:configs/common/pose_estimator_config.textproto:1-12`, used at
  `OMTSold:BUILD:244-249`).
* Startup hard-fails if either model dependency is missing
  (`ioc_pose_estimator_service.py:337-346`). It only warns if
  `inference_service` is missing (`:328-335`); the later model constructors
  then raise on `ml_service_stub is None`
  (`segmentation_model.py:178-179`, `pose_estimator_model.py:182-183`).
* The OIP channel is built from the resolved dependency's
  `interfaces[*].grpc.connection.address` plus its metadata headers
  (`ioc_pose_estimator_service.py:372-391`).

### 2.4 Triton server: version, flags, model repository

* Image: `nvcr.io/nvidia/tritonserver`, tag `26.07-py3`, `linux/amd64`,
  **pinned by tag only, with no digest** (`IC:MODULE.bazel:702-708`).
  Upstream has the same tag (`UP:MODULE.bazel:769-774`). A CPU-only
  validation image `gcr.io/giza-workcells/tritonserver-input-validation`
  is pinned by digest (`IC:MODULE.bazel:710-716`).
* The inference service runs as a pod of two containers:
  `inference_service_image` (the OIP front end) and
  `triton_inference_server_image` (`inference_service.manifest.textproto:36,49`).
* Triton arguments (`…manifest.textproto:65-80`, repeated for sim at
  `:127-142`):
  * `tritonserver --model-repository=/models --allow-client-shm=true`
  * `--allow-http=false --allow-grpc=true`
  * `--grpc-address=unix:///dev/shm/triton.sock --grpc-port=0`
  * `--metrics-port=8002 --model-control-mode=explicit`
  * OpenTelemetry trace flags pointing at
    `http://oc-agent.app-intrinsic-base.svc.cluster.local:4318/v1/traces`

  The design doc's planned Triton TCP ports 18030 (HTTP) and 18031 (gRPC)
  (`P:docs/designs/robot_platform.md:283-285`) do not exist in Intrinsic's
  setup. There, Triton is reachable only over a Unix socket, through the
  OIP front end.
* The Triton container requests `nvidia.com/gpu: 1` in both `real_spec`
  and `sim_spec` (`…manifest.textproto:59-63,121-125`). It shares
  `/models` (emptyDir) and `/dev/shm` (memory emptyDir) with the front end
  (`:38-46,83-94`).
* The front end connects to Triton at `unix:///dev/shm/triton.sock:0` and
  waits up to 1200 s for it (`inference_service_main.py:47-48,57,185-191`).
* The front end polls InstalledAssets every 5 s (`:55,146-151`) for data
  assets that provide `MlModelAsset` (`model_assets_manager.py:227-238`).
  For each one it downloads the `intcas://` references from CAS into
  `/models/<model_name>/<path>` (`:178-195`), writes `config.pbtxt` there
  (`model_controller_triton.py:144-146`) and calls Triton
  `RepositoryModelLoad` (`:108-109`).
* Each pose estimator wrapper polls `ModelReady` every 20 s for up to
  600 s before each inference (`segmentation_model.py:39-40,195-221`;
  `pose_estimator_model.py:43-44,200-226`).

The model repository layout, from the `config.pbtxt` files and the old
OMTS packaging:

```text
/models/
  ai.intrinsic.ioc_pose_estimation.segmentor.rfdetr/
    config.pbtxt            # backend onnxruntime, KIND_GPU x1
    1/segmentation.onnx
  ai.intrinsic.ioc_pose_estimation.pose_estimator.foundationpose/
    config.pbtxt            # backend python, KIND_GPU x1, EXECUTION_ENV_PATH=$$TRITON_MODEL_DIRECTORY/env.tar.gz
    env.tar.gz              # Python 3.12 wheels + foundationpose_cpp.so
    1/model.py
    1/foundationpose_refine.onnx
    1/foundationpose_score.onnx
```

Sources: `IC:…/model_files/segmentation/config.pbtxt:15-60`;
`IC:…/model_files/pose/config.pbtxt:15-89`;
`OMTSold:BUILD:370-380`; `OMTSold:src/foundationpose/BUILD:39-52`;
`OMTSold:third_party/foundationpose/build_defs.bzl:19-26` (Python 3.12
transition).

Model I/O (from the `config.pbtxt` files):

| Model | Inputs | Outputs |
|---|---|---|
| rfdetr | `input` UINT8 `[1,3,-1,-1]`; `thresholds` FP32 `[1,2]` (confidence, visibility) | `boxes` FP32 `[-1,4]`; `scores` FP32 `[-1]`; `masks` BOOL `[-1,-1,-1]`; `visibility` FP32 `[-1]` |
| foundationpose | `MASK` UINT8 `[-1,-1,-1]`; `RGB` UINT8 `[-1,-1,3]`; `DEPTH` FP32 `[-1,-1]`; `CAM_K` FP32 `[3,3]`; `CAD_MODEL_BYTES` UINT8 `[-1]`; optional `NUM_ITERATIONS`, `BATCH_SIZE` INT32 `[1]` | `ROTATION` FP32 `[-1,3,3]`; `TRANSLATION` FP32 `[-1,3]`; `CONFIDENCE` FP32 `[-1,1]` |

The `segmentation/config.pbtxt` in `IC:`, without its licence header, is
643 B, the same size as the one inside the release tarball. Its printed
text is the same (§4.1); I did not diff the two bytewise.

### 2.5 Packaging rule for models

`intrinsic_mlmodel` (`IC:intrinsic_inference/assets/inference_service/bazel/intrinsic_mlmodel.bzl:120-199`)
wraps a backend config plus a `{src: target_path}` map into an
`intrinsic_data` asset (MlModelAsset). Local files are bundled. Remote
`intcas://` URIs need `source_project` (`:155-167`). `IC:` uses it only
for a minimal test model (`IC:intrinsic_inference/assets/inference_service/bazel/BUILD:48`;
`rg "intrinsic_mlmodel\("`). The real RF-DETR and FoundationPose assets
were defined in OMTSold (§4).

### 2.6 Upstream differences in `ioc_pose_estimator`

At `UP:` the `segmentation_model.py` is identical to `IC:` (`diff -q`).
The differences are:

* `default_config.textproto` adds an `intrinsic_runtime` dependency.
* The service no longer hard-codes `INGRESS_ADDRESS`. It connects to
  InstalledAssets and Operations through
  `connect_with_runtime_asset_fallback_for_asset_migration_only` (diff of
  `service/ioc_pose_estimator_service.py`).
* BUILD labels move to `//intrinsic_sdk/...`.

`CAS_ADDRESS` is still hard-coded upstream; the diff does not remove it.

## 3. gRPC API

### 3.1 `PoseEstimationService` (`IC:intrinsic_apis/intrinsic/perception/proto/v1/pose_estimation_service.proto`)

* RPCs: `RunPoseEstimation(RunPoseEstimationRequest) → RunPoseEstimationResponse`
  and `DeletePoseEstimation(DeletePoseEstimationRequest) → Empty` (`:31-37`).
* `RunPoseEstimationRequest` (`:47-65`):
  * `oneof source_type { CaptureResult capture_result = 1; CaptureDataList capture_data_list = 2; }`
  * `intrinsic_proto.assets.Id asset_id = 3`: the pose estimator *data
    asset*, not the scene object.
  * `PoseEstimationRunConfig pose_estimation_run_config = 4`
* `PoseEstimationRunConfig` (`:75-108`) has `inference_parameters` (Any),
  `pose_priors`, `timeout`, `logging_context`, `suppress_logging`,
  `log_full_request`, `publish_annotated_image` and
  `check_camera_focal_length_and_principle_point`. **The ioc service reads
  none of these fields** (`rg pose_estimation_run_config` over
  `ioc_pose_estimator_service.py` finds no use; the request handling is at
  `:636-734`).
* `RunPoseEstimationResponse { repeated PoseEstimate pose_estimates = 1; }` (`:67-69`).
* `PoseEstimate { string id = 1; Pose camera_t_target = 2; double score = 3; }`.
  `score` is documented as "between 0..1" (`pose_estimate.proto:23-33`).
* `DeletePoseEstimationRequest { Id asset_id = 1; }` (`:71-73`). The ioc
  service only evicts its CAD and parameter caches
  (`ioc_pose_estimator_service.py:736-748`).

### 3.2 Input requirements, as implemented

* `asset_id.name` must be set; otherwise the call fails with
  `INVALID_ARGUMENT` (`ioc_pose_estimator_service.py:643-648`).
* Image source:
  * `capture_result`: used directly (`:183-186`).
  * `capture_data_list`: must hold exactly one entry. The service reads it
    from a Zenoh KV store with `pubsub.KeyValueStore(store).Get(key)`
    (`:172-182`; `IC:…/skills/multi_view/multi_view_pose_utils.py:214-252`).
* Exactly one `PIXEL_INTENSITY` image (H×W, H×W×1, ×3 or ×4; gray is
  repeated to 3 channels, RGBA is truncated) and its `intrinsic_matrix`.
  Exactly one `PIXEL_DEPTH` image (squeezed to 2-D float32)
  (`:200-268`). Depth is in metres (`pose_estimator_model.py:246`).
  Intrinsics come from the intensity sensor (`:231`). There is no check
  that depth and RGB are registered to each other.
* Missing RGB, depth or intrinsics → `INVALID_ARGUMENT` (`:255-268,664-668`).

### 3.3 Pipeline per call (`ioc_pose_estimator_service.py:636-734`)

1. `_ensure_asset_loaded(asset_id)` (cached per asset, `:393-545`):
   * `InstalledAssets.GetInstalledAsset` (`:424`), then unpack
     `deployment_data.data.data.data` as a `PerceptionModel` (`:436-442`).
   * Take the first `data_references` key ending in
     `.glb/.gltf/.obj/.ply` (`:450-460`) and fetch it from CAS (`:468`).
   * Require exactly one target (`:478-484`) and apply its
     `target_t_mesh` (`:486-516`).
   * Convert the mesh to OBJ with trimesh (`:505-531`).
   * Read `refinement_iters`, `confidence_threshold` and
     `visibility_threshold` from `inference_params`
     (`IocPoseEstimatorParams`). The defaults are 3, 0.6 and 0.6
     (`:67-69,77-150`).
2. RF-DETR on RGB (CHW, uint8) with the two thresholds (`:547-566`;
   `segmentation_model.py:264-306`).
3. FoundationPose on all masks in one OIP call: RGB, depth, masks, K, OBJ
   bytes, iterations and `batch_size` (default 240, `:70,592-607`).
4. Each result becomes `PoseEstimate(id=request.asset_id.name,
   score=CONFIDENCE, camera_t_target=…)` (`:619-630,728-734`).
   * Rotations are projected to SO(3) by SVD (`:273-287`).
   * `CONFIDENCE` is documented by the wrapper as a "Pose score /
     confidence logit … (higher/closer to 0 is better)"
     (`pose_estimator_model.py:268-269`). The API documents `score` as
     0..1 (§3.1). This mismatch is unresolved.
5. Visualisations:
   * Written to `/tmp/pose_estimation_debug/{detections,poses}.png` on
     every call (`:72,687-702`).
   * Published to PubSub topics `ioc_pose_estimator/detections` and
     `ioc_pose_estimator/poses` (`:73-74,704-726`).
   * `pubsub_ = pubsub.PubSub()` is a class attribute, so the Zenoh
     session is created when the class is defined (`:296`).

Consequence of the class-agnostic segmentor: in a scene with several
catalog objects, one `RunPoseEstimation` for the cube returns one
"cube" estimate per detected object of any type. Callers can tell them
apart only by score. The multi-view skill keeps the highest-scoring
estimate (`estimate_pose_multi_view.py:117-119`).

### 3.4 Skill and client path (not needed if `robot_service` calls gRPC directly)

* `estimate_pose_multi_view` has four camera slots
  (`estimate_pose_multi_view.py:51-52`). It requires `CameraConfig`
  equipment and a `perception` slot with capability
  `intrinsic_proto.perception.v1.PoseEstimationService`
  (`estimate_pose_multi_view_manifest.textproto:120-148`).
* The skill captures images itself, client-side through the camera
  clients (`pose_estimation_client_utils.py:287-323`), then calls the
  service.
* Client channels use the resource handle's address and the
  `x-resource-instance-name` header. Without connection info they fall back
  to the ingress address (`pose_estimation_client_utils.py:50,71-84`).
* CLIs: `run_pose_estimation.py` runs the skill on the executive
  (`run_pose_estimation.py:18-19,45-106`), and
  `register_using_ioc_train_service.py` is described in §7.

## 4. Models used by `ioc_pose_estimator`, and where the weights come from

Neither `IC:` nor `UP:` builds the two `ai.intrinsic.ioc_pose_estimation.*`
assets (`rg -i "ioc_pose_estimation|rfdetr|foundationpose"` over `IC:`
finds only the configs, the service code and a troubleshooting row in
`IC:developer_resources/learn/tutorials/getting_started.md:160`). OMTSold
built and installed both, together with `inference_service_asset` and
`ioc_pose_estimator_service_asset` (`OMTSold:BUILD:67-74,103-105`).

### 4.1 RF-DETR segmentor (`ai.intrinsic.ioc_pose_estimation.segmentor.rfdetr`)

* Source: the GitHub release asset
  `https://github.com/intrinsic-ai/intrinsic-omts/releases/download/20260922.0/segmentation.tar.gz`
  (120,804,747 B, sha256 `b872cd42…7283d`; `gh api …/releases/tags/20260922.0`).
  OMTSold pins it with the same checksum
  (`OMTSold:MODULE.bazel:180-187`). The README calls it "Pretrained
  RF-DETR segmentation model" (`OMTSold:README.md:139`).
* Packaging: `intrinsic_mlmodel(name="rfdetr_mlmodel", id="ai.intrinsic.ioc_pose_estimation.segmentor.rfdetr", model_files={segmentation.onnx: "1/segmentation.onnx"})`
  (`OMTSold:BUILD:356-380`).
* Contents (streamed and listed in memory with Python `tarfile`):

  | Member | Size | sha256 |
  |---|---|---|
  | `config.pbtxt` | 643 B | `864c056d…ad5e5` (same text as `IC:…/model_files/segmentation/config.pbtxt` minus its licence header) |
  | `segmentation.onnx` | 131,104,556 B | `399a866e…fda2ae` |
  | `LICENSE` | 11,358 B | `cfc7749b…3d30`: the plain Apache-2.0 text, with an unfilled `Copyright [yyyy] [name of copyright owner]` |

* The ONNX producer string is `pytorch`. A string scan of the first 4 KB
  and last 64 KB found no RF-DETR version, Roboflow reference,
  training-data note or class list.
* The model has a custom `visibility` output and a `thresholds` input
  (§2.4). Neither is part of a stock RF-DETR export (not verified, §11).
* The current OMTS still lists the release, but not the segmentation file
  (`OMTS:MODULE.bazel:190-235`).

### 4.2 FoundationPose (`ai.intrinsic.ioc_pose_estimation.pose_estimator.foundationpose`)

* Weights (`OMTSold:third_party/foundationpose/deps.bzl:32-48`):
  * `https://api.ngc.nvidia.com/v2/models/nvidia/isaac/foundationpose/versions/1.0.0_onnx/files/refine_model.onnx`
    (sha256 `06ad19f2…e094`)
  * `…/score_model.onnx` (sha256 `49a4f5f0…0be8`)

  NGC reports this version as created 2024-12-03. The latest version is
  `1.0.1_onnx` (`curl api.ngc.nvidia.com/v2/models/nvidia/isaac/foundationpose[/versions/1.0.0_onnx]`).
* Triton backend: the `python` backend with `model.py`
  (`OMTSold:third_party/foundationpose/model.py`, 546 lines).
  * Runs both ONNX files with onnxruntime using
    `["CUDAExecutionProvider", "CPUExecutionProvider"]` (`:53-71`).
  * Uses a CUDA rasteriser `foundationpose_cpp.RasterizeCudaContext(160, 160, 64)` (`:47`).
  * Builds float32 patches (`:123,232`).
  * The default `batch_size` is 240, chunked over candidates (`:378-439`).
* `foundationpose_cpp.so` is pybind11 + CUDA code compiled from
  `OMTSold:third_party/foundationpose/cpp/*` and from nvdiffrast sources
  taken from `NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation@9caca619…` with a
  patch (`OMTSold:third_party/foundationpose/BUILD:57-107`; `deps.bzl:20-30`).
  It is packed with Python 3.12 wheels from
  `third_party/foundationpose/requirements.in` (`flatbuffers 25.12.19`,
  `numpy 1.26.4`, `onnxruntime-gpu 1.29.0`, `opencv-python-headless 4.11.0.86`,
  `pillow 12.3.0`, `protobuf 7.35.1`, `scipy 1.17.1`, `trimesh 5.0.0`)
  into `env.tar.gz` (`build_defs.bzl:28-68`; `BUILD:109-113`).
* Packaging: `intrinsic_mlmodel(id="ai.intrinsic.ioc_pose_estimation.pose_estimator.foundationpose", …)`
  (`OMTSold:src/foundationpose/BUILD:39-52`).
* That code was deleted in OMTS #161. `OMTS:` has no `src/foundationpose/`
  or `third_party/foundationpose/` (tree listing of `OMTS:`).

### 4.3 Upstream replacement (`nvidia_pose_estimator`, for comparison)

* Models (`UP:…/nvidia_pose_estimator/README.md:18-20,121-125`):
  * **SAM 3** for zero-shot, prompted segmentation, with the default
    prompt `"object"` (`UP:…/configs/default_config.textproto:46-51`).
  * **FoundationStereo** for depth from the IR pair.
  * **FoundationPose**, with defaults `n_hypotheses 252`, `n_refine 5` and
    `batch_size 42` (`:59-64`).
* Weights (`UP:bazel/non_module_deps.bzl:78-108`):
  * `refiner_net.onnx` (sha256 `4a4445d2…c954`) and `score_net.onnx`
    (sha256 `bb720891…48d`) from
    `https://api.ngc.nvidia.com/v2/models/nvidia/tao/foundationpose/versions/deployable_v1.0/files/…`,
    with a fallback to `https://huggingface.co/nvidia/foundationpose/resolve/18d8309a…/…`.
  * `deployable_foundation_stereo_s_dynamic_v2.0.onnx` from NGC `nvidia/tao/foundationstereo`.
  * Downloads are gated behind `--…:download_weights=True`.
* The SAM 3 weights cannot be fetched by Bazel. The Hugging Face repo
  `facebook/sam3` is gated (`gated: "manual"`, `curl huggingface.co/api/models/facebook/sam3`).
  `scripts/prepare_sam3_models.sh` exports four ONNX graphs, 3.3 GB in
  total (`UP:README.md:44-72`; `UP:bazel/nvidia_pose_estimator_models.bzl:15-60`).
* Runtime: base image `nvcr.io/nvidia/tensorrt:26.05-py3` with
  TensorRT 10.16.1.11 (`UP:README.md:448-460`), built with
  `--…:build_full_image=True` (`:91-98`).
* Compiling the seven engines takes "tens of minutes" and produces about
  3.2 GB of plans. Plans are not portable across GPUs
  (`UP:README.md:161-216`).
* The upstream build deliberately excludes nvdiffrast because of its
  NVIDIA Source Code License (`UP:README.md:441-444`).
* `MODULE.bazel` marks it "TODO(b/572956581): Remove NVIDIA Pose Estimator
  out of insrc" (`UP:MODULE.bazel:670,678`; `UP:bazel/non_module_deps.bzl:21`).

## 5. Licences (Q4)

### 5.1 NVIDIA Open Model License (FoundationPose TAO `deployable_v1.0` and Hugging Face)

* The Hugging Face `nvidia/foundationpose` card says `license: other`,
  `license_name: nvidia-open-model-license`, "This model is ready for
  commercial use" (`curl …/api/models/nvidia/foundationpose`;
  `…/raw/main/README.md:2-4,27,31-33,265`). The repo is not gated, and its
  sha is `18d8309a…`, which matches the upstream fallback URL.
* NGC `nvidia/tao/foundationpose` (latest `deployable_v1.0`) has the same
  wording (`curl api.ngc.nvidia.com/v2/models/nvidia/tao/foundationpose`).
* License text (`https://www.nvidia.com/en-us/agreements/enterprise-software/nvidia-open-model-license/`,
  "Last Modified: October 24, 2025"):
  * "Models are commercially usable. You are free to create and
    distribute Derivative Models. NVIDIA does not claim ownership to any
    outputs."
  * Redistribution requires a "Notice" file with "Licensed by NVIDIA
    Corporation under the NVIDIA Open Model License".
  * Rights terminate if a "Guardrail" is bypassed or if you bring patent
    litigation.
  * NVIDIA "may update this Agreement".
* Training datasets listed on the card: LINEMOD (CC BY 4.0) and YCB-V
  (MIT) (`…README.md:143-145`).

### 5.2 TAO Toolkit 21-08 Model EULA (FoundationPose Isaac `1.0.0_onnx`, used by the old `ioc` path)

* NGC `nvidia/isaac/foundationpose`: "License to use this model is
  covered by the Model EULA … `tao_toolkit_21-08_models_eula.pdf`"
  (`curl api.ngc.nvidia.com/v2/models/nvidia/isaac/foundationpose`).
* EULA text (PDF parsed in memory):
  * The grant is "non-exclusive, non-transferable … (i) Install and use
    … (ii) Distribute SOFTWARE as incorporated in object code format into
    a software application".
  * "The SOFTWARE - are only distributable after trained using the
    NVIDIA TAO Toolkit … and the SOFTWARE identified as deployable."
  * "licensed for you to develop applications only for use in systems
    with NVIDIA GPUs."
  * The application "must have material additional functionality".
* **Discrepancy:** `OMTSold:README.md:322-331` states that these same
  `1.0.0_onnx` URLs are under the NVIDIA Open Model License. NGC's model
  card says the TAO Model EULA.

### 5.3 Code licences

* NVlabs/FoundationPose `LICENSE` §3.3: "The Work and any derivative
  works thereof only may be used or intended for use non-commercially …
  'non-commercially' means for research or evaluation purposes only"
  (`gh api repos/NVlabs/FoundationPose/contents/LICENSE`). The NVlabs
  weights are not used by either path above.
* NVlabs/nvdiffrast `LICENSE.txt` is also non-commercial (`:56-59`).
  The copy inside `isaac_ros_pose_estimation` carries Apache-2.0 headers
  (`…/gxf/foundationpose/nvdiffrast/common/common.cpp:1-12` at
  `9caca619`), and that repo is Apache-2.0 (`gh api repos/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation`).
* The OMTSold FoundationPose glue (`third_party/foundationpose/`) is
  declared Apache-2.0, with `package_version = "release-3.2"` of
  isaac_ros_pose_estimation (`OMTSold:third_party/foundationpose/BUILD:27-40`).
* intrinsic-core, intrinsic-omts and `foundationpose_perception_pipeline`
  are Apache-2.0 (`gh repo view intrinsic-ai/intrinsic-omts`;
  `UP:intrinsic/production/external/foundationpose_perception_pipeline.BUILD.bazel:33`).
* SAM 3 is under the Meta SAM License (`UP:README.md:46-47,71-72`). The
  Hugging Face API reports `license: other`. I did not read the SAM
  License text.

## 6. How objects are registered

Chain used by OMTS (`OMTSold:README.md:192-205`; `IC:…/ioc_train_service/tools/register_using_ioc_train_service.py`):

1. The object must already be an installed **SceneObject asset** whose
   geometry is in CAS. The tool fetches it with
   `InstalledAssets.GetInstalledAsset` and reads
   `deployment_data.scene_object.manifest.assets.scene_object_model`
   (`register_using_ioc_train_service.py:157-184,313-330`).
2. `TrainService.CreateTrainingJob`:
   * `pose_estimator_type` UNSPECIFIED or ONE_SHOT.
   * `inference_params` packed as `IocPoseEstimatorParams`.
   * At least one target with a `scene_object`.

   (`ioc_train_service.py:142-176`)
3. The train service:
   * Requires exactly one entity (`:70-74`).
   * Takes the first `intcas://` `renderable_ref` or `exact_geometry_ref`
     (`:88-95`).
   * Writes `data_references["/poseEstimators/<asset>/<target>.glb"] = <intcas URI>`
     — always the `.glb` suffix (`:211-244`).
   * Adds a target with `target_t_mesh` (identity unless given,
     `:228-262`) and copies `inference_params`/`params` (`:272-279`).
   * Keeps the result in an in-memory LRU of 100 jobs (`:280-286`).
4. `GetTrainingJob` always returns `done=True, progress=1.0` (`:292-316`).
5. `SaveTrainingJob` packs the `PerceptionModel` into a `DataAsset`
   (`asset_type = 5`, package defaults to `ai.intrinsic`) and calls
   `InstalledAssets.CreateInstalledAsset` with `UPDATE_POLICY_ADD_NEW_ONLY`
   (`:352-412`).
6. `RunPoseEstimation(asset_id=<that data asset>)` then loads it (§3.3).

* OMTS example: `--scene_object_id=ai.intrinsic.raw_stock_2x3x5
  --pose_estimator_id=ai.intrinsic.raw_stock_2x3x5_estimator
  --refinement_iters=3 --confidence_threshold=0.9 --visibility_threshold=0.85`
  (`OMTSold:README.md:197-205`).
* No images, renders or labels are involved. The only per-object input
  is the CAD mesh (any of `.glb/.gltf/.obj/.ply` at load time,
  `ioc_pose_estimator_service.py:450-460`).
* The train service manifest: `ai.intrinsic.ioc_train_service`, prefix
  `/intrinsic_proto.perception.v1.TrainService/`, config
  `google.protobuf.Empty` (`ioc_train_service/service_manifest.textproto:20-35`).
  The default port is 50052 (`ioc_train_service_main.py:37`).

`TrainService` API (`IC:intrinsic_apis/intrinsic/perception/proto/v1/train_service.proto:32-190`):
`CreateTrainingJob`, `SaveTrainingJob`, `ListTrainingJobs`,
`GetTrainingJob`, `DeleteTrainingJob`, `CancelTrainingJob` and
`VerifyTrainingJob`, plus the `PoseEstimatorType` enum (`EDGE_BASED`,
`SURFACE_BASED`, `SINGLE_VIEW_ML`, `MULTI_VIEW_ML`, `ONE_SHOT`). The ioc
implementation lacks `ListTrainingJobs`, `CancelTrainingJob` and
`VerifyTrainingJob` (`ioc_train_service.py` defines only
`CreateTrainingJob`, `GetTrainingJob`, `SaveTrainingJob` and
`DeleteTrainingJob`).

`PerceptionModel` (`perception_model.proto:27-40`) is
`oneof config { PoseEstimationConfig pose_estimation_config = 1; OneShotBaseModelConfig one_shot_base_model_config = 3; }`
plus `map<string, ReferencedData> data_references = 2`.
`PoseEstimationConfig` (`pose_estimation_config.proto:25-43`) holds
`targets`, `params`, `inference_params`, `intrinsic_params` and `origin`.

## 7. Training pipeline for RF-DETR on custom objects

**None found.**

* `ioc_train_service` does no training (§6). Its manifest says it
  "generates a PoseEstimator data asset" from a SceneObject and three
  parameters (`ioc_train_service/service_manifest.textproto:28`).
* No RF-DETR training code, dataset tooling or recipe exists in `IC:`,
  `UP:` (tree search for `rfdetr|rf-detr|segment` in paths), `OMTSold:`
  or `OMTS:` (tree listings; `gh api search/code?q=repo:intrinsic-ai/intrinsic-omts+rfdetr`
  returned nothing).
* The release bundle contains no training metadata (§4.1).
* The upstream direction avoids per-object training. SAM 3 is prompted
  with text (`prompt: "object"`) and refined with CAD box prompts
  (`UP:…/default_config.textproto:46-57`).

## 8. Hard-coded cluster DNS names and ports

| Name | Where (`IC:` unless noted) |
|---|---|
| `istio-ingressgateway.app-ingress.svc.cluster.local:80` | `ioc_pose_estimator_service.py:57,309` (InstalledAssets); `ioc_train_service.py:40,139`; `service/python/pose_estimation_client_utils.py:50,83,352`; `intrinsic/assets/dependencies/utils.py:32,107` (fallback, with `x-resource-instance-name: <runtime instance>`, `:85-112`) |
| `content-addressable-storage.app-intrinsic-base.svc.cluster.local:9747` | `ioc_pose_estimator_service.py:58-60,313` (CAD download); `inference_service_main.py:49-51,121-124` (model download). Still hard-coded upstream (§2.6) |
| `oc-agent.app-intrinsic-base.svc.cluster.local:4317` (OTLP gRPC) | `inference_service_main.py:54,70-73` |
| `http://oc-agent.app-intrinsic-base.svc.cluster.local:4318/v1/traces` | Triton args, `inference_service.manifest.textproto:76,138` |
| `unix:///dev/shm/triton.sock` | `inference_service_main.py:48`; manifest `:71,133` |
| `/etc/intrinsic/runtime_config.pb` | `ioc_service_main.py:40-47`; `ioc_train_service_main.py` (flag); `inference_service_main.py:46,60-62` (read unconditionally, with no fallback if absent) |
| `/models` (Triton repository) | `inference_service_main.py:47`; manifest `:67` |
| Default ports 50051 (pose estimator) and 50052 (train) | `ioc_service_main.py:38`; `ioc_train_service_main.py:37` |
| `/tmp/pose_estimation_debug/` (writes on every call) | `ioc_pose_estimator_service.py:72,687-702` |

## 9. Dependencies on other platform services

### 9.1 `ioc_pose_estimator`

| Dependency | Use | Source |
|---|---|---|
| `InstalledAssets` (through ingress) | `GetInstalledAsset(asset_id)` to read the `PerceptionModel` | `ioc_pose_estimator_service.py:309-312,424` |
| CAS | `client_helpers.get(uri)` for the CAD mesh | `:313-316,468` |
| OIP server (`inference.GRPCInferenceService`) | `ModelReady`, `ModelInfer` for both models | `:372-391`; `segmentation_model.py:209,288`; `pose_estimator_model.py:214,318` |
| PubSub / Zenoh | session created at import; two visualisation publishers; KV store reads for `capture_data_list` | `:296,356-359,172-182` |
| CameraService | **not called by the service.** Images arrive in the request. The multi-view skill and client utils capture from cameras client-side (§3.4) | `:153-270` |
| ObjectWorld | not used by the service. The skill updates object poses itself | `estimate_pose_multi_view.py:106-133` |

### 9.2 `inference_service`

| Dependency | Use | Source |
|---|---|---|
| `InstalledAssetsReader`, `DataAssets` | resolved through `intrinsic_runtime`, with the ingress fallback | `inference_service_main.py:92-120`; `v1/inference_service_config.proto` (`requires: grpc://intrinsic_proto.assets.v1.InstalledAssetsReader`, `grpc://intrinsic_proto.data.v1.DataAssets`) |
| CAS | model file download | `:121-124` |
| OTLP collector | tracing | `:54,70-73` |
| Triton | over the Unix socket | `:110-113` |

Each platform channel has a 10 s readiness timeout. On failure the server
keeps running in an ERROR state (`:56,175-196`).

### 9.3 `ioc_train_service`

`InstalledAssets` through ingress, for `CreateInstalledAsset`
(`ioc_train_service.py:136-140,384-393`).

## 10. GPU

* **Memory figures:** none appear in `IC:`, `UP:` or `OMTSold:`
  (`rg -i "memory|vram|gpu"` over the services, configs and READMEs).
* **GPU requests:** the only explicit request is `nvidia.com/gpu: 1` on
  the Triton container (§2.4). Both Triton models use `KIND_GPU`, count 1.
* **Levers on memory and latency:**
  * FoundationPose `batch_size`: 240 by default in the service and
    `model.py`, 128 in OMTSold (§2.3, §4.2).
  * `NUM_ITERATIONS`: default 3.
  * The rasteriser context is fixed at 160×160 (`model.py:47`).
* **Precision:** the old Triton backend feeds float32 tensors to
  onnxruntime-gpu (`model.py:53-71,123,232`). No FP16 conversion is
  visible. The design doc's "RF-DETR and FoundationPose in FP16, Triton
  ≤ 3.5 GB" (`P:docs/designs/robot_platform.md:309-313`) is not supported
  by any code found. The ONNX weights' internal precision was not checked
  (§11).
* **Upstream** keeps all seven TensorRT engines resident after warmup,
  about 3.2 GB of plans on disk (`UP:README.md:163-164,211-216`). The
  FoundationStereo input size "can be adjusted to suit … GPU memory"
  (`UP:…/proto/nvidia_service_config.proto:83-92`).
* **Hardware recommendation:** the Intrinsic docs recommend a "dedicated
  NVIDIA RTX 3060/4060+ required for ML/Vision workloads"
  (`IC:developer_resources/learn/tutorials/getting_started.md:15`), plus
  `intrinsic_runtime/setup_nvidia.sh` for k3s (`:99-102`).
* **This machine:**
  * RTX 3080 with 10 GB (`P:sim/run.log:63,435`).
  * WSL GPU plumbing is present: `/dev/dxg`, `/usr/lib/wsl/lib/libcuda.so*`,
    `libnvidia-ml.so.1`, `nvidia-smi`, driver library
    `libnvidia-gpucomp.so.615.71.08` (`ls`).
  * **Docker has only the `runc` runtimes, with no `nvidia` runtime and
    no CDI specs** (`docker info --format '{{json .Runtimes}}'`;
    `/etc/cdi` and `/var/run/cdi` absent; no `/etc/docker/daemon.json`;
    `which nvidia-ctk nvidia-container-cli` finds nothing).
  * As configured today, no container can reach the GPU with `--gpus`.
    The upstream README's "plain `runc` with host driver mounts" recipe is
    written for native Linux `/dev/nvidia*` devices
    (`UP:README.md:240-251`), which WSL does not have (WSL exposes
    `/dev/dxg`).
  * Phase 2 saw another application using 6.5 GB of GPU memory with Isaac
    not running (`P:docs/designs/phases/phase_2_inventory.md:40-44`).

## 11. Not verified / unknown

* **GPU memory use** of RF-DETR, FoundationPose (any batch size) or
  Triton itself. Not measured, because GPU use was forbidden for this task.
* **Triton 26.07-py3 details:** the Python version inside the image,
  whether it matches the Python 3.12 `env.tar.gz`, image size, and the
  onnxruntime backend version. The image was not pulled.
* **RF-DETR provenance:** base model (which RF-DETR size or segmentation
  variant), training data, class set (whether it is truly "any object"),
  how `visibility` is produced, and who holds copyright under the bundled
  Apache-2.0 file. None of this is recorded in the bundle or the repos
  examined.
* **Weight internals:** whether the FoundationPose `refine_model.onnx` /
  `score_model.onnx` (Isaac `1.0.0_onnx`) and `refiner_net.onnx` /
  `score_net.onnx` (TAO `deployable_v1.0`) are the same networks, and
  whether any of them are FP16 internally. I did not download them.
* **The full Isaac `1.0.0_onnx` licence position:** I read only the TAO
  21-08 EULA clauses quoted in §5.2. Whether those weights count as
  "deployable" under that EULA, and why OMTSold's README cites the Open
  Model License for them, is unresolved. I did not seek legal
  interpretation of any licence.
* **The SAM 3 licence text** (upstream path).
* **Missing model files:** `OMTSold:third_party/foundationpose/model.py`
  beyond the lines cited, and `foundationpose_binding.cpp`, were not read
  in full. Nor was the content of `segmentation.onnx` beyond producer
  strings.
* **Runtime behaviour without a cluster:** whether `ioc_pose_estimator`
  imports and starts without one. `pubsub.PubSub()` at class definition
  and InstalledAssets/CAS at construction suggest it needs Zenoh plus
  stand-ins for both. Not run.
* **Whether `inference_service` tolerates a missing runtime context:** it
  reads `/etc/intrinsic/runtime_config.pb` unconditionally
  (`inference_service_main.py:60-62,85`). Not run.
* **Whether the `IC:` targets build**, given `bazel build` was not
  allowed. `bazel query` resolves them (§2.2).
* **The `connection.ConnectionParams` header** formed from
  `server_instance` (`pose_estimation_client_utils.py:71-84`): its header
  name was inferred from the `x-resource-instance-name` convention in
  `dependencies/utils.py:108-110`, not read in `connection.py`.
* **The current OMTS README** (#171 documents the model DataAsset build
  steps) was not read.
