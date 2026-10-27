# Phase 0 inventory: running six Intrinsic Core binaries without k3s

Factual inventory for [robot_platform.md](../robot_platform.md) (process
model and open question Q6). It covers `world_binary`, `geometry_service`,
`motion_planner_service_assetized_main`, `generic_icon_machine_server`,
`loopback_hardware_module_main` and `localcas`.

**Citation conventions.** Paths without a prefix are relative to
`/home/mew/intrinsic-core`. `project/` means `/home/mew/untitled-project/`.
`$EXT/` means
`/home/mew/.cache/bazel/_bazel_mew/c588f6cf2ea4110d87635daafacf38aa/external/`
(the Bazel external repositories of the build that produced `bazel-bin`).
"`--helpfull`" means the output of running the built binary with
`--helpfull`; that is the evidence for the list of transitively linked flags,
and each flag's source file is cited where it matters.

**Behaviour of the shared connection helpers** (used throughout below):

- `connect::kGrpcClientConnectDefaultTimeout` is 5 s
  (`intrinsic_sdk/intrinsic/connect/cc/grpc/channel.h:35-36`), and it is the
  default timeout of `WaitForChannelReady` (`channel.h:49-51`).
- `connect::CreateClientChannel` blocks until the channel connects or the
  deadline passes, then returns `DeadlineExceeded`
  (`intrinsic_sdk/intrinsic/connect/cc/grpc/channel.cc:206-224`, `:266-322`).
  Its health check counts an `UNIMPLEMENTED` reply as healthy
  (`channel.cc:94-96`), so servers without a gRPC Health service pass.
- `ConnectWithRuntimeAssetFallbackForAssetMigrationOnly`: if the
  `ResolvedDependency` lacks the requested interface, it dials
  `istio-ingressgateway.app-ingress.svc.cluster.local:80` and adds an
  `x-resource-instance-name` header (`intrinsic/assets/dependencies/utils.cc:51-52`,
  `:102-123`). The channel itself is created lazily
  (`intrinsic/assets/instances/connect/connect.cc:62-97`); failure shows up
  only in a following `WaitForChannelReady`.

---

## 1. Flags, address flags and config files per binary

### 1.1 `world_binary`

Source: `intrinsic/world/binary/world_binary.cc`.

| Flag | Default | Meaning | Source |
|---|---|---|---|
| `--port` | `10000` | Port for world services | `:66` |
| `--resource_registry_address` | `""` | ResourceRegistry address | `:67` |
| `--cas_service_address` | `""` | CAS address (geometry storage) | `:69` |
| `--worlds_path` | `""` | Directory that world files are loaded from and saved to | `:71` |
| `--data_logger_grpc_service_address` | `""` | DataLogger address | `:73` |
| `--grpc_client_connect_timeout` | 5 s (`kGrpcClientConnectDefaultTimeout`) | Used only for the DataLogger connect (`:200-201`) | `:76` |
| `--disable_asset_frame_edits` | `false` | Passed to `ObjectWorldService::Init` (`:278`) | `:79` |
| `--shutdown_grace_period` | `10s` | Shutdown timeout (`:311`) | `:82` |
| `--hss_service_address` | `""` | Hot shared state service (conductor) | `:85` |
| `--sim_service_address` | `""` | Simulation service (conductor) | `:87` |
| `--workcell_cluster_service` | `""` | Workcell cluster service (conductor) | `:88` |
| `--asset_deployment_service` | `""` | Asset deployment service (conductor) | `:90` |
| `--asset_instances_service` | `""` | Asset instances service (conductor) | `:92` |
| `--runtime_db_service_address` | `""` | Runtime DB (conductor) | `:94` |
| `--logger_address` | `""` | Declared; not read anywhere in `world_binary.cc` (only `FLAGS_grpc_client_connect_timeout` and the data-logger flag are used, `:197-201`) | `:96` |
| `--conductor_port` | `8082` | Second listening port for the conductor | `:97` |
| `--enable_sim_pause` | `false` | Conductor option (`:244`) | `:98` |

Transitively linked flags listed by `--helpfull` (non-abseil, non-gRPC):

| Flag | Default | Source |
|---|---|---|
| `--opencensus_metrics_port` | `0` | `intrinsic/stats/opencensus.cc:65` |
| `--opencensus_tracing` | `false` | `opencensus.cc:67` |
| `--otlp_tracing_endpoint` | `oc-agent.app-intrinsic-base.svc.cluster.local:4317` | `opencensus.cc:69` |
| `--zenoh_router` | `""` | `intrinsic_sdk/intrinsic/platform/pubsub/zenoh_util/zenoh_config.cc:23-24` |
| `--use_replicated_kv_store` | `false` | `intrinsic_sdk/intrinsic/platform/pubsub/kvstore.cc:53` |
| `--admin_set_proxy_endpoint` | `zenoh-router.app-intrinsic-base.svc.cluster.local:8081` | `kvstore.cc:55-56` |
| `--enable_on_change_pubsub` | `true` | `intrinsic/world/service/object_world_service.cc:130` |
| `--save_proto_worlds` | `true` | `intrinsic/world/service/world_storage.cc:57` |
| `--save_worlds_to_disk` | `true` | `world_storage.cc:60` |
| `--trace_dir` | `/tmp/icon_traces` (created if missing, per help text) | `intrinsic_sdk/intrinsic/util/thread/rt_trace_internal.cc:48` |
| `--sleep` | `false` (sleep forever instead of running) | `intrinsic_control/intrinsic/icon/release/portable/init_intrinsic.cc:33` |
| `--object_store_cache_size_mb` | `500` | `intrinsic_sdk/intrinsic/util/object_store/object_store_internal.cc:30` |
| `--object_store_cache_max_idle_seconds` | `-1` | `intrinsic_sdk/intrinsic/util/object_store/object_cache.cc:30` |
| `--chain_ik_solver` | `NewtonRaphson` | `intrinsic_kinematics/intrinsic/kinematics/ik/chain_inverse_kinematics.cc:39` |
| `--chain_ik_random_seed_solver_timeout_s` | `0` | `chain_inverse_kinematics.cc:37` |
| `--fan_out_joint_winding` | `false` | `intrinsic/utils/ik_helper.cc:35` |
| `--use_global_collision_checking_stats` | `true` | `intrinsic/world/collision/collision_checker_utils.cc:59` |
| `--threshold_for_slow_collision_checks` | `300us` | `collision_checker_utils.cc:64` |
| `--sdf_to_object_world_authoring_hints` | `false` | `intrinsic/world/objects/object_world_creation.cc:59` |
| `--always_deserialize_geometries` | `false` | `intrinsic/world/objects/world_object_internal.cc:100` |
| `--gzfile_tmpdir` | `""` | `intrinsic_sdk/intrinsic/world/gzfile/gzfile.cc:50` |

**Address flags and what happens when empty or unreachable.**

- `--resource_registry_address` (ResourceRegistry). Called as
  `CreateResourceRegistryClient(addr, absl::Seconds(60))` (`:207-210`). The
  60 s is the per-call timeout; the connection timeout is the third parameter,
  defaulting to 5 s (`intrinsic/resources/client/resource_registry_client.h:39-42`),
  not `--grpc_client_connect_timeout`. The result goes through `ASSIGN_OR_DIE`
  (`:207`), so an empty or unreachable address kills the process after about
  5 s (via `CreateClientChannel`, see the conventions above).
- `--cas_service_address` (CAS). `MakeCASServiceStub` creates a lazy channel
  (`intrinsic/geometry/storage/cas_client.cc:55-71`). Its retry policy is
  wait-for-ready, 300 s timeout, 5 attempts on `UNAVAILABLE`
  (`cas_client.cc:38-52`). `GetCasGeometryLibrary` only constructs a library
  object (`intrinsic/geometry/storage/cas_storage.cc:294-309`), so nothing is
  dialled at start-up; RPCs needing CAS wait on an empty or unreachable
  address.
- `--data_logger_grpc_service_address` (DataLogger). Skipped if empty; on
  failure only `LOG(ERROR)` (`:197-204`).
- Conductor addresses (`--hss_service_address`, `--sim_service_address`,
  `--workcell_cluster_service`, `--asset_deployment_service`,
  `--asset_instances_service`, `--runtime_db_service_address`).
  `ConductorImpl::Create` builds lazy `grpc::CreateChannel` channels for sim,
  workcell, asset deployment and asset instances
  (`intrinsic/conductor/conductor.cc:1218-1242`). `ResourceReader` and
  `ResourceWorld` use lazy Go `grpc.NewClient`
  (`intrinsic/conductor/resource_world_wrapper.go:63-72`, `:75-83`,
  `:110-131`). The save monitor also uses lazy channels
  (`intrinsic/conductor/save_scene_monitor.cc:74-82`); its polling loop logs a
  `WARNING` on every failed scan and keeps running
  (`save_scene_monitor.cc:134-141`). None of these is contacted
  synchronously at start-up.
- The conductor's own world address is the world server's `"[::]:<port>"`
  (`:123-124`, `:235`).
- `--zenoh_router`: see section 4. An unreachable router does not stop the
  process from starting.

**Conductor start-up dependency on the Zenoh KV store.** `ConductorImpl::Create`
opens `PubSub("conductor")` and its KV store (`conductor.cc:1249-1252`), then
polls `ExecutionContext` from the KV store for up to 1 minute. It retries only
`DeadlineExceeded`, `Unavailable` and `ResourceExhausted`
(`conductor.cc:1255-1271`). `NotFound` is treated as first deployment and
start-up continues (`conductor.cc:1281-1286`). Any other error is returned
(`:1276-1280`) and is fatal through `ASSIGN_OR_DIE` (`world_binary.cc:262`).
`KeyValueStore` `Get` starts with `NotFound` (`kvstore.cc:437`) and keeps it
when the Zenoh query finishes with no reply (`kvstore.cc:479-484`). It
returns `DeadlineExceeded` only if the query's on-done callback does not fire
within timeout + 1 s (`:478-481`), and `InternalError` if `imw_query` itself
fails (`:473-476`). A Zenoh router without a storage plugin therefore yields
`NotFound`. The k8s manifest notes that the world waits for the router so that
"key_not_found" is not returned falsely (`intrinsic/world/binary/world.yaml:33-36`).

**Other start-up facts.**

- World storage: if loading fails, the binary logs an error and continues
  with empty worlds (`:156-173`). An empty `--worlds_path` lists no files
  (`world_storage.cc:69-73`). Saving is gated by `--save_worlds_to_disk`
  (`world_storage.cc:433-436`, `:454-457`) and `--save_proto_worlds`
  (`:466`). Save paths are `JoinPath(worlds_path_, id + ".gzf" / ".pbbin")`
  (`world_storage.cc:515-523`), so with an empty `--worlds_path` they are
  relative to the working directory (inference from `JoinPath`). No directory
  creation call exists in `world_storage.cc` or `world_binary.cc` (rg for
  `CreateDir|mkdir|RecursivelyCreate` found none).
- `WorldUpdater::Create` only validates its inputs
  (`intrinsic/world/service/updater/world_updater.cc:184-207`).
- Two `WorldTfPublisher`s (worlds `world` and `sim_world`) publish over Zenoh
  (`:282-305`).
- Runtime context: **not read**. The binary has no RuntimeContext or
  `runtime_config.pb` code; all configuration is by flags.
- k8s reference arguments: `--port=8080`, CAS on `:9747`, resource registry on
  `:8080`, `--grpc_client_connect_timeout=5m`, `--opencensus_metrics_port=9101`,
  `--worlds_path=/mnt/worlds`, `--enable_on_change_pubsub`, the conductor
  flags and `--conductor_port=8082` (`world.yaml:38-60`). It runs as user
  65532 (`world.yaml:70-71`); services expose 8080, 9101 and 8082
  (`world.yaml:101-120`).

### 1.2 `geometry_service`

Source: `intrinsic/geometry/service/geometry_service.cc`,
`intrinsic/geometry/service/geometry_server.cc`.

| Flag | Default | Meaning | Source |
|---|---|---|---|
| `--port` | `10001` | gRPC port | `geometry_service.cc:28` |
| `--cas_service_address` | `""` | CAS address | `:29` |
| `--geometry_cas_cache_size` | `1000` | Geometry cache entries | `:31` |
| `--shutdown_grace_period` | `10s` | Shutdown timeout | `:33` |

Transitive flags (`--helpfull`): `--opencensus_metrics_port`,
`--opencensus_tracing`, `--otlp_tracing_endpoint`, `--sleep`,
`--object_store_cache_size_mb`, `--object_store_cache_max_idle_seconds`
(sources as in 1.1). There are **no Zenoh flags** (`zenoh_router` is absent
from its `--helpfull`).

- `--cas_service_address`: `MakeCASServiceStub` is lazy
  (`geometry_server.cc:74-75`), with the same retry policy as in 1.1. Nothing
  is dialled at start-up.
- Server: `"[::]:" + port` (`geometry_server.cc:84-89`). It registers
  `GeometryService` and long-running `Operations` (`:91-92`), with unlimited
  receive size (`:97`). If the server fails to build it returns
  `InternalError` (`:99-103`), which `main` turns fatal with `QCHECK_OK`
  (`geometry_service.cc:49`). OpenCensus plugin at `geometry_service.cc:38`.
- Runtime context: **not read**.
- k8s reference arguments: `--port=8080`, CAS, cache 1000, metrics on 9101
  (`intrinsic/geometry/service/geomservice.yaml:33-38`).

### 1.3 `motion_planner_service_assetized_main`

Source:
`intrinsic_motion_planning/intrinsic/motion_planning/service/motion_planner_service_assetized_main.cc`
(abbreviated `mp_main.cc` below).

| Flag | Default | Meaning | Source |
|---|---|---|---|
| `--world_service_address` | `world.app-intrinsic-base.svc.cluster.local:8080` | World service | `mp_main.cc:62` |
| `--geometry_service_address` | `geomservice.app-intrinsic-base.svc.cluster.local:8080` | Geometry service | `:65` |
| `--grpc_connect_timeout` | `5s` | Connect deadline for world, geometry and DataLogger | `:68` |
| `--cas_grpc_service_address` | `""` | CAS, used only by the non-volatile cache | `:71` |
| `--workcell_name` | `""` | Deprecated; a warning is logged if set (`:155-162`) | `:74` |
| `--organization_id` | `""` | Deprecated; as above | `:77` |
| `--runtime_context_file` | `/etc/intrinsic/runtime_config.pb` | Binary RuntimeContext | `:79` |

Transitive flags (`--helpfull`): `--opencensus_*`, `--otlp_tracing_endpoint`,
`--zenoh_router`, `--use_replicated_kv_store`, `--admin_set_proxy_endpoint`,
`--sleep`, `--object_store_cache_*`, `--chain_ik_*`, `--fan_out_joint_winding`,
`--use_global_collision_checking_stats`, `--threshold_for_slow_collision_checks`,
`--sdf_to_object_world_authoring_hints`, `--always_deserialize_geometries`,
`--gzfile_tmpdir` (sources as in 1.1), plus `--path_planner_graph_config_path`
(default `""`,
`intrinsic_motion_planning/intrinsic/motion_planning/path_planning/data_structures/path_planner_graph.cc:50`)
and `--pipeline_path_planner_config_path` (default `""`,
`.../path_planning/planners/pipeline.cc:38`).

**Start-up order and address handling.**

1. World and geometry stubs: `CreateClientChannel` with deadline
   `--grpc_connect_timeout`, through `ASSIGN_OR_DIE` (`mp_main.cc:89-99`;
   `motion_planner_service_utils.cc:144-175`). If either is unreachable the
   process dies after 5 s by default.
2. Runtime context: reads the binary RuntimeContext from
   `--runtime_context_file` and unpacks
   `intrinsic_proto.motion_planning.v1.MotionPlannerServiceConfig` from
   `context.config()`. Both are `ASSIGN_OR_DIE` (`mp_main.cc:100-108`).
3. The world pre-cache thread only warns on failure (`mp_main.cc:117-123`).
4. DataLogger: fallback connect, then `WaitForChannelReady` with
   `--grpc_connect_timeout`. Failure only warns (`mp_main.cc:127-150`).
5. Non-volatile cache: created only if
   `enable_motion_planner_service_nonvolatile_cache` is set; it dials
   `--cas_grpc_service_address` with a 5 s `CreateClientChannel` and is fatal
   (`mp_main.cc:195-202`;
   `.../service/nonvolatile_cache/motion_planner_nonvolatile_cache.cc:344-367`).
6. InstalledAssetsReader: fallback connect, then
   `QCHECK_OK(WaitForChannelReady(...))` with the default 5 s, which is fatal
   (`mp_main.cc:204-213`). It then calls `GetInstalledAsset` for
   `ai.intrinsic/motion_planner_service`, which must succeed
   (`mp_main.cc:219-221`; `.../service/motion_planner_service_asset_utils.cc:60-79`,
   `motion_planner_service_asset_utils.h:37`, `:44`).
7. gRPC server on `"[::]:" + context.port()` (`mp_main.cc:239-249`).
   `context.port()` is not validated, so port 0 means an OS-assigned port
   (gRPC semantics).
8. `MotionPlannerService` owns a `PubSub("motion_planning")`
   (`.../service/motion_planner_service.cc:143`, `:502`;
   `motion_planner_service.h:238-239`), so it opens a Zenoh session.

**Config proto.** `MotionPlannerServiceConfig`
(`intrinsic_apis/.../motion_planning/proto/v1/motion_planner_service.proto:113-134`).
Its `intrinsic_runtime` field declares the InstalledAssetsReader and
DataLogger dependencies (`:115-119`). `flags.parameterization_service_address`
is field 16 (`:90`), default `""`
(`.../motion_planner/motion_planner_flags.h:63`); its only consumer is
`.../service/motion_planner_service_in_process.cc:83`.

**RuntimeContext proto**
(`intrinsic_apis/intrinsic/resources/proto/runtime_context.proto:24-64`):
`port` = 1, `level` = 2 (`UNSPECIFIED`, `REALITY`, `PHYSICS_SIM`), `name` = 5,
`config` = 6 (`Any`), `http_port` = 7, `files` = 8. The planner does not read
`http_port`.

Existing project config: `project/container/config/motion_planner.runtime.txtpb:1-32`
sets port 18002 and the InstalledAssetsReader at `localhost:18010`
(`platform_stubs.py`), with no DataLogger interface, so step 4 falls back to
the ingress and warns after 5 s.

### 1.4 `localcas`

Source: `intrinsic/storage/onprem_cas/cmd/localcas.go` (Go `flag` package).

| Flag | Default | Meaning | Source |
|---|---|---|---|
| `--port` | `50051` | gRPC port | `localcas.go:42` |
| `--objects_dir` | `/tmp/onprem-cas` | Object store directory | `:43` |
| `--partial_dir` | `/tmp/onprem-cas-partial` | Partial uploads | `:44` |
| `--syncset_retention_days` | `30` | SyncSet retention | `:45` |

glog flags are also present (`--logtostderr`, `--alsologtostderr`,
`--log_dir`, `--v`, `--vmodule`, `--stderrthreshold`, `--log_backtrace_at`,
`--log_link`, `--logbuflevel`; `--helpfull`). `intrinsic.Init` sets
`logtostderr=true` and parses flags (`intrinsic/kubernetes/init.go:29-33`;
`intrinsic/production/init.go:28-37`).

- It has no address flags, no runtime context, no Zenoh and no metrics port.
- It creates both directories with `MkdirAll` (`localcas.go:57-62`), garbage
  collects partial files (`:72-86`), listens on `tcp ":<port>"` (`:89`), runs
  in local-only mode with the Sync API disabled (`:94`), and registers CAS
  plus gRPC reflection (`:96-100`).

### 1.5 `generic_icon_machine_server`

Source: `intrinsic_control/intrinsic/icon/server/icon_main.cc`
(abbreviated `icon_main.cc`).

| Flag | Default | Meaning | Source |
|---|---|---|---|
| `--config_pbtxt_file` | `""` | Plain `IconMainConfig` text proto; if empty, the runtime context is used | `icon_main.cc:63-64` |
| `--robot` | `"robot"` | Declared; no `FLAGS_robot` read anywhere in `intrinsic_control` (rg) | `:70` |
| `--server_name` | `"robot"` | ICON name (only with `--config_pbtxt_file`, `:152-153`) | `:74` |
| `--resource_id` | `""` | Resource id (only with `--config_pbtxt_file`, `:154-155`) | `:79` |
| `--non_realtime` | `false` | `kNonRealtime` mode (only with `--config_pbtxt_file`, `:157-163`) | `:83` |
| `--icon_server_port` | `0` | gRPC port; 0 means no listening port (only with `--config_pbtxt_file`, `:165`) | `:89` |
| `--shared_memory_namespace_testonly` | `""` | Subdirectory of `/tmp/intrinsic_icon` for HWM sockets (`:248-249`) | `:92` |
| `--grpc_client_connection_timeout` | `5s` | DataLogger connect wait (`:280-282`) | `:96` |
| `--resource_registry_service` | `istio-ingressgateway.app-ingress.svc.cluster.local:80` | ResourceRegistry for AutoGeneratedIconConfigService (`:246-247`, `:333-335`) | `:100-101` |
| `--use_runtime_asset_fallback` | `true` | Dial the Istio ingress for dependencies missing from `intrinsic_runtime` | `:108` |

Transitive flags (`--helpfull`): `--opencensus_*`, `--otlp_tracing_endpoint`,
`--zenoh_router`, `--use_replicated_kv_store`, `--admin_set_proxy_endpoint`,
`--trace_dir`, `--sleep`, `--object_store_cache_*`, `--chain_ik_*`,
`--fan_out_joint_winding`, `--use_global_collision_checking_stats`,
`--threshold_for_slow_collision_checks`, `--gzfile_tmpdir` (sources as in 1.1).

**Config file handling.**

- With `--config_pbtxt_file`: parses a **plain** `IconMainConfig` text proto
  (not `Any`-wrapped) through `GetTextProtoPortable` (`icon_main.cc:139-145`;
  `intrinsic/util/proto/get_text_proto.cc:69-91`). A parse error makes
  `main` return `EXIT_FAILURE` (`:230-232`, `:381-383`).
- Without it: reads the hard-coded `/etc/intrinsic/runtime_config.pb`
  (`:176-182`; there is no flag for this path). `port` must be non-zero
  (`:185-188`). It unpacks `intrinsic_proto.icon.IconMainConfig` from
  `context.config()`; an unpack failure is only logged here (`:190-195`).
  `server_name` and `resource_id` are set from `context.name` (`:197-199`).
  `REALITY` maps to `kNormal`, `PHYSICS_SIM` to `kNonRealtime`, and
  `UNSPECIFIED` is an error (`:201-218`). Any error becomes `NotFound` and
  the process exits with failure (`:236-243`, `:381-383`). In this path the
  flags `--non_realtime`, `--icon_server_port`, `--server_name` and
  `--resource_id` are not consulted.
- Plugin directory defaults to `/mnt/local-uploads`; it is used only if
  `plugin_loading_config` is set (`:124-137`).

**Address handling.**

- DataLogger (`grpc://intrinsic_proto.data_logger.DataLogger`): with
  fallback it dials the ingress; without fallback it connects only if the
  interface is present; failure is logged and ignored (`:261-295`).
- ObjectWorldService and GeometryService: created inside main-loop creation,
  see 3.4. The world address for AutoGeneratedIconConfigService comes from
  the `grpc://intrinsic_proto.world.ObjectWorldService` interface (`:321-330`).
- `--resource_registry_service`: used only per RPC by
  `AutoGeneratedIconConfigService`, which calls `CreateResourceRegistryClient`
  inside the handler
  (`intrinsic_control/intrinsic/icon/server/autogenerated_icon_config_service.cc:78-130`).
  Not needed at start-up.

**Process requirements.** `LockMemory<256K,256K>` (`mlockall`); failure makes
`Main` return an error and the process exit with failure (`:307-311`,
`:390-393`). The default health check service is enabled (`:387`). OpenCensus
plugin at `:371`.

### 1.6 `loopback_hardware_module_main`

The binary is generated by the `hardware_module_binary` macro, whose source
is `hal:hardware_module_main`
(`intrinsic_control/intrinsic/icon/hal/bzl/hardware_module_binary.bzl:39-77`;
`intrinsic_control/intrinsic/icon/hardware_modules/loopback/BUILD:78-81`).
Flag source: `intrinsic_control/intrinsic/icon/hal/hardware_module_main.cc`
(`hwm_main.cc`); helpers in `hardware_module_main_util.cc` (`hwm_util.cc`).

| Flag | Default | Meaning | Source |
|---|---|---|---|
| `--module_config_file` | `""` | Plain `HardwareModuleConfig` text proto; preferred over the runtime context | `hwm_main.cc:52` |
| `--realtime` | `false` | RT scheduling (only with `--module_config_file`) | `:54` |
| `--runtime_context_file` | `/etc/intrinsic/runtime_config.pb` | Binary RuntimeContext | `:56` |
| `--realtime_core` | unset | RT core if the config has no `realtime_cores`; else `/proc/cmdline` | `:59` |
| `--shared_memory_namespace_testonly` | `""` | Socket subdirectory; must match ICON's | `:62` |
| `--grpc_server_port` | unset | Health gRPC port, used only without a runtime context | `:65-66` |
| `--logger_connection_timeout` | `5m` | DataLogger wait; `0s` means no logger | `:70` |

Transitive flags (`--helpfull`): `--zenoh_router`, `--use_replicated_kv_store`,
`--admin_set_proxy_endpoint`, `--trace_dir`, `--sleep` (sources as in 1.1).
There are no OpenCensus flags.

**Start-up.**

1. `QCHECK_OK(LockMemory<256K,256K>)` runs first and is fatal
   (`hwm_main.cc:193-194`).
2. `LoadConfig` (`hwm_util.cc:146-175`). With `--module_config_file` it parses
   a plain `HardwareModuleConfig` text proto and RT scheduling follows
   `--realtime` (`:153-162`). Otherwise it reads the RuntimeContext and
   unpacks `intrinsic_proto.icon.HardwareModuleConfig` (`:104-110`). `name`
   comes from `context.name` if empty (`:112-118`), and `context_name` is
   always set (`:120`). `REALITY` means RT; `PHYSICS_SIM` and `UNSPECIFIED`
   mean non-RT (`:126-139`).
3. DataLogger (`hwm_util.cc:69-100`). If the timeout is ≤ 0 it returns
   `InvalidArgument` at once. Otherwise it always uses the fallback connect
   (there is no flag to disable it) and then `WaitForChannelReady(timeout)`.
   With the default, start-up blocks for up to **5 minutes** before a
   `WARNING` (`hwm_main.cc:96-108`).
4. `SharedMemoryManager::Create(namespace, module_config.name)`
   (`hwm_main.cc:135-139`).
5. `SetupRtScheduling` (`hwm_util.cc:177-225`). `drives_realtime_clock`
   creates a `RealtimeClock` (`:182-185`). In RT mode an affinity set is
   required (`:204-207`); in non-RT mode a missing affinity is ignored
   (`:222-224`).
6. gRPC health server: started only if there is a runtime-context port or
   `--grpc_server_port`, on `"[::]:<port>"` with `HardwareModuleHealthService`
   (`hwm_util.cc:298-348`). On a config or init failure the process **does not
   exit**; it goes into lame-duck mode and reports the error over that
   service (`:313-328`).

The inspection publisher (the only PubSub use found in the HWM runtime) runs
only if the RuntimeContext has `service_inspection_topic`
(`hwm_util.cc:282-290`;
`intrinsic_control/intrinsic/icon/hal/hardware_module_runtime.cc:1161-1180`).

**Loopback module.** `LoopbackConfig`
(`intrinsic_apis/intrinsic/icon/hardware_modules/loopback/loopback_config.proto:21-34`,
package `intrinsic_proto.icon`): `num_dof` (default 6),
`sensed_joint_position_noise_stddev_rad`, `advertise_system_limits_interface`.
`Init` advertises `joint_position_command`, `joint_position_state`,
`joint_velocity_state`, `joint_acceleration_state`, `payload_command`,
`payload_state`, `joint_system_limits` (optional) and `safety_status`
(`.../hardware_modules/loopback/loopback_hardware_module.cc:118-160`). If
`GetRealtimeClock()` is non-null it requires a control period and runs a
thread that ticks the clock and sleeps for one period (`:70-98`, `:163-189`).

---

## 2. Listening ports

| Process | Port | Fixed or random | Set by | Interface | Source |
|---|---|---|---|---|---|
| world | world services | fixed, default 10000 | `--port` | `[::]` (all) | `world_binary.cc:123-124`, `:229` |
| world | conductor (second port) | fixed, default 8082 | `--conductor_port` | `[::]` | `world_binary.cc:263-264` |
| geometry | geometry services | fixed, default 10001 | `--port` | `[::]` | `geometry_server.cc:84-89` |
| planner | planner services | from RuntimeContext `port` (0 means OS-assigned) | runtime context file | `[::]` | `mp_main.cc:239-241` |
| ICON | ICON gRPC | fixed; none if 0 (default) | `--icon_server_port` or RuntimeContext `port` | `0.0.0.0` | `icon_main.cc:313-317`, `:185` |
| HWM | health gRPC | fixed; none if unset | RuntimeContext `port` or `--grpc_server_port` | `[::]` | `hwm_util.cc:301-337` |
| HWM | Unix socket `/tmp/intrinsic_icon/<ns>/<module>.sock` plus `<module>.lock` | path | module name, namespace flag | filesystem | see 3.3 |
| localcas | CAS gRPC | fixed, default 50051 | `--port` | all (`":port"`) | `localcas.go:89` |
| world, geometry, planner, ICON | Prometheus metrics | off by default; fixed if > 0 | `--opencensus_metrics_port` | `0.0.0.0` | `opencensus.cc:234-238`, `:250`, `:285` |
| world, planner, ICON (any process opening a Zenoh session) | Zenoh peer listener | **random** (`tcp/0.0.0.0:0`) | environment only (below) | `0.0.0.0` | `zenoh_config.h:55-57` |
| zenohd (project entrypoint) | 7447 | fixed | `--listen tcp/127.0.0.1:7447` | 127.0.0.1 | `project/container/entrypoint.sh:8`, `:15` |

Notes:

- Both world ports belong to the same `grpc::ServerBuilder`
  (`world_binary.cc:227-273`), so both serve every registered service (world,
  ObjectWorld, compatibility, updater and conductor; `:250-265`). The project
  entrypoint does not pass `--conductor_port`, so world also listens on 8082
  (`project/container/entrypoint.sh:20-21`).
- The metrics exporter starts only if the port is > 0 or tracing is on
  (`opencensus.cc:234-238`); traces go to `--otlp_tracing_endpoint`
  (`:291-300`). The HWM and localcas have no exporter (no OpenCensus flags in
  their `--helpfull`).
- Zenoh listen endpoint selection (`zenoh_config.cc:100-120`): if
  `TEST_TMPDIR` or `PORTSERVER_ADDRESS` is set, there is no listener
  (`zenoh_helpers.cc:30-33`). If `ALLOWED_PUBSUB_IPv4` is set, it listens on
  `tcp/<ip>:0` (`zenoh_config.cc:105-107`). If `KUBERNETES_SERVICE_HOST` is
  set, there is no listener (`zenoh_helpers.cc:35-37`). Otherwise it uses the
  default `tcp/0.0.0.0:0`.
- Geometry and localcas open no Zenoh session (no Zenoh flags; 1.2, 1.4). The
  HWM links Zenoh but uses it only for the inspection topic (1.6).
- The k8s zenoh router listens on `tcp/0.0.0.0:7447` (Service ports 7447 and
  17447) (`intrinsic/platform/pubsub/fds_core.yaml:159`, `:346`, `:363`,
  `:378-379`). `--admin_set_proxy_endpoint` (`:8081`) is used only by
  `AdminSet` (`kvstore.cc:647`), not at start-up.

---

## 3. Running ICON and the loopback HWM standalone

### 3.1 Existing config files and their gaps

- `loopback/BUILD:20-26` exports `loopback_module_config.textproto` and
  `loopback_module_driven_config.textproto`; **neither file exists** in the
  checkout or in git history (the directory contains only `BUILD`,
  `loopback_hardware_module.cc` and `loopback_hardware_module.h`).
- `.../icon/testing/BUILD:18-30` exports
  `mainloop_loopback_hwm_integration_test_helper.*`; those files are also
  missing.
- Example HWM config:
  `.../icon/hardware_modules/kuka_rsi/loopback_hwm_config.textproto:19-29`.
  It is **`Any`-wrapped**: `name: "kuka_rsi_hal_module"`, `LoopbackConfig`
  with `num_dof: 6` and noise `1e-6`, `drives_realtime_clock: true`,
  `control_frequency_hz: 250`. It is the default config of
  `kr6_r900_2_loopback_hardware_module_service_node`
  (`kuka_rsi/BUILD:318-323`).
- Default ICON config:
  `.../icon/machines/common/icon_machine_default_main_config.textproto:19-23`.
  It is **`Any`-wrapped** and contains only
  `intrinsic_runtime { name: "intrinsic_runtime" }`, with no
  `control_frequency_hz`, so it fails the `> 0` check in 3.4.
- Full ICON example configs:
  `developer_resources/learn/tutorials/interactive_tutorials/icon/icon_introduction/icon_introduction.md:519-530`,
  `:600-606` (HalArmPart interfaces), `:678-821`; `robot_bringup.md:285-357`.
- `AutoGeneratedIconConfigService` builds the same structure in code: arm part
  `"arm"`, `HalArmPart`, `safety_action_type_name: "intrinsic.stop"`,
  `hardware_resource_name` set to the robot resource, `control_frequency_hz`
  from the registry or world, `world_service_from_grpc` with the world id,
  and kinematics and assembly only if the robot has an IK solver
  (`intrinsic_control/intrinsic/icon/server/auto_config_helpers.cc:298-320`,
  `:590-629`).

Because `--module_config_file` and `--config_pbtxt_file` take **plain**
messages (`hwm_util.cc:156-158`; `icon_main.cc:139-145`), the `Any`-wrapped
examples above must be unwrapped to the inner message to be used with those
flags.

### 3.2 Minimal configs derived from the code

The following are derived from the checks cited in 3.4; they are not files in
either repository. Field names were checked against
`intrinsic_apis/intrinsic/icon/hal/proto/hardware_module_config.proto:27-90`,
`intrinsic_apis/intrinsic/icon/server/config/icon_main_config.proto:73-133`,
`services_config.proto:32-63`,
`intrinsic_apis/intrinsic/icon/control/parts/proto/v1/realtime_part_config.proto:31-54`,
`intrinsic_apis/intrinsic/icon/control/parts/hal/arm_part/hal_arm_part_config.proto:30-122`
and `intrinsic_apis/.../hal/v1/hal_part_config.proto:22-27`.

HWM (`--module_config_file`, plain `intrinsic_proto.icon.HardwareModuleConfig`):

```textproto
name: "robot_hwm"            # ICON finds the module by this name (3.3)
module_config {
  [type.googleapis.com/intrinsic_proto.icon.LoopbackConfig] { num_dof: 6 }
}
drives_realtime_clock: true  # must agree with ICON's hardware_module_that_drives_clock
control_frequency_hz: 250    # must equal ICON's control_frequency_hz (within 1 µs of period)
```

ICON, start-up only (`--config_pbtxt_file`, plain
`intrinsic_proto.icon.IconMainConfig`). No services and no parts. Whether
`MainLoop::Create` accepts an empty `realtime_control_config` was **not
verified**.

```textproto
control_frequency_hz: 250
hardware_module_read_write_timeout_seconds: 10   # must be > 0 even in non-RT mode
hardware_module_names: "robot_hwm"
hardware_module_that_drives_clock: "robot_hwm"
```

ICON with a position-commanded arm. This needs the world service and a
kinematic object named `hardware_resource_name` in world `world` (3.5):

```textproto
control_frequency_hz: 250
hardware_module_read_write_timeout_seconds: 10
hardware_module_names: "robot_hwm"
hardware_module_that_drives_clock: "robot_hwm"
services {
  world_service_from_grpc { world_id: "world" }
  kinematics_from_world_service: true            # optional; snapshots robots at start (3.5)
}
realtime_control_config {
  parts_by_name {
    key: "arm"
    value {
      part_type_name: "HalArmPart"
      safety_action_type_name: "intrinsic.stop"
      hardware_resource_name: "<robot SceneObject name in world 'world'>"
      config {
        [type.googleapis.com/intrinsic_proto.icon.HalArmPartConfig] {
          joint_position_command { module_name: "robot_hwm" interface_name: "joint_position_command" }
          joint_position_state   { module_name: "robot_hwm" interface_name: "joint_position_state" }
          joint_velocity_state   { module_name: "robot_hwm" interface_name: "joint_velocity_state" }
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

The `interfaces` syntax matches the project's working planner config
(`project/container/config/motion_planner.runtime.txtpb:7-12`) and the
accessor chain `interfaces()` → `grpc().connection().address()` in
`icon_main.cc:323-328`. The interface names are those advertised by the
loopback module (`loopback_hardware_module.cc:118-160`).

Command lines:

```bash
loopback_hardware_module_main \
  --module_config_file=/path/loopback.hwm.txtpb \
  --logger_connection_timeout=0s \
  [--grpc_server_port=<M>]           # optional health/ServiceState port

generic_icon_machine_server \
  --config_pbtxt_file=/path/icon.txtpb \
  --non_realtime \
  --icon_server_port=<N> \
  --use_runtime_asset_fallback=false \
  --server_name=robot \
  [--zenoh_router=tcp/127.0.0.1:7447]
```

Why each flag matters:

- `--logger_connection_timeout=0s` avoids the 5-minute DataLogger wait (1.6
  step 3).
- `--icon_server_port` is needed because 0 means no port (1.5).
- `--use_runtime_asset_fallback=false` stops ICON from dialling the Istio
  ingress for geometry (3.4).
- `--non_realtime` selects normal scheduling and the 20 s read/write timeout
  (3.4).
- `--realtime` on the HWM stays at its default `false`.
- Both processes must use the same `--shared_memory_namespace_testonly`
  (default `""` on both).

### 3.3 How ICON and the HWM find each other

- There is no network discovery. The HWM's runtime creates a
  `DomainSocketServer` (`.../icon/hal/hardware_module_runtime.cc:730`), which
  creates the socket directory and takes a lock file
  (`.../interprocess/shared_memory_manager/domain_socket_server.cc:462-470`,
  `kLockSuffix ".lock"` at `:58`).
- The socket directory is `/tmp/intrinsic_icon` joined with the namespace
  (`domain_socket_utils.cc:47`, `:381-384`). The socket is
  `<dir>/<module name>.sock` (`domain_socket_utils.cc:97-110`), and the path
  must fit `sockaddr_un.sun_path` (`:49-58`). The directory is created
  recursively (`:112-122`).
- ICON connects per name in `hardware_module_names`
  (`.../icon/server/create_icon_main_loop.cc:312-317`) through
  `HardwareModuleProxy` (`.../icon/hal/hardware_module_proxy.cc:214`,
  `WaitForHardwareModule` at `:495-540`). It retries the socket connection
  every 1 s (`domain_socket_utils.cc:62-91`) for up to 120 s per module
  (`create_icon_main_loop.cc:268-273`).
- Shared memory segments are `memfd_create` + `ftruncate` + `mmap`
  `MAP_SHARED | MAP_LOCKED` + `mlock`
  (`shared_memory_manager.cc:169-215`; `memory_segment.cc:92-93`, `:104`).
  The file descriptors are passed with `SCM_RIGHTS`
  (`domain_socket_server.cc:217`; `domain_socket_utils.cc:255-258`). No
  `/dev/shm` or `shm_open` use was found in `intrinsic_sdk`, `intrinsic_control`
  or `intrinsic` (rg).
- Name matching: the HWM's `name` must appear in ICON's
  `hardware_module_names`, and in each part's `module_name`. With
  `--module_config_file`, `name` is not filled from a context, so it must be
  set in the file (`hwm_util.cc:153-162`).
- Control period: the HWM runtime always advertises a `control_period`
  interface initialised to 0 (`hardware_module_runtime.cc:808-813`;
  `.../hal/interfaces/control_period_utils.cc:27-31`). It sets the value only
  if the config has a rate (`hardware_module_runtime.cc:986-992`);
  `ModuleConfig::GetControlPeriod` returns `NotFound` if neither
  `control_frequency_hz` nor `control_period_ns` is set
  (`.../hal/module_config.cc:83-113`). ICON requires the reported period to
  equal `1 / control_frequency_hz` within 1 µs, otherwise it returns
  `FailedPrecondition` (`hardware_module_proxy.cc:64-112`). So both sides must
  set the same rate.
- Clock driver: `hardware_module_that_drives_clock` must be listed in the
  module set (`create_icon_main_loop.cc:304-311`). A module named as clock
  driver must expose the clock interface, and a module not named must not
  (`:330-353`). In other words, HWM `drives_realtime_clock` and ICON
  `hardware_module_that_drives_clock` must agree. If the field is empty, ICON
  uses its internal clock (`:275-278`).
- A module reporting `kInitFailed` makes main-loop creation fail with
  `Unavailable` (`:321-328`).

### 3.4 ICON main-loop creation checks (`create_icon_main_loop.cc`)

- `control_frequency_hz > 0` (`:142-146`).
- `hardware_module_read_write_timeout_seconds > 0` even in non-RT mode
  (`:148-153`); in non-RT mode the value used is `kNonRealtimeReadWriteTimeout`
  = 20 s (`:73`, `:212-216`).
- `kNormal` mode requires RT priority plus an affinity from
  `realtime_core_override` or `/proc/cmdline`, otherwise `InvalidArgument`
  (`:166-197`). Non-RT mode uses normal priority and ignores `hard_deadline`
  (`:198-210`).
- Service collection (`:361-485`), retried every 10 s for up to 180 s
  (`:74-75`, `:219-236`):
  - The world service is created only if `services.world_service_from_grpc`
    is set **and** (fallback is on **or** the ObjectWorldService interface is
    present) (`:371-373`).
  - Geometry is created if fallback is on **or** the GeometryService
    interface is present (`:402-403`).
  - Each waits 5 s with `WaitForChannelReady` (`:386`, `:416`).
  - Kinematics and assembly require a world service, otherwise
    `FailedPrecondition` (`:426-447`); dynamics requires kinematics
    (`:449-460`).
  - With the default `--use_runtime_asset_fallback=true`, geometry always
    dials `istio-ingressgateway.app-ingress.svc.cluster.local:80`
    (`utils.cc:51-52`, `:117`), so every attempt fails and creation fails
    after 180 s.
- Order: `GrpcEnvelope` calls the implementation factory synchronously in its
  constructor before `StartServer()`
  (`.../icon/server/grpc_envelope.cc:640-647`). The ICON gRPC port therefore
  opens only after main-loop creation finishes or fails (up to 180 s of
  service retries plus up to 120 s per HWM). On failure the envelope keeps
  serving and returns the error from RPCs; `ClearFaults` while in fatal fault
  re-runs the factory (`grpc_envelope.cc:238-256`, `:723-735`). The factory
  calls `CreateIconMainLoopFromProto` then `main_loop->Start()`
  (`.../icon/server/icon_main_loop_factory.cc:176-181`).
- The envelope registers ServiceState (health), IconApi, GPIO, Jogging,
  AutoGeneratedIconConfigService (if provided) and RealtimeTracing, with
  512 MB max receive (`grpc_envelope.cc:829-862`). It publishes robot status
  on a lazily created PubSub, topic `/icon/<name>/robot_status`
  (`grpc_envelope.cc:737-753`; `.../icon/common/topic_names.cc:39-43`).

### 3.5 ResourceRegistry and robot SceneObject at start-up

- ResourceRegistry: **not needed** at start-up. It is used only per RPC by
  `AutoGeneratedIconConfigService` (1.5).
- Robot SceneObject: needed only for certain part and service configs.
  - `HalArmPart` registers `JointLimits` only when `hardware_resource_name` is
    non-empty and a position, velocity or torque command is configured. It
    then needs a `WorldService` (otherwise `NotFound`) and
    `GetKinematicObject(hardware_resource_name)` to succeed
    (`.../control/parts/hal/arm_part/hal_arm_part.cc:366-451`, `:374-380`,
    `:384-386`).
  - `joint_position_command` requires `joint_position_state` and `JointLimits`
    (`hal_arm_part.cc:685-694`). A position-commanded arm therefore needs
    `world_service_from_grpc` plus a kinematic object of that name in the
    configured world.
  - Cartesian limits come from the world or from
    `mode_of_safe_operation_limits_config`; otherwise only an error is logged
    (`:522-567`). `kinematics_model_name` requires a KinematicsService
    (`:592-626`).
  - `KinematicsFromWorldService::Initialize` lists all world objects once and
    builds skeletons for every kinematic object; duplicate names are an error
    (`.../control/services/kinematics_from_world_service.cc:48-77`). Robots
    added to the world afterwards are not seen until the main loop is
    re-created.
- The world binary's two worlds are `world` (belief) and `sim_world`
  (`world_binary.cc:216`, `:282-299`).

### 3.6 How a client reaches ICON's gRPC API

- ICON listens on `0.0.0.0:<--icon_server_port>` (`icon_main.cc:313-317`).
- The Python client `Client.connect(grpc_host="localhost", grpc_port=8128)`
  uses `ConnectionParams.no_ingress`, so no routing header is sent
  (`intrinsic_control/intrinsic/icon/python/icon_api.py:168-197`).
  `for_solution` adds `x-resource-instance-name` (`icon_api.py:233-246`).
  8128 is only the client default; the server default is 0 (no port).
- Services exposed: see 3.4. The k8s manifest routes the IconApi, Jogging,
  GPIO and AutoGeneratedIconConfigService prefixes
  (`.../icon/machines/common/generic_icon_machine_manifest.textproto:37-40`).

---

## 4. Zenoh versions and protocol compatibility

| Item | Version | Evidence |
|---|---|---|
| zenoh-c Bazel module | `1.7.2.bcr.1` | `MODULE.bazel:570`; lock `MODULE.bazel.lock:1462-1463` |
| Patch | `static.patch` (switches `rust_shared_library` to `rust_static_library`) | `MODULE.bazel:983-987` |
| zenoh-c crate | `1.7.2` | `$EXT/zenoh-c+/Cargo.toml:14-17` |
| zenoh, zenoh-ext, zenoh-runtime, zenoh-util | `1.7.2`, git branch `release/1.7.2` | `$EXT/zenoh-c+/Cargo.toml:82-85`, `:92` |
| zenoh commit | `790faadb43c8f3da356bc5e3abc3ddc4d2eb41e6` | `$EXT/zenoh-c+/Cargo.lock:4138-4140`; zenoh-protocol `:4620-4622`; zenoh-transport `:4725-4727` |
| Protocol version linked | `0x09` | `$EXT/rules_rust++crate2+crates_zenoh_c__zenoh-protocol-1.7.2/src/lib.rs:31` |
| zenohd image (Bazel) | `zenohd:1.7.2`, digest `sha256:9a8195c9…db01d` | `MODULE.bazel:786-791` |
| zenohd image (k8s) | `zenohd:1.7.2`, run as `/zenohd -c /zenoh-router-config.json` | `intrinsic/platform/pubsub/fds_core.yaml:84`, `:99-104` |
| zenohd image (project) | same digest | `project/container/Dockerfile:3` |

**Compatibility evidence.** The transport rejects an `InitSyn` whose protocol
version differs from its own
(`$EXT/rules_rust++crate2+crates_zenoh_c__zenoh-transport-1.7.2/src/unicast/establishment/accept.rs:186-192`).
Both sides are zenoh 1.7.2 (crate version and image tag), and the linked
1.7.2 protocol constant is `0x09`. The zenohd binary inside the image was not
executed (the image is not in the local Docker store), so its version is
taken from the tag and digest only.

**Client session behaviour.**

- The generated peer config (`zenoh_config.cc:64-90`) sets: connect to the
  router endpoint; `mode: peer`; listen per section 2; multicast scouting
  off; gossip on, multihop off, autoconnect to peers and routers; shared
  memory off; `imw` introspection on.
- The default connect endpoint is
  `tcp/zenoh-router.app-intrinsic-base.svc.cluster.local:7447`
  (`zenoh_config.h:55-56`); `--zenoh_router` overrides it
  (`zenoh_config.cc:114-118`).
- A failed session open is fatal: `LOG(FATAL)` if `imw_init` fails
  (`intrinsic_sdk/intrinsic/platform/pubsub/zenoh_pubsub_data.cc:35-61`;
  `intrinsic_sdk/intrinsic/middleware/zenoh/imw_zenoh.cc:59-104`).
- In zenoh 1.7.2 peer mode, connect retry defaults are `timeout_ms = -1` and
  `exit_on_failure = false` for peers (`true` for clients)
  (`$EXT/rules_rust++crate2+crates_zenoh_c__zenoh-config-1.7.2/src/defaults.rs:38-48`).
  Listener `exit_on_failure` is `true` (`defaults.rs:56-57`). `start_peer`
  binds listeners and then connects to peers
  (`$EXT/rules_rust++crate2+crates_zenoh_c__zenoh-1.7.2/src/net/runtime/orchestrator.rs:178-231`).
  With `exit_on_failure` false, an unreachable endpoint gets a background
  connector (`orchestrator.rs:381-410`, `:709`). So an unreachable router
  does not make `z_open` fail, but a listener bind failure does.
- The k8s router has a filesystem storage backend rooted at
  `ZENOH_BACKEND_FS_ROOT=/data` (`fds_core.yaml:96`, `:113`, `:261-287`). The
  project's zenohd runs with only `--listen` and `--no-multicast-scouting`
  (`project/container/entrypoint.sh:15`), so KV `Get` returns `NotFound`
  (1.1) and nothing is persisted.

---

## 5. Other start-up requirements

**Memory locking and capabilities.**

- ICON and the HWM both call `LockMemory`: `mallopt` then
  `mlockall(MCL_CURRENT | MCL_FUTURE)`, then 256 KiB heap and stack prefault
  (`intrinsic/util/memory_lock.h:28-65`). If `mlockall` fails, ICON exits
  with failure (1.5) and the HWM aborts at `QCHECK_OK` (1.6). Shared memory
  segments are also `MAP_LOCKED` + `mlock` (3.3). This needs `CAP_IPC_LOCK`
  or a `RLIMIT_MEMLOCK` large enough for the whole process (`MCL_FUTURE`
  locks all future mappings).
- Manifests: ICON requests `SYS_NICE` and `IPC_LOCK` in both the real and sim
  specs, with `requires_rtpc_node` true for real and false for sim
  (`generic_icon_machine_manifest.textproto:41-55`, `:70-84`). The HWM
  template requests `SYS_NICE` + `IPC_LOCK` and `host_network: true` in the
  real spec and `IPC_LOCK` only in the sim spec
  (`.../icon/hal/bzl/hal_service_manifest.textproto.tmpl:29-56`, `:76-88`).
- RT mode (ICON `kNormal`, HWM `--realtime` or `REALITY`) also needs an
  affinity from config, flag or `/proc/cmdline` (3.4, 1.6). Non-RT mode does
  not.

**Directories.**

| Path | Used by | Created by the binary? | Source |
|---|---|---|---|
| `/tmp/intrinsic_icon[/<ns>]` | ICON + HWM sockets and locks; ICON `runtime_settings_dir` | yes (both sides call `CreateSocketDirectory`) | `domain_socket_utils.cc:286-299`; `domain_socket_server.cc:462-466`; `.../icon/server/runtime_options.h:48` |
| Same filesystem for ICON and the HWM | k8s mounts `/tmp/intrinsic_icon` as a hostPath in both | n/a | `generic_icon_machine_manifest.textproto:47`, `:65`, `:76`, `:94`; `hal_manifest.go:39` |
| `--objects_dir`, `--partial_dir` | localcas | yes (`MkdirAll`) | `localcas.go:57-62` |
| `--worlds_path` | world | no; empty means working directory (1.1) | `world_storage.cc:69-73`, `:515-523` |
| `--trace_dir` (`/tmp/icon_traces`) | world, ICON, HWM | yes (help text) | `rt_trace_internal.cc:48` |
| `/mnt/local-uploads` | ICON plugins | only if `plugin_loading_config` is set | `icon_main.cc:124-137` |
| `/etc/intrinsic/runtime_config.pb` | ICON (fixed path), planner and HWM (flag defaults) | must be provided if used | `icon_main.cc:176-177`; `mp_main.cc:79`; `hwm_main.cc:56` |

No `/dev/shm` requirement was found (3.3).

**Environment variables read.**

| Variable | Effect | Source |
|---|---|---|
| `KUBERNETES_SERVICE_HOST` | If set: no Zenoh listener | `zenoh_helpers.cc:35-37`; `zenoh_config.cc:108-110` |
| `ALLOWED_PUBSUB_IPv4` | Zenoh listener on `tcp/<ip>:0` instead of `0.0.0.0` | `zenoh_config.cc:105-107` |
| `TEST_TMPDIR`, `PORTSERVER_ADDRESS` | If set: treated as a test, no Zenoh listener | `zenoh_helpers.cc:30-33` |
| `LD_LIBRARY_PATH` | Project sets `/opt/intrinsic/lib` for staged solibs (below) | `project/container/Dockerfile:19` |
| `RUST_LOG`, `ZENOH_BACKEND_FS_ROOT` | zenohd logging and storage root (k8s) | `fds_core.yaml:87-96` |

**Shared libraries and CPU.**

- `ldd` on the built binaries shows that `world_binary`,
  `motion_planner_service_assetized_main` and `generic_icon_machine_server`
  load `librosidl_typesupport_introspection_c_identifier.so` and
  `librosidl_typesupport_introspection_cpp_identifier.so` from Bazel's
  `_solib_k8` directory. `geometry_service` and
  `loopback_hardware_module_main` load no Bazel-cache libraries. `localcas`
  is a dynamically linked Go binary using only system libc libraries
  (including `libresolv`). The project's staging script copies these
  (`project/container/stage_intrinsic.sh:1-30`).
- Builds use `common --config=haswell` (`--cpu=haswell`, `-mcrc32`), output
  in `bazel-out/haswell-opt` (`.bazelrc:215-218`). The host CPU must support
  the Haswell instruction set.

**Process and logging behaviour.**

- The C++ binaries call `InitIntrinsic`, which parses flags, forces the
  stderr threshold to INFO, installs the failure signal handler, restores
  default `SIGTERM`, initialises RT logging and honours `--sleep`
  (`intrinsic_control/intrinsic/icon/release/portable/init_intrinsic.cc`).
- ICON and the HWM install their own `SIGTERM` and `SIGINT` shutdown handlers
  (`icon_main.cc:301-305`; `hwm_main.cc:86-89`).
- Exit behaviour on dependency failure:

| Binary | Exits on | Keeps running on |
|---|---|---|
| world | ResourceRegistry unreachable (about 5 s); conductor KV error other than `NotFound` | CAS, DataLogger, conductor peers unreachable |
| geometry | server build failure | CAS unreachable |
| planner | world or geometry unreachable (`--grpc_connect_timeout`); RuntimeContext missing; InstalledAssetsReader unreachable or `GetInstalledAsset` failure; CAS unreachable when the non-volatile cache is enabled | DataLogger unreachable |
| ICON | config load failure; `mlockall` failure | main-loop creation failure (serves the error) |
| HWM | `mlockall` failure; `SharedMemoryManager` or `SetupRtScheduling` error | config or init failure (lame-duck mode) |
| localcas | directory creation or listen failure | — |

Sources: 1.1-1.6 above; `hwm_main.cc:134-148` for the HWM's
`INTR_ASSIGN_OR_RETURN` exits.

**Not checked.**

- The k8s zenoh router JSON config beyond the lines cited.
- Docker's default `RLIMIT_MEMLOCK` in the target environment.
- HWM behaviour with an empty `name` in `--module_config_file`.
- Whether `MainLoop::Create` accepts a config with no parts.
- The zenohd binary version inside the image (tag and digest only).
