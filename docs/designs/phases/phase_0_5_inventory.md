# Phase 0.5 inventory: ROS 2 between Isaac Sim (Fast DDS, Windows) and the container (CycloneDDS, WSL)

Factual inventory for [robot_platform.md](../robot_platform.md) (Owner
Decisions D1 and D3, Open Question Q1, "Isaac Sim side", "Timing",
"Non-Functional Requirements: Security"). It covers what phase 0.5 needs to
make ROS 2 messages flow both ways between Isaac Sim 5.1's built-in ROS 2
bridge on Windows and CycloneDDS nodes in the Docker container on WSL2, and
to measure the round trip.

**Citation conventions.** `$ISAAC/` means `/mnt/c/isaac-sim/` (`C:\isaac-sim`).
`$BR/` means `$ISAAC/exts/isaacsim.ros2.bridge/`. `$EXT/` means Bazel's
external repository directory
`/home/mew/.cache/bazel/_bazel_mew/c588f6cf2ea4110d87635daafacf38aa/external/`
(intrinsic-core's output base; read only, nothing was built). `$RR/` means
`$EXT/com_github_mvukov_rules_ros2+/`, and `$X_<name>/` means
`$EXT/com_github_mvukov_rules_ros2++non_module_deps+<name>/`. `ic/` means
`/home/mew/intrinsic-core/`. `project/` means `/home/mew/untitled-project/`.
"Observed" means the output of a command run while writing this document. No
file other than this one was written, Bazel was not run, and the only
containers started were throwaway `docker run --rm` ones. "External
knowledge" marks claims taken from upstream documentation or source as
remembered, not checked against anything on this machine. Phase 0.5 must
confirm those.

---

## 0. Key findings

1.  **Isaac on Windows ships Fast DDS only.** Humble: Fast DDS 2.6.10, Fast CDR
    1.0.24, `rmw_fastrtps_cpp`. No Cyclone library exists in the Humble
    directory (§1.3).
2.  **A standalone script must set `ROS_DISTRO`, `RMW_IMPLEMENTATION` and
    `PATH` before `SimulationApp` starts.** Without them the bridge fails to
    start. With them, observed: the bridge starts, `rclpy` 3.3.17 loads
    from `$BR/humble/rclpy`, and `rclpy` reports `rmw_fastrtps_cpp` (§1.5).
3.  **Under WSL mirrored networking, multicast does not cross in both
    directions** (observed, §4.4). Windows → WSL multicast works on the LAN
    interface only. WSL → Windows multicast was not received on either
    interface, and multicast on loopback was not received in either
    direction. Unicast over `127.0.0.1` works both ways. **Discovery
    therefore needs unicast peers on both sides.** The first Q1 fallback
    is the only one that works here.
4.  **Windows and WSL share one UDP port space on loopback and on wildcard
    binds**, and `SO_REUSEADDR` does not share a port across the boundary
    (observed, §4.4). Expected consequence for DDS (not tested with DDS):
    whichever side binds the domain's multicast discovery port (7400 for
    domain 0) first blocks the other side. Multicast should be turned off
    on both sides. The unicast participant ports (7410, 7412, …)
    auto-increment across the boundary as if both sides were one host.
5.  **CycloneDDS 0.10.5 defaults to `ParticipantIndex=none`**, which its
    manual says "makes unicast discovery impossible". The container must
    set `ParticipantIndex=auto` (§4.1).
6.  **Cross-vendor ROS 2 *services* are at risk.** rmw_cyclonedds puts a
    16-byte request header (GUID + sequence number) into the CDR payload
    of service requests and replies (§5.3). This is the expected cause of
    the known incompatibility between rmw_cyclonedds and rmw_fastrtps for
    services (external knowledge). The design's `/sim/*` Trigger services
    and `isaacsim.ros2.sim_control`'s `simulation_interfaces` services
    cross this boundary. Topics (`JointState`, `Time`) are expected to
    interoperate.
7.  **Intrinsic's Bazel ROS 2 is Humble with CycloneDDS 0.10.5 and
    rmw_cyclonedds 1.3.4, and the RMW is fixed at link time**
    (`RMW_IMPLEMENTATION` is ignored). Apt offers CycloneDDS 0.10.5
    (same) and rmw_cyclonedds_cpp 1.3.5 (one patch newer) (§5.1, §6).
8.  The Hyper-V firewall for WSL is `DefaultInboundAction : Block`,
    `LoopbackEnabled : True` (observed, §4.5).

---

## 1. Isaac Sim 5.1's ROS 2 bridge on Windows

### 1.1 Version and extensions

- Isaac Sim build: `5.1.0-rc.19+release.26219.9c81211b.gl` (`$ISAAC/VERSION`).
- ROS 2 extensions in `$ISAAC/exts/`: `isaacsim.ros2.bridge`,
  `isaacsim.ros2.sim_control`, `isaacsim.ros2.tf_viewer`, `isaacsim.ros2.urdf`
  (observed: `ls $ISAAC/exts | grep -i ros`).
- `isaacsim.ros2.bridge` version `4.12.4` (`$BR/config/extension.toml:2`;
  `$BR/docs/CHANGELOG.md:3`, dated 2025-10-04).
- `isaacsim.ros2.sim_control`: "Extension that uses the ROS 2 Simulation
  Interfaces to control Isaac Sim"
  (`$ISAAC/exts/isaacsim.ros2.sim_control/config/extension.toml:5`).
- The bridge ships two distros, each with `lib/` (native DLLs) and `rclpy/`
  (Python packages): `$BR/humble/{lib,rclpy}` and `$BR/jazzy/{lib,rclpy}`
  (observed). Native bridge plugins: `$BR/bin/isaacsim.ros2.bridge.humble.dll`,
  `isaacsim.ros2.bridge.jazzy.dll`, `isaacsim.ros2.bridge.plugin.dll`, plus
  the loader check `isaacsim.ros2.bridge.check.exe` (observed: `ls $BR/bin`).

### 1.2 How the distro and libraries are selected

Settings (`$BR/config/extension.toml`):

| Setting | Default | Line |
|---|---|---|
| `exts."isaacsim.ros2.bridge".ros_distro` | `"system_default"` | 59 |
| `exts."isaacsim.ros2.bridge".publish_without_verification` | `false` ("Whether ROS 2 publishers are allowed to publish even if there is no active subscription for their topics") | 60-61 |
| `exts."isaacsim.ros2.bridge".publish_multithreading_disabled` | `false` | 63 |
| `exts."isaacsim.ros2.bridge".enable_nitros_bridge` | `false` | 65 |

Start-up logic (`$BR/isaacsim/ros2/bridge/impl/extension.py`):

1. `ros_distro = os.environ.get("ROS_DISTRO")` (line 52). If unset and the
   setting is `system_default`, Windows falls back to `"humble"` with the
   warning "Using 'humble' as default" (lines 53-69), then sets
   `os.environ["ROS_DISTRO"]` (line 73).
2. The distro must be in `SUPPORTED_ROS_DISTROS` = `{"22": "humble",
   "24": "jazzy"}` (`impl/ros2_common.py:28-31`), otherwise the extension
   disables itself (`extension.py:75-78`).
3. On Windows it appends only `<ext>/bin` to `PATH`, not `<ext>/humble/lib`
   (`extension.py:80-86`). It also extends `sys.path` from `PYTHONPATH`, a
   workaround for rclpy not being found.
4. It loads `isaacsim.ros2.bridge.plugin` from `<ext>/bin`
   (`extension.py:89-92`). It then runs `isaacsim.ros2.bridge.check.exe
   "<ext>/<distro>/lib/"` in a subprocess (`extension.py:136-152`). If that
   fails, it prints the environment instructions and disables the bridge
   (`extension.py:96-100`).
5. The rclpy shim (`$BR/humble/rclpy/__init__.py:31-74`) first tries a system
   `rclpy`. It then appends `$BR/humble/rclpy` to `sys.path` and calls
   `setup_ros2_environment`, which appends `<ext>/humble/lib` to `PATH`
   (`ros2_common.py:65-67`). **Bug:** `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
   is set only in the branch where `PATH` was empty (`ros2_common.py:68-70`),
   so in practice the shim does not set it.

Documented environment (printed by the extension on failure,
`ros2_common.py:96-105`; also observed in the failing run in §1.5):

```bat
set ROS_DISTRO=humble
set RMW_IMPLEMENTATION=rmw_fastrtps_cpp
set PATH=%PATH%;c:/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib
```

On Linux the same function also documents `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
(`ros2_common.py:89-93`). The Windows text documents Fast DDS only
(`ros2_common.py:96-105`).

Launchers:

- `isaac-sim.bat` calls `setup_ros_env.bat` unless `--no-ros-env` is passed
  (`$ISAAC/isaac-sim.bat:4-21`). `setup_ros_env.bat` sets `ROS_DISTRO=humble`
  and appends `exts\isaacsim.ros2.bridge\humble\lib` to `PATH` only if
  `ROS_DISTRO` was empty. It sets `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` if
  that was empty (`$ISAAC/setup_ros_env.bat:26-42`).
- **`python.bat` does not call `setup_ros_env.bat`.** It calls only
  `setup_python_env.bat`, which adds `%~dp0site` to `PYTHONPATH`
  (`$ISAAC/python.bat:20-22`; `$ISAAC/setup_python_env.bat:2`). It runs
  `kit\python\kit.exe` (`python.bat:30-41`).
- Environment variables set in WSL do not reach `cmd.exe` (observed: the
  probe printed `ROS_DISTRO= None RMW= None` at script start, §1.5).

### 1.3 RMW implementations, DDS and versions shipped

- `$BR/humble/lib` contains `rmw_fastrtps_cpp.dll`,
  `rmw_fastrtps_dynamic_cpp.dll`, `rmw_fastrtps_shared_cpp.dll`,
  `rmw_implementation.dll`, `fastrtps-2.6.dll`, `fastcdr-1.0.dll`, and **no
  Cyclone, Connext or Zenoh library** (observed: `ls $BR/humble/lib | grep
  -iE 'rmw|fast|cyclone|dds|zenoh|connext'`). Same for `$BR/jazzy/lib`
  (`fastrtps-2.14.dll`, `fastcdr-2.2.dll`, no Cyclone).
- The changelog confirms Cyclone support is Linux-only: "Isaac Sim now
  supports CycloneDDS (Linux ROS2 Humble only)" (`$BR/docs/CHANGELOG.md:711`)
  and "CycloneDDS is now supported for Jazzy" (`:71`).
- Versions (observed from the DLL version resource, `strings -el … |
  rg -A1 FileVersion`):

| Library | Version |
|---|---|
| `humble/lib/fastrtps-2.6.dll` | 2.6.10 |
| `humble/lib/fastcdr-1.0.dll` | 1.0.24 |
| `jazzy/lib/fastrtps-2.14.dll` | 2.14.5 |
| `jazzy/lib/fastcdr-2.2.dll` | 2.2.6 |

- The Humble libraries were built from `C:\dev\win\humble311\src\eProsima\Fast-DDS`
  (observed: build paths in `strings fastrtps-2.6.dll`).
- Bundled Python packages (egg-info names in `$BR/humble/rclpy/`):
  `rclpy-3.3.17-py3.11`, `sensor_msgs-4.9.0`, `builtin_interfaces-1.2.2`,
  `std_srvs-4.9.0`, `simulation_interfaces-1.1.0`, `rosgraph_msgs-1.2.2`,
  `geometry_msgs-4.9.0`, `rmw_dds_common-1.6.0`. **`control_msgs` is not
  bundled** (not in the listing). The design's `/gripper_command` uses
  `control_msgs/msg/GripperCommand` ("Isaac Sim side" table), so this
  matters for phase 2.

### 1.4 rclpy from a standalone script

- Python: 3.11.13, executable `C:\isaac-sim\kit\python\kit.exe` (observed).
  The extension modules are built for `cp311-win_amd64`
  (`$BR/isaacsim/ros2/bridge/_ros2_bridge.cp311-win_amd64.pyd`).
- Path: nothing to add by hand. After `enable_extension("isaacsim.ros2.bridge")`
  the shim appends `$BR/humble/rclpy` to `sys.path`
  (`$BR/humble/rclpy/__init__.py:52`). Observed `rclpy.__file__` =
  `c:\isaac-sim/exts/isaacsim.ros2.bridge\humble\rclpy\rclpy\__init__.py`.
- Import order used by NVIDIA's examples: create `SimulationApp`, call
  `enable_extension("isaacsim.ros2.bridge")`, call `simulation_app.update()`,
  and only then `import rclpy` ("Note that this is not the system level
  rclpy, but one compiled for omniverse",
  `$ISAAC/standalone_examples/api/isaacsim.ros2.bridge/clock.py:21-34`;
  same in `subscriber.py:18-35`).

### 1.5 Observed start-up (two probe runs)

Command: `cmd.exe /c "cd /d C:\isaac-sim && python.bat -"` with a script on
stdin and `SimulationApp({"headless": True, "extra_args": ["--/app/vulkan=false"]})`.

- **Run 1, no environment set:** log shows `Using 'humble' as default`,
  `Using backup internal ROS2 humble distro`, the `set ROS_DISTRO=…`
  instructions, then `[Error] [isaacsim.ros2.bridge.impl.extension] ROS2
  Bridge startup failed`.
- **Run 2, environment set inside the script before `SimulationApp`:**

  ```python
  os.environ["ROS_DISTRO"] = "humble"
  os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
  os.environ["PATH"] += r";C:\isaac-sim\exts\isaacsim.ros2.bridge\humble\lib"
  ```

  Output: `bridge enabled: True`, `Attempting to load system rclpy`,
  `Could not import system rclpy: No module named 'rclpy'`, `Attempting to
  load internal rclpy for ROS Distro: humble`, `rclpy loaded`, `rmw id
  rmw_fastrtps_cpp`. A `sensor_msgs/JointState` publisher was created
  (`/probe_js`). `builtin_interfaces/Time` imported.

**Documented steps for a standalone script**, from the sources above:
(1) set `ROS_DISTRO`, `RMW_IMPLEMENTATION` and `PATH`, plus the DDS
variables in §3, before `SimulationApp`; (2) create `SimulationApp`;
(3) call `enable_extension("isaacsim.ros2.bridge")`; (4) call
`simulation_app.update()`; (5) `import rclpy` if the script uses it.
Setting the variables inside the script works (Run 2). Setting them on the
`cmd.exe` line (`set …&& python.bat -`) should also work but was not tried.

---

## 2. Publishing and subscribing from a standalone script

### 2.1 OmniGraph nodes

All from `$BR/ogn/docs/*.rst`.

| Node (type id) | Inputs relevant here | Outputs |
|---|---|---|
| ROS2 Context (`isaacsim.ros2.bridge.ROS2Context`) | `domain_id` (uchar, default 0); `useDomainIDEnvVar` (bool, default **True**: use `ROS_DOMAIN_ID` if set) (`OgnROS2Context.rst:38-39`) | `context` (uint64) (`:48`) |
| ROS2 Publish Joint State (`…ROS2PublishJointState`) | `context`, `execIn`, `nodeNamespace`, `qosProfile` (string), `queueSize` (10), `targetPrim`, `timeStamp` (double, s), `topicName` (default `joint_states`) (`OgnROS2PublishJointState.rst:38-45`) | none |
| ROS2 Subscribe Joint State (`…ROS2SubscribeJointState`, version 2) | `context`, `execIn`, `nodeNamespace`, `qosProfile`, `queueSize` (10), `topicName` (default `joint_command`) (`OgnROS2SubscribeJointState.rst:38-43`) | `effortCommand`, `execOut` (fires when a new message is received), `jointNames`, `positionCommand`, `timeStamp` (double, s), `velocityCommand` (`:52-57`) |
| ROS2 Publisher (generic, `…ROS2Publisher`) | `context`, `execIn`, `messagePackage` (e.g. `geometry_msgs`), `messageName` (e.g. `WrenchStamped`), `messageSubfolder` (default `msg`), `nodeNamespace`, `qosProfile`, `queueSize`, `topicName` (`OgnROS2Publisher.rst:38-55`) | `execOut` (`:64`) |
| ROS2 Subscriber (generic, `…ROS2Subscriber`) | same as the generic publisher (`OgnROS2Subscriber.rst:38-55`) | `execOut` when a message is received (`:64`) |
| ROS2 Publish Clock (`…ROS2PublishClock`) | `context`, `execIn`, `qosProfile`, `queueSize`, `timeStamp`, `topicName` (default `clock`) (`OgnROS2PublishClock.rst:38-44`) | none |
| ROS2 QoS Profile (`…ROS2QoSProfile`) | see §2.2 | `qosProfile` (string, JSON) |

- In every node, `context` = 0 means "use the default global context"
  (e.g. `OgnROS2PublishJointState.rst:38`).
- The generic publisher and subscriber grow per-field attributes once
  `messagePackage` and `messageName` are set. The tests set those two and
  then write fields such as `inputs:data`
  (`$BR/isaacsim/ros2/bridge/tests/test_publisher.py:159-219`). The
  attribute names for nested fields (e.g. `header`, `wrench`) were not
  checked.
- `publish_without_verification` defaults to `false` (§1.2). Publishers
  therefore wait for a matched subscription. Across vendors this depends
  on discovery having matched the Cyclone reader.

### 2.2 QoS options

`ROS2QoSProfile` inputs (`$BR/ogn/docs/OgnROS2QoSProfile.rst:38-51`):

- `createProfile` presets: `Default for publishers/subscribers`, `Services`,
  `Sensor Data`, `System Default`, `Custom`.
- `history`: `systemDefault`, `keepLast` (default), `keepAll`, `unknown`.
- `depth`: default 10.
- `reliability`: `systemDefault`, `reliable` (default), `bestEffort`,
  `unknown`.
- `durability`: `systemDefault`, `transientLocal`, `volatile` (default),
  `unknown`.
- `liveliness`: `systemDefault` (default), `automatic`, `manualByTopic`,
  `unknown`.
- `deadline`, `lifespan`, `leaseDuration`: double, seconds, default 0.

Presets (`$BR/isaacsim/ros2/bridge/ogn/python/nodes/OgnROS2QoSProfile.py:61-89`):
Default and Services are keepLast/10/reliable/volatile. Sensor Data is
keepLast/5/bestEffort/volatile. Changing any policy input switches the
preset to `Custom` (`:41-44`).

The output is a JSON string with keys `history, depth, reliability,
durability, deadline, lifespan, liveliness, leaseDuration`
(`OgnROS2QoSProfile.py:102-113`). It is computed only on the node's first
frame (`state.firstFrame`, `:101-115`) and wired into a node's
`inputs:qosProfile`. When given, `qosProfile` overrides `queueSize`
(`OgnROS2PublishJointState.rst:42`). Writing the JSON string straight into
`inputs:qosProfile`, without a QoS node, should work because it is a plain
string input, but this was not tested.

The design asks for "reliable QoS, depth 1" in both directions
(`robot_platform.md`, "Timing", "Duplicates and QoS"). That is
`Custom`: keepLast, depth 1, reliable, volatile.

### 2.3 Ticking the graph from a script that steps physics itself

Three patterns ship with Isaac:

1. **On-demand graph + `evaluate_sync`.** Create the graph with
   `"pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND`
   and call `og.Controller.evaluate_sync(graph)` when needed
   (`standalone_examples/api/isaacsim.ros2.bridge/camera_manual.py:76,117`;
   `standalone_examples/api/isaacsim.core.api/omnigraph_triggers.py:103-123`).
   An existing graph can be switched with
   `graph.change_pipeline_stage(...ONDEMAND)` (`omnigraph_triggers.py:105-106`).
   Evaluation then happens inside the call, which suits the lockstep cycle
   in "Timing".
2. **`OnImpulseEvent`.** Wire `omni.graph.action.OnImpulseEvent.outputs:execOut`
   into the ROS nodes' `execIn`, set
   `/ActionGraph/OnImpulseEvent.state:enableImpulse = True` once per cycle,
   and step (`standalone_examples/api/isaacsim.ros2.bridge/moveit.py:76-134`,
   which uses PublishJointState, SubscribeJointState →
   `IsaacArticulationController`, PublishClock, and `ReadSimTime` for
   stamps). The graph then evaluates on the next app update, not inside
   the call.
3. **Physics callback.** `simulation_context.add_physics_callback(name,
   lambda dt: og.Controller.evaluate_sync(action_graph))` runs the graph
   on every physics step (`omnigraph_triggers.py:128`).

Stepping APIs used in these examples: `simulation_context.step(render=False)`
(physics only), `simulation_context.render()`, and `simulation_app.update()`
(`clock.py:95-118`; `omnigraph_triggers.py:140-150`). Stamps come from
`isaacsim.core.nodes.IsaacReadSimulationTime.outputs:simulationTime`
(`clock.py:44,56`; `moveit.py:81,96-97`). In-script `rclpy` is spun with
`rclpy.spin_once(node, timeout_sec=0.0)` after each step (`clock.py:106`;
`subscriber.py:66-67`).

`rclpy` from the script creates its own DDS participant, separate from the
bridge's. Each ROS2Context, and the default context, is its own
participant (external knowledge: in Humble `rmw_fastrtps` creates one
participant per context). Each one takes a unicast port pair (§3.4).

---

## 3. Fast DDS configuration on Windows (Fast DDS 2.6.10)

### 3.1 Recognised environment variables

Observed by `strings` on the shipped DLLs:

| Variable | Where | Meaning |
|---|---|---|
| `FASTRTPS_DEFAULT_PROFILES_FILE` | `fastrtps-2.6.dll` | Path of the XML profiles file |
| `SKIP_DEFAULT_XML_FILE` | `fastrtps-2.6.dll` | Skip loading `DEFAULT_FASTRTPS_PROFILES.xml` from the working directory (that file name is also in the DLL) |
| `FASTDDS_BUILTIN_TRANSPORTS` | `fastrtps-2.6.dll` | Built-in transport set. Accepted values (adjacent strings): `NONE`, `DEFAULT`, `DEFAULTv6`, `UDPv4`, `UDPv6`, `LARGE_DATA` |
| `FASTDDS_ENVIRONMENT_FILE`, `ROS_DISCOVERY_SERVER`, `ROS_SUPER_CLIENT` | `fastrtps-2.6.dll` | Not needed here |
| `RMW_FASTRTPS_USE_QOS_FROM_XML`, `RMW_FASTRTPS_PUBLICATION_MODE` (`SYNCHRONOUS`, `ASYNCHRONOUS`, `AUTO`; unknown values fall back to SYNCHRONOUS) | `rmw_fastrtps_shared_cpp.dll` | rmw-level |
| `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY`, `RMW_IMPLEMENTATION`, `ROS_SECURITY_*` | `rcl.dll` | rcl-level |

**`FASTDDS_DEFAULT_PROFILES_FILE` does not occur in `fastrtps-2.6.dll`**
(observed). Use `FASTRTPS_DEFAULT_PROFILES_FILE` with a Windows path, set
before `SimulationApp` (§1.5). `C:\isaac-sim` (the `python.bat` working
directory) contains no `DEFAULT_FASTRTPS_PROFILES.xml` (observed: `ls`).

`ROS_LOCALHOST_ONLY=1` with rmw_fastrtps (Humble) turns off built-in
transports and adds SHM plus UDPv4 with interface whitelist `127.0.0.1`
(external knowledge; the string `127.0.0.1` is present in
`rmw_fastrtps_shared_cpp.dll`). Discovery stays multicast. §4.4 shows
loopback multicast does not cross Windows ↔ WSL, so `ROS_LOCALHOST_ONLY`
alone is not enough.

### 3.2 XML elements present in this build

These element names occur in the XML parser's string table of
`fastrtps-2.6.dll` (observed): `profiles`, `transport_descriptors`,
`transport_descriptor`, `transport_id`, `type`, `interfaceWhiteList`,
`maxMessageSize`, `maxInitialPeersRange`, `sendBufferSize`,
`receiveBufferSize`, `participant` (`profile_name`, `is_default_profile`),
`rtps`, `userTransports`, `useBuiltinTransports`, `builtinTransports`,
`builtin`, `metatrafficUnicastLocatorList`, `metatrafficMulticastLocatorList`,
`initialPeersList`, `avoid_builtin_multicast`, `ignoreParticipantFlags`
(`FILTER_DIFFERENT_HOST`, `FILTER_DIFFERENT_PROCESS`, `FILTER_SAME_PROCESS`),
`port` (`portBase`, `domainIDGain`, `participantIDGain`, `offsetd0`…`offsetd3`),
`participantID`, `defaultUnicastLocatorList`.

- **Interface restriction syntax for 2.6:** `interfaceWhiteList` (camel
  case, "white list") inside a `transport_descriptor`, with `<address>`
  children. The newer `<interfaces><allowlist>` syntax does not appear in
  the DLL strings (`allowlist` not found). Whether 2.6.10 accepts interface
  *names* inside `interfaceWhiteList` is unknown; use addresses.

### 3.3 Candidate profile: UDPv4 only, loopback, unicast discovery

Assembled from the element names above. **Not parsed or run on this
machine.**

```xml
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_loopback</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>
      <maxInitialPeersRange>10</maxInitialPeersRange>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="isaac_loopback" is_default_profile="true">
    <rtps>
      <useBuiltinTransports>false</useBuiltinTransports>  <!-- drops SHM and the default UDP -->
      <userTransports><transport_id>udp_loopback</transport_id></userTransports>
      <builtin>
        <metatrafficUnicastLocatorList>
          <locator><udpv4><address>127.0.0.1</address></udpv4></locator>
        </metatrafficUnicastLocatorList>
        <initialPeersList>
          <locator><udpv4><address>127.0.0.1</address></udpv4></locator>
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>
```

The following are external knowledge, to be confirmed in phase 0.5:

- `useBuiltinTransports=false` plus `userTransports` is the 2.6 way to drop
  SHM. `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` is an alternative that needs no
  XML.
- When `initialPeersList` is set it replaces the default multicast initial
  peer.
- When `metatrafficUnicastLocatorList` is set and the multicast list is
  left empty, no SPDP multicast locator is added, so port 7400 is not
  bound.
- A locator without a port gets the well-known port for the participant
  ID.
- Default `maxInitialPeersRange` is 4: SPDP goes to participant IDs 0–3
  on each peer address. The container runs several Cyclone processes
  (`ros2_hwm`, Python nodes, grasp planner), and the Isaac process uses
  several participant IDs of its own, so 10 is suggested.

Check after start: `netstat -ano -p udp` on Windows shows no `:7400`
listener, and `127.0.0.1:741x` listeners for Isaac.

### 3.4 Port numbers (domain 0)

DDSI defaults, identical in both vendors: base 7400, domain gain 250,
participant gain 2, offsets d0 = 0 (multicast meta), d1 = 10 (unicast meta),
d2 = 1 (multicast data), d3 = 11 (unicast data). Cyclone's manual
(`$X_cyclonedds/docs/manual/options.md:167-220`) documents them. The Fast
DDS string table has the same fields (`portBase`, `domainIDGain`,
`participantIDGain`, `offsetd0`…`offsetd3`); the Fast DDS defaults are
external knowledge. So domain 0 uses 7400/7401 for multicast and
7410 + 2·i / 7411 + 2·i for participant i. None of these is in a Windows
excluded UDP range (observed: `netsh int ipv4 show excludedportrange
protocol=udp` lists 50000–50059, 54355–54454, 57836–58635 in blocks, and
65022–65121), and none is in WSL's ephemeral range 44620–48715 (§4.4).

---

## 4. CycloneDDS configuration (container) and WSL mirrored networking

### 4.1 Relevant Cyclone 0.10.5 options

From `$X_cyclonedds/docs/manual/options.md` (the 0.10.5 source that Bazel
fetched):

| Option | Default | Note | Line |
|---|---|---|---|
| `Discovery/ParticipantIndex` | **`none`** | "`none` … makes unicast discovery impossible"; `auto` picks a free index | 126-138 |
| `Discovery/MaxAutoParticipantIndex` | 9 | upper bound for `auto` | 118-124 |
| `Discovery/Peers/Peer[@Address]` | "" | host or IP, optional `:PORT`, "in addition to the default multicast address" | 141-158 |
| `General/AllowMulticast` | `default` (= `spdp` on Wi-Fi, `true` on wired) | `false` disables all multicast | 255-275 |
| `General/Interfaces/NetworkInterface` | – | attributes `address`, `name`, `multicast` (`default`/`true`/`false`), `priority`, `presence_required`, `autodetermine` | 344-402 |
| `General/MaxMessageSize` | 14720 B | | 405-415 |
| `General/FragmentSize` | 1344 B | | 328-336 |
| `Internal/SocketReceiveBufferSize` | – | attributes `min`, `max` | 886-894 |

Build flags in Bazel (`$RR/repositories/cyclonedds.BUILD.bazel`):
`enable_shm` is `False` by default (lines 9-12; no override in
`ic/.bazelrc`, observed by `rg`). Also `ENABLE_SECURITY=OFF`,
`ENABLE_SSL=OFF`, `ENABLE_TYPE_DISCOVERY=OFF`, `ENABLE_TOPIC_DISCOVERY=OFF`
(lines 99-106), and `ENABLE_IPV6=OFF` on one platform branch (line 118).

How rmw_cyclonedds 1.3.4 builds the configuration
(`$X_ros2_rmw_cyclonedds/rmw_cyclonedds_cpp/src/rmw_node.cpp:1052-1071`):
with `ROS_LOCALHOST_ONLY=1` it prepends
`<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="127.0.0.1"/>…`.
It then appends the contents of `CYCLONEDDS_URI`. `CYCLONEDDS_URI` may hold
inline XML or a `file://` path (external knowledge).

### 4.2 Candidate `CYCLONEDDS_URI` for the container

**Not run on this machine.**

```xml
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface address="127.0.0.1" multicast="false"/>
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>20</MaxAutoParticipantIndex>
      <Peers><Peer Address="127.0.0.1"/></Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

- `name="lo"` is also valid (`options.md:373-378`). The address form avoids
  any ambiguity, because WSL's `lo` also carries `10.255.255.254/32` (§4.3).
- WSL's `lo` has no `MULTICAST` flag (§4.3), so with `multicast="default"`
  Cyclone would treat it as not multicast-capable anyway
  (`options.md:366-370`).
- A peer without a port: Cyclone sends SPDP to the well-known unicast
  discovery ports of participant indices 0…`MaxAutoParticipantIndex`
  (external knowledge). Explicit ports (`127.0.0.1:7410`, `…:7412`) are the
  fallback.

### 4.3 WSL network state (observed)

- `.wslconfig` (`/mnt/c/Users/Allwin/.wslconfig`): `[wsl2] networkingMode=mirrored`,
  `[experimental] hostAddressLoopback=true`.
- `ip -br addr`: `lo 127.0.0.1/8 10.255.255.254/32 ::1/128`, `eth4 UP
  192.168.0.161/24` (LAN, default route via 192.168.0.1), `eth1 10.77.0.3/24`,
  `eth2 169.254.83.107/16`, a separate `loopback0 UP` with no IPv4 address,
  and `docker0 172.17.0.1/16` (down).
- `ip link show lo`: flags `<LOOPBACK,UP,LOWER_UP>`, **no `MULTICAST`**.
- Policy routing: rule priority 1, `from all ipproto udp lookup 127`, and
  table 127 has `127.0.0.1 via 169.254.73.152 dev loopback0`. So **every
  UDP/TCP packet to 127.0.0.1 leaves through `loopback0` to the Windows
  host**. Packets coming back in on `loopback0` are delivered locally
  (rule priority 0, `iif loopback0 ipproto udp lookup local`). There are no
  IPv4 multicast routes (`ip -4 route show table all | rg multicast` is
  empty).
- Docker: native engine inside this distro (`docker info`: `Ubuntu 24.04.4
  LTS | 6.6.114.1-microsoft-standard-WSL2`; context `unix:///var/run/docker.sock`).
  `project/container/run.sh:5` uses `--network host`, so the container shares
  WSL's network namespace and everything in §4.4 applies to it unchanged.
- Socket buffers: `net.core.rmem_max = rmem_default = wmem_max = 212992`.
  Camera frames are about 1.2 MB (RGB 848×480×3) and 1.6 MB (32FC1), so
  Cyclone's receive buffer request will be capped at 208 KiB unless the
  limit is raised. Raising it is a host sysctl, outside the container.
- WSL ephemeral port range: `44620 48715` (`/proc/sys/net/ipv4/ip_local_port_range`).
  Windows refused to bind UDP ports in that range ("Only one usage of each
  socket address" / WSAEACCES with `ReuseAddress`) while nothing was bound
  in WSL (observed with ports 47561–47577). Mirrored mode therefore
  reserves WSL's ephemeral range on Windows.

### 4.4 UDP probes between Windows and WSL (observed)

Python 3 in WSL, PowerShell `System.Net.Sockets` on Windows. Multicast group
239.255.0.1 (the SPDP group). Throwaway ports 38571–38597 and 47551–47557.

| Direction | Path | Result |
|---|---|---|
| Windows → WSL | unicast to 127.0.0.1 | received |
| Windows → WSL | unicast to 192.168.0.161 (LAN IP) | received |
| Windows → WSL | multicast, sender and receiver on 192.168.0.161 | received |
| Windows → WSL | multicast, sender on 127.0.0.1, receiver joined on 127.0.0.1 | **not received** (timeout) |
| WSL → Windows | unicast to 127.0.0.1 | received |
| WSL → Windows | unicast to 192.168.0.161 | received |
| WSL → Windows | multicast via 192.168.0.161 | **not received** (timeout) |
| WSL → Windows | multicast via 127.0.0.1 | **not received** (timeout) |
| Windows holds `127.0.0.1:38591`; WSL binds same | – | WSL `OSError(98, 'Address already in use')`; WSL's datagram to that port reached the **Windows** socket |
| WSL holds `127.0.0.1:38593`; Windows binds same | – | Windows "Only one usage of each socket address…" |
| Windows binds `0.0.0.0:38595` with `ReuseAddress`; WSL binds same with `SO_REUSEADDR` | – | WSL `Address already in use` |
| WSL binds `0.0.0.0:38597` with `SO_REUSEADDR`; Windows binds same with `ReuseAddress` | – | Windows WSAEACCES ("forbidden by its access permissions") |

Answer to "does mirrored networking support multicast on loopback or on the
LAN interface": on loopback, no, in either direction. On the LAN interface,
only Windows → WSL. Microsoft's mirrored-mode documentation lists multicast
support among the mode's features (external knowledge, not checkable
offline). The probes above show it is not bidirectional here. The probes did
not use DDS; they say nothing about Fast DDS or Cyclone behaviour beyond the
socket level.

Consequences, inferred from the probes and not tested with DDS:

- Discovery must be unicast on both sides (peers = 127.0.0.1). This is Q1's
  first fallback, and it also keeps DDS on loopback as the Security section
  asks.
- Both stacks must avoid binding the multicast port 7400 (and 7401). If
  one binds it first, the other cannot (last two table rows).
- Unicast participant ports are unique across Windows and WSL (rows 9-10),
  so participant indices auto-increment as on a single host: Isaac takes
  index 0 (7410/7411), and the first Cyclone process takes the next free
  one. This needs `ParticipantIndex=auto` in Cyclone (§4.1).
- UDP to a WSL-owned 127.0.0.1 port from inside WSL also goes through the
  Windows host (§4.3 routing). This was not measured and may add latency
  to container-internal DDS.

### 4.5 Hyper-V firewall (observed)

`powershell.exe Get-NetFirewallHyperVVMSetting -PolicyStore ActiveStore`:
`Name {40E0AC32-46A5-438A-A0B2-2B479E8F2E90}`, `Enabled True`,
`DefaultInboundAction Block`, `DefaultOutboundAction Allow`,
`LoopbackEnabled True`, `AllowHostPolicyMerge True`.
`Get-NetFirewallHyperVVMCreator`: that ID is `FriendlyName WSL`. The
probes in §4.4 show that Windows → WSL inbound UDP from the host itself (on
127.0.0.1 and on the LAN IP) is not blocked by this setting. Whether a
*different* LAN host can reach WSL ports was not tested.

---

## 5. Container side: packages and interoperability

### 5.1 Packages

- `project/container/Dockerfile:6` is already `FROM ros:humble-ros-base`.
  Lines 8-9 install only `python3-pip`, `iproute2`, `grpcio`, `protobuf`.
- What `ros:humble-ros-base` contains (observed in the local
  `intrinsic-core:dev` image, built on it; `dpkg -l`):
  `ros-humble-fastrtps 2.6.12`, `ros-humble-fastcdr 1.0.29`,
  `ros-humble-rmw-fastrtps-cpp 6.2.10`, `ros-humble-rmw-implementation 2.8.5`,
  `ros-humble-ros-base 0.10.0`. **No Cyclone packages.**
  `/opt/ros/humble/lib` has `librmw_fastrtps_cpp.so` but no
  `librmw_cyclonedds_cpp.so`.
- Apt candidates (observed in a throwaway container after `apt-get update`):

| Package | Candidate |
|---|---|
| `ros-humble-rmw-cyclonedds-cpp` | 1.3.5-1jammy.20260907 |
| `ros-humble-cyclonedds` | 0.10.5-2jammy.20260226 |
| `ros-humble-iceoryx-posh` (pulled in via `iceoryx-binding-c`) | 2.0.5 |
| `ros-humble-demo-nodes-cpp` | 0.20.10 |
| `ros-humble-demo-nodes-py` | 0.20.10 |

  `ros-humble-rmw-cyclonedds-cpp` depends on `ros-humble-cyclonedds` and
  `ros-humble-iceoryx-binding-c` (observed: `apt-cache depends`).
- Minimum for phase 0.5: `ros-humble-rmw-cyclonedds-cpp`, with
  `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and `CYCLONEDDS_URI`. Optionally
  `ros-humble-demo-nodes-cpp`/`-py`. A Python round-trip script needs only
  `rclpy`, `sensor_msgs` and `builtin_interfaces`, which ros-base already
  has (external knowledge: they are part of ros-base).
- The intrinsic-core:dev image's existing contents are not changed by this
  phase. Adding the apt package is a Dockerfile edit for the phase 0.5 plan.

### 5.2 Topic interoperability: `sensor_msgs/JointState`, `builtin_interfaces/Time`

- Both sides are Humble message definitions: Isaac `sensor_msgs-4.9.0`,
  `builtin_interfaces-1.2.2` (§1.3); the container gets them from the
  Humble apt repository. Bazel uses `release-humble-20250721` (§6).
- **Type hash:** not an issue in Humble. Type hashes (REP 2011) arrived in
  Iron, so Humble endpoints match on the DDS type name only
  (`sensor_msgs::msg::dds_::JointState_`) (external knowledge). Cyclone's
  XTypes type discovery is compiled out in Bazel
  (`ENABLE_TYPE_DISCOVERY=OFF`, `cyclonedds.BUILD.bazel:106`), so it sends
  no type information that Fast DDS would need to understand.
- **Encoding:** both use XCDR1 for ROS types in Humble (external knowledge).
- **Discovery:** must be unicast (§4.4). Cyclone needs
  `ParticipantIndex=auto` (§4.1).
- **Large messages** (camera, phase 2): the Cyclone defaults of 14720 B
  messages and 1344 B fragments (§4.1) and the 208 KiB socket buffer
  limit (§4.3) mean a 1.6 MB depth frame becomes about 1200 fragments.
  Reliable cross-vendor fragment reassembly is expected to work but was
  not tested. Not needed for the phase 0.5 round trip.
- Cross-vendor JointState / Time exchange between Cyclone 0.10.x and Fast
  DDS 2.6.x is a commonly reported working combination (external
  knowledge). **Not run on this machine.**

### 5.3 Service interoperability (risk)

rmw_cyclonedds serialises a `cdds_request_header_t { uint64_t guid;
int64_t seq; }` into the CDR stream ahead of every service request and
reply (`$X_ros2_rmw_cyclonedds/rmw_cyclonedds_cpp/src/serdata.hpp:73-77`;
injected at `serdata.cpp:126-132`, `:594-596`, `:619-623`; parsed as a
prefix at `serdata.cpp:366-376`). The source comment at `serdata.cpp:619-621`
reads: "inject the service invocation header data into the CDR stream -- I
haven't checked how it is done in the official RMW implementations, so it is
probably incompatible." rmw_fastrtps carries the request identity
in RTPS sample identity metadata instead of the payload (external
knowledge; its source is not on this machine and its DLL strings were not
conclusive). If so, a Cyclone client calling a Fast DDS server (or the
reverse) gets 16 bytes of misaligned payload or no match.

This affects `/sim/reset`, `/sim/attach_board`, `/sim/detach_board`
(`std_srvs/Trigger`) and `isaacsim.ros2.sim_control`'s
`simulation_interfaces` services (`SpawnEntity`, `GetEntities`, …), all of
which the design routes across the vendor boundary ("Isaac Sim side"
table). Phase 0.5 should call one `std_srvs/srv/Trigger` service in each
direction.

---

## 6. Intrinsic's Bazel ROS 2 (what `ros2_hwm` will link)

- Module: `bazel_dep(name = "com_github_mvukov_rules_ros2", version =
  "0.0.0-20260118-78d85ff")` (`ic/MODULE.bazel:457`). Pinned in the lock
  file (`ic/MODULE.bazel.lock:614-615`). Patched only to drop `-latomic` from
  rclpy (`ic/MODULE.bazel:974-981`;
  `ic/third_party/rules_ros2/rules_ros2_atomic.patch:1-12`).
- Repositories used: `ros2_common_interfaces`, `ros2_geometry2`,
  `ros2_rcl_interfaces`, `ros2_rclcpp`, `ros2_rclpy`, `ros2_rcutils`,
  `ros2_rmw`, `ros2_rosbag2`, `ros2_rosidl`, `ros2_rosidl_typesupport`
  (`ic/MODULE.bazel:958-972`).
- ROS distro: "ROS 2 packages are by default locked to versions from
  release-humble-20250721" (`$RR/README.md:42`; `$RR/repositories/repositories.bzl:245-246`).
  "Only CycloneDDS middleware can be used at the moment" (`$RR/README.md:25-26`).
- Versions (`$RR/repositories/ros2_repositories_impl.bzl`):

| Component | Version | Lines |
|---|---|---|
| CycloneDDS | 0.10.5 (`cyclonedds-0.10.5`) | 35-39 |
| iceoryx | 2.0.5 | 55-59 |
| rmw_cyclonedds | 1.3.4 | 232-236 |

  Confirmed in the fetched sources: `project(CycloneDDS VERSION 0.10.5)`
  (`$X_cyclonedds/CMakeLists.txt:13`); `<version>1.3.4</version>`
  (`$X_ros2_rmw_cyclonedds/rmw_cyclonedds_cpp/package.xml:5`).
- **The RMW is fixed at build time.** `rmw_implementation` is compiled with
  `RMW_LIBRARY_PATH` = the rmw_cyclonedds library
  (`$RR/repositories/rmw_implementation.BUILD.bazel:14-20`). `load_library()`
  loads only that path and never reads `RMW_IMPLEMENTATION`
  (`$X_ros2_rmw_implementation/rmw_implementation/src/functions.cpp:84-86`;
  the patch is `$RR/repositories/patches/rmw_implementation_library_path.patch`).
  `CYCLONEDDS_URI` and `ROS_LOCALHOST_ONLY` are still honoured
  (`rmw_node.cpp:1052-1071`, §4.1).
- Shared memory: off by default (`enable_shm` default `False`,
  `$RR/repositories/cyclonedds.BUILD.bazel:9-12`).
- Matching on the container side: apt's `ros-humble-cyclonedds 0.10.5`
  equals Bazel's. Apt's `rmw-cyclonedds-cpp` is 1.3.5 against Bazel's 1.3.4
  (§5.1). Only the latest version is a candidate in the apt index, so 1.3.4
  cannot be pinned from apt. The 1.3.4 → 1.3.5 changes were not reviewed.
- Not DDS-related: `ic/third_party/ros2/ros_interfaces/jazzy/` holds
  *protobuf* versions of a few ROS messages (`*_proto`, `*_py_pb2` targets
  used by `ic/intrinsic_sdk/intrinsic/math/python/BUILD:63-82` and
  `ic/intrinsic/tools/introspect/BUILD:41-43`). It does not change the DDS
  distro.
- Intrinsic code that links rclcpp: `ic/intrinsic_sdk/intrinsic/platform/pubsub/BUILD:109`,
  `ic/intrinsic/world/BUILD:149-152`,
  `ic/intrinsic_perception/intrinsic/perception/cameras/BUILD:512-518,550-551`.
- Intrinsic's own ROS tutorials use Zenoh, not DDS:
  `export RMW_IMPLEMENTATION=rmw_zenoh_cpp`
  (`ic/developer_resources/learn/tutorials/visualize_the_robot.md:35`;
  `moveit_grasp_planning.md:76`). `icon/ros2_bridge_tutorial.md` does not
  mention an RMW, DDS vendor or domain (observed: `rg` empty).

---

## 7. Not verified, and open items

- **No DDS traffic was exchanged.** Fast DDS ↔ Cyclone discovery, the XML
  in §3.3, the `CYCLONEDDS_URI` in §4.2, and the round-trip latency are all
  unmeasured. §4.4 is socket-level only.
- Whether Fast DDS 2.6.10 skips binding port 7400 when
  `metatrafficUnicastLocatorList` is set (§3.3). What Cyclone 0.10.5 does
  if 7400 is already taken by Windows while multicast is enabled (refuses
  to start, or logs and continues).
- Whether Fast DDS 2.6.10 accepts interface names in `interfaceWhiteList`.
  Its default `maxInitialPeersRange` (believed to be 4).
- Whether cross-vendor services fail (§5.3). The rmw_fastrtps side was not
  inspected (binary only).
- WSL → Windows LAN multicast failed in one probe. Whether that comes from
  Windows' multicast loopback behaviour on the physical NIC or from mirrored
  mode was not isolated. It does not change the conclusion (use unicast).
- Reachability of WSL ports from another LAN host under
  `DefaultInboundAction Block` was not tested (needs a second machine).
- Latency added by WSL's 127.0.0.1 path through `loopback0` and the Windows
  host, for both container ↔ Isaac and container-internal traffic.
- `ROS2Publisher` generic-node attribute names for nested fields of
  `geometry_msgs/WrenchStamped` (§2.1). Writing raw JSON into
  `inputs:qosProfile` without a QoS node (§2.2).
- `control_msgs` is not bundled with Isaac's internal Humble (§1.3). The
  design's `/gripper_command` type needs a phase 2 decision.
- The user-level rule "BEST_EFFORT, KEEP_LAST depth 1" differs from the
  design's "reliable QoS, depth 1" ("Timing"). This inventory does not
  resolve that; both are expressible with `ROS2QoSProfile` (§2.2).
- Microsoft's documentation on mirrored-mode multicast was not consulted
  (offline). §4.4 is the only evidence.
