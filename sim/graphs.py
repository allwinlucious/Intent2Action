"""On-demand OmniGraph graphs: all of run_scene.py's ROS 2 topics (D7).

Each graph is evaluated by run_scene.py with og.Controller.evaluate_sync, so
it runs exactly when the loop says (phase_2.md, target file 9; Inv §3.4).
"""

import json

import numpy as np
import omni.graph.core as og
import omni.syntheticdata
from omni.syntheticdata import _syntheticdata as sd
import usdrt.Sdf

KEYS = og.Controller.Keys
ON_DEMAND = og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND
BRIDGE = "isaacsim.ros2.bridge"
SIM_TIME = "isaacsim.core.nodes.IsaacReadSimulationTime"
ON_TICK = ("OnTick", "omni.graph.action.OnTick")
OPTICAL_FRAME = "d455_optical"
# The bridge already publishes the camera prim's /tf in ROS optical convention
# (z forward, y down; phase_2_notes.md), so d455_optical is the same frame.
OPTICAL_ROTATION_IJKR = [0.0, 0.0, 0.0, 1.0]
# A bridge ROS2Publisher creates its DDS writer on its 2nd evaluation and
# publishes from its 3rd (phase_2_notes.md, Addendum A).
PUBLISHER_WARMUP_EVALUATIONS = 2
# Gates in front of the Camera Helper writers, one per render product; the
# helper sets their inputs:step itself (OgnROS2CameraHelper.post_attach).
IMAGE_GATES = [
    omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(name)
    + "IsaacSimulationGate"
    for name in (
        sd.SensorType.Rgb.name,
        sd.SensorType.DistanceToImagePlane.name,
    )
]


def _qos(durability: str = "volatile") -> str:
  return json.dumps(
      {
          "history": "keepLast",
          "depth": 1,
          "reliability": "reliable",
          "durability": durability,
          "deadline": 0.0,
          "lifespan": 0.0,
          "liveliness": "systemDefault",
          "leaseDuration": 0.0,
      }
  )


def _graph(name, nodes, connect, values, evaluator="execution") -> og.Graph:
  graph, _, _, _ = og.Controller.edit(
      {
          "graph_path": f"/{name}_graph",
          "evaluator_name": evaluator,
          "pipeline_stage": ON_DEMAND,
      },
      {
          KEYS.CREATE_NODES: [ON_TICK] + nodes,
          KEYS.CONNECT: connect,
          KEYS.SET_VALUES: values,
      },
  )
  return graph


def _attribute(graph: og.Graph, node: str, name: str) -> og.Attribute:
  """Looks up by node, which also finds dynamic message-field attributes (Inv §6.1)."""
  return og.Controller.attribute(
      name, og.Controller.node(f"{graph.get_path_to_graph()}/{node}")
  )


class Graphs:
  """The graphs and the attributes run_scene.py reads or writes."""

  def __init__(self, robot_prim: str, render_product: str):
    qos = _qos()
    self.state = _graph(
        "state",
        [
            ("SimTime", SIM_TIME),
            ("JointState", f"{BRIDGE}.ROS2PublishJointState"),
            ("Wrench", f"{BRIDGE}.ROS2Publisher"),
            ("Clock", f"{BRIDGE}.ROS2PublishClock"),
        ],
        [
            ("OnTick.outputs:tick", "JointState.inputs:execIn"),
            ("OnTick.outputs:tick", "Wrench.inputs:execIn"),
            ("OnTick.outputs:tick", "Clock.inputs:execIn"),
            ("SimTime.outputs:simulationTime", "JointState.inputs:timeStamp"),
            ("SimTime.outputs:simulationTime", "Clock.inputs:timeStamp"),
        ],
        [
            ("JointState.inputs:topicName", "joint_states"),
            ("JointState.inputs:qosProfile", qos),
            ("Wrench.inputs:topicName", "wrist_wrench"),
            ("Wrench.inputs:qosProfile", qos),
            ("Wrench.inputs:messagePackage", "geometry_msgs"),
            ("Wrench.inputs:messageName", "WrenchStamped"),
            ("Clock.inputs:topicName", "clock"),
            ("Clock.inputs:qosProfile", qos),
        ],
    )
    _attribute(self.state, "JointState", "inputs:targetPrim").set(
        [usdrt.Sdf.Path(robot_prim)]
    )

    self.command = _graph(
        "command",
        [
            ("Arm", f"{BRIDGE}.ROS2SubscribeJointState"),
            ("Gripper", f"{BRIDGE}.ROS2SubscribeJointState"),
        ],
        [
            ("OnTick.outputs:tick", "Arm.inputs:execIn"),
            ("OnTick.outputs:tick", "Gripper.inputs:execIn"),
        ],
        [
            ("Arm.inputs:topicName", "joint_command"),
            ("Arm.inputs:qosProfile", qos),
            ("Gripper.inputs:topicName", "gripper_command"),
            ("Gripper.inputs:qosProfile", qos),
        ],
    )

    self.camera_request = _graph(
        "camera_request",
        [
            ("Request", f"{BRIDGE}.ROS2Subscriber"),
            ("Counter", "omni.graph.action.Counter"),
        ],
        [
            ("OnTick.outputs:tick", "Request.inputs:execIn"),
            ("Request.outputs:execOut", "Counter.inputs:execIn"),
        ],
        [
            ("Request.inputs:topicName", "camera/request"),
            ("Request.inputs:qosProfile", qos),
            ("Request.inputs:messagePackage", "std_msgs"),
            ("Request.inputs:messageName", "Header"),
        ],
    )

    self.epoch = _graph(
        "epoch",
        [("Epoch", f"{BRIDGE}.ROS2Publisher")],
        [("OnTick.outputs:tick", "Epoch.inputs:execIn")],
        [
            ("Epoch.inputs:topicName", "sim/epoch"),
            ("Epoch.inputs:qosProfile", _qos("transientLocal")),
            ("Epoch.inputs:messagePackage", "std_msgs"),
            ("Epoch.inputs:messageName", "UInt32"),
        ],
    )

    self.tf = _graph(
        "tf",
        [
            ("SimTime", SIM_TIME),
            ("Optical", f"{BRIDGE}.ROS2PublishRawTransformTree"),
        ],
        [
            ("OnTick.outputs:tick", "Optical.inputs:execIn"),
            ("SimTime.outputs:simulationTime", "Optical.inputs:timeStamp"),
        ],
        [
            ("Optical.inputs:topicName", "tf"),
            ("Optical.inputs:qosProfile", qos),
            ("Optical.inputs:parentFrameId", "Camera_Pseudo_Depth"),
            ("Optical.inputs:childFrameId", OPTICAL_FRAME),
            ("Optical.inputs:rotation", OPTICAL_ROTATION_IJKR),
        ],
    )

    # Evaluated once here: that creates the writers, which then publish on
    # every render() (Inv §3.4, §4.5).
    self.camera = None if render_product is None else _graph(
        "camera",
        [
            ("Rgb", f"{BRIDGE}.ROS2CameraHelper"),
            ("Depth", f"{BRIDGE}.ROS2CameraHelper"),
            ("Info", f"{BRIDGE}.ROS2CameraInfoHelper"),
        ],
        [
            ("OnTick.outputs:tick", "Rgb.inputs:execIn"),
            ("OnTick.outputs:tick", "Depth.inputs:execIn"),
            ("OnTick.outputs:tick", "Info.inputs:execIn"),
        ],
        [
            ("Rgb.inputs:renderProductPath", render_product),
            ("Rgb.inputs:topicName", "camera/rgb"),
            ("Rgb.inputs:type", "rgb"),
            ("Rgb.inputs:frameId", OPTICAL_FRAME),
            ("Rgb.inputs:qosProfile", qos),
            ("Depth.inputs:renderProductPath", render_product),
            ("Depth.inputs:topicName", "camera/depth"),
            ("Depth.inputs:type", "depth"),
            ("Depth.inputs:frameId", OPTICAL_FRAME),
            ("Depth.inputs:qosProfile", qos),
            ("Info.inputs:renderProductPath", render_product),
            ("Info.inputs:topicName", "camera/camera_info"),
            ("Info.inputs:frameId", OPTICAL_FRAME),
            ("Info.inputs:qosProfile", qos),
        ],
        evaluator="push",
    )
    if self.camera is not None:
      og.Controller.evaluate_sync(self.camera)

    # Subscribers create their output attributes when they first compute
    # (Inv §6.1). Publishers in graphs evaluated every cycle need no warm-up;
    # /sim/epoch is evaluated only at start-up and on reset, so it gets one
    # here (with data 0, the first epoch, should a warm-up publish).
    for graph in (self.command, self.camera_request):
      og.Controller.evaluate_sync(graph)
    for _ in range(PUBLISHER_WARMUP_EVALUATIONS):
      og.Controller.evaluate_sync(self.epoch)
    self._render_product = render_product

    self._wrench = {
        name: _attribute(self.state, "Wrench", f"inputs:{name}")
        for name in (
            "header:frame_id",
            "header:stamp:sec",
            "header:stamp:nanosec",
            "wrench:force:x",
            "wrench:force:y",
            "wrench:force:z",
            "wrench:torque:x",
            "wrench:torque:y",
            "wrench:torque:z",
        )
    }
    self._wrench["header:frame_id"].set("panda_hand")
    self._arm_stamp = _attribute(self.command, "Arm", "outputs:timeStamp")
    self._arm_position = _attribute(
        self.command, "Arm", "outputs:positionCommand"
    )
    self._arm_velocity = _attribute(
        self.command, "Arm", "outputs:velocityCommand"
    )
    self._gripper_stamp = _attribute(
        self.command, "Gripper", "outputs:timeStamp"
    )
    self._gripper_position = _attribute(
        self.command, "Gripper", "outputs:positionCommand"
    )
    self._gripper_effort = _attribute(
        self.command, "Gripper", "outputs:effortCommand"
    )
    self._request_count = _attribute(
        self.camera_request, "Counter", "outputs:count"
    )
    self._request_sec = _attribute(
        self.camera_request, "Request", "outputs:stamp:sec"
    )
    self._request_nanosec = _attribute(
        self.camera_request, "Request", "outputs:stamp:nanosec"
    )
    self._epoch_data = _attribute(self.epoch, "Epoch", "inputs:data")
    self._qos = qos
    self._tf_target_paths = None

  def publish_state(
      self, sec: int, nanosec: int, force: np.ndarray, torque: np.ndarray
  ) -> None:
    """Publishes /joint_states, /wrist_wrench and /clock for this cycle."""
    self._wrench["header:stamp:sec"].set(sec)
    self._wrench["header:stamp:nanosec"].set(nanosec)
    for axis, f, tau in zip("xyz", force, torque):
      self._wrench[f"wrench:force:{axis}"].set(float(f))
      self._wrench[f"wrench:torque:{axis}"].set(float(tau))
    og.Controller.evaluate_sync(self.state)

  def poll_command(self) -> float:
    """Takes in waiting commands; returns the arm command's stamp (s)."""
    og.Controller.evaluate_sync(self.command)
    return self._arm_stamp.get()

  def arm_command(self) -> tuple[np.ndarray, np.ndarray]:
    return (
        np.asarray(self._arm_position.get(), dtype=float),
        np.asarray(self._arm_velocity.get(), dtype=float),
    )

  def gripper_command(self) -> tuple[float, float, float]:
    """(stamp s, position m, effort N) of the latest /gripper_command."""
    position = self._gripper_position.get()
    effort = self._gripper_effort.get()
    return (
        self._gripper_stamp.get(),
        float(position[0]) if len(position) else 0.0,
        float(effort[0]) if len(effort) else 0.0,
    )

  def poll_camera_request(self) -> tuple[int, float]:
    """(requests received so far, stamp s of the latest)."""
    og.Controller.evaluate_sync(self.camera_request)
    return (
        self._request_count.get(),
        self._request_sec.get() + self._request_nanosec.get() * 1e-9,
    )

  def publish_epoch(self, epoch: int) -> None:
    self._epoch_data.set(epoch)
    og.Controller.evaluate_sync(self.epoch)

  def publish_tf(self) -> None:
    og.Controller.evaluate_sync(self.tf)

  def set_image_publishing(self, enabled: bool) -> None:
    """Whether the next render() publishes /camera/rgb and /camera/depth."""
    if self._render_product is None:
      return
    for gate in IMAGE_GATES:
      omni.syntheticdata.SyntheticData.Get().set_node_attributes(
          gate, {"inputs:step": int(enabled)}, self._render_product
      )

  def set_tf_targets(
      self, paths: list[str], prims_replaced: bool = False
  ) -> None:
    """Publishes the poses of paths on /tf from now on.

    The Transform Tree node reads rigid bodies through a physics view built on
    its first evaluation: it ignores targets added later and fails for good
    once one of its bodies is deleted. So the node is re-created whenever the
    targets change or their prims were replaced.
    """
    if paths == self._tf_target_paths and not prims_replaced:
      return
    if self._tf_target_paths is not None:
      og.Controller.edit(self.tf, {KEYS.DELETE_NODES: ["Tree"]})
    root = self.tf.get_path_to_graph()
    og.Controller.edit(
        self.tf,
        {
            KEYS.CREATE_NODES: [("Tree", f"{BRIDGE}.ROS2PublishTransformTree")],
            KEYS.CONNECT: [
                (f"{root}/OnTick.outputs:tick", f"{root}/Tree.inputs:execIn"),
                (
                    f"{root}/SimTime.outputs:simulationTime",
                    f"{root}/Tree.inputs:timeStamp",
                ),
            ],
            KEYS.SET_VALUES: [
                (f"{root}/Tree.inputs:topicName", "tf"),
                (f"{root}/Tree.inputs:qosProfile", self._qos),
            ],
        },
    )
    _attribute(self.tf, "Tree", "inputs:targetPrims").set(
        [usdrt.Sdf.Path(p) for p in paths]
    )
    self._tf_target_paths = paths
