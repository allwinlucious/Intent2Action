#ifndef EXT_ROS2_HWM_ROS2_HWM_H_
#define EXT_ROS2_HWM_ROS2_HWM_H_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "ext/ros2_hwm/ros2_hwm_config.pb.h"
#include "ext/ros2_hwm/ros2_hwm_logic.h"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "intrinsic/icon/control/realtime_clock_interface.h"
#include "intrinsic/icon/control/safety/safety_messages.fbs.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/hardware_module_init_context.h"
#include "intrinsic/icon/hal/hardware_module_interface.h"
#include "intrinsic/icon/hal/interfaces/force_torque.fbs.h"
#include "intrinsic/icon/hal/interfaces/gripper.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/payload_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/payload_state.fbs.h"
#include "intrinsic/icon/utils/async_buffer.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/u_int32.hpp"

namespace ros2_hwm {

// ICON hardware module for the Isaac Franka (phase_3.md, decisions 5-12):
// ticks ICON's clock once per /joint_states + /wrist_wrench pair and publishes
// ICON's command on /joint_command and /gripper_command.
//
// Threads: the ROS executor thread pairs states and fills the tick queue; the
// tick thread ticks ICON and publishes; ICON's runtime threads call
// ReadStatus/ApplyCommand during a tick. Data crosses into and out of the
// real-time calls only through AsyncBuffers and atomics, so those calls never
// lock or allocate.
class Ros2HardwareModule final
    : public intrinsic::icon::HardwareModuleInterface {
 public:
  Ros2HardwareModule() = default;
  ~Ros2HardwareModule() override;

  absl::Status Init(
      intrinsic::icon::HardwareModuleInitContext& init_context) override;
  absl::Status Prepare() override;
  intrinsic::icon::RealtimeStatus Activate() override;
  intrinsic::icon::RealtimeStatus Deactivate() override;
  absl::Status EnableMotion() override;
  absl::Status DisableMotion() override;
  absl::Status ClearFaults() override;
  absl::Status Shutdown() override;
  intrinsic::icon::RealtimeStatus ReadStatus() override;
  intrinsic::icon::RealtimeStatus ApplyCommand() override;

 private:
  // What ApplyCommand produced in one tick.
  struct Command {
    int64_t stamp_ns = 0;
    std::array<double, ARM_DOF> position{};
    std::array<double, ARM_DOF> velocity{};
    GripperCommand gripper;
  };

  void onJointState(const sensor_msgs::msg::JointState& msg);
  void onWrench(const geometry_msgs::msg::WrenchStamped& msg);
  void onEpoch(const std_msgs::msg::UInt32& msg);
  void onStatusTimer();
  void enqueue(const TickEntry& entry);

  void tickLoop();
  bool tickWithRetries(const TickEntry& entry);
  void publishCommand(const Command& command);
  void startTickThread();
  void stopTickThread();
  bool tickStopping();
  bool queueReadyOrStopping() const ABSL_SHARED_LOCKS_REQUIRED(queue_mutex_);
  void stopRos();

  Ros2HwmConfig config_;
  intrinsic::icon::RealtimeClockInterface* realtime_clock_ = nullptr;

  // HAL interfaces (decision 8).
  intrinsic::icon::StrictHardwareInterfaceHandle<
      intrinsic_fbs::JointPositionCommand>
      joint_position_command_;
  intrinsic::icon::MutableHardwareInterfaceHandle<
      intrinsic_fbs::JointPositionState>
      joint_position_state_;
  intrinsic::icon::MutableHardwareInterfaceHandle<
      intrinsic_fbs::JointVelocityState>
      joint_velocity_state_;
  intrinsic::icon::MutableHardwareInterfaceHandle<
      intrinsic_fbs::JointAccelerationState>
      joint_acceleration_state_;
  intrinsic::icon::MutableHardwareInterfaceHandle<
      intrinsic_fbs::JointTorqueState>
      joint_torque_state_;
  intrinsic::icon::HardwareInterfaceHandle<intrinsic_fbs::PayloadCommand>
      payload_command_;
  intrinsic::icon::MutableHardwareInterfaceHandle<intrinsic_fbs::PayloadState>
      payload_state_;
  intrinsic::icon::MutableHardwareInterfaceHandle<
      intrinsic_fbs::SafetyStatusMessage>
      safety_status_;
  intrinsic::icon::MutableHardwareInterfaceHandle<intrinsic_fbs::GripperStatus>
      gripper_status_;
  intrinsic::icon::HardwareInterfaceHandle<intrinsic_fbs::GripperCommand>
      gripper_command_;
  intrinsic::icon::MutableHardwareInterfaceHandle<
      intrinsic_fbs::ForceTorqueStatus>
      force_torque_status_;
  intrinsic::icon::HardwareInterfaceHandle<intrinsic_fbs::ForceTorqueCommand>
      force_torque_command_;

  // ROS executor thread only.
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      wrench_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr epoch_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::thread ros_thread_;
  std::vector<std::string> arm_joint_names_;
  std::string finger_joint_name_;
  std::array<int, ARM_DOF> arm_indices_{};
  int finger_index_ = -1;
  bool indices_known_ = false;
  std::optional<StampPairer> pairer_;
  bool states_flowing_ = false;
  int64_t last_summary_ns_ = 0;
  uint64_t summary_ticks_ = 0;
  uint64_t summary_dropped_ = 0;
  uint64_t logged_reset_faults_ = 0;
  uint64_t logged_taring_starts_ = 0;
  uint64_t logged_taring_ends_ = 0;

  // Tick thread only (publishers are thread-safe in rclcpp).
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      gripper_command_pub_;
  sensor_msgs::msg::JointState joint_command_msg_;
  sensor_msgs::msg::JointState gripper_command_msg_;
  std::optional<GripperCommand> last_gripper_;
  std::thread tick_thread_;

  // Between the ROS thread and the tick thread (decision 11: depth 2).
  absl::Mutex queue_mutex_;
  DropOldestQueue<TickEntry, 2> queue_ ABSL_GUARDED_BY(queue_mutex_);
  bool active_ ABSL_GUARDED_BY(queue_mutex_) = false;
  bool tick_stopping_ ABSL_GUARDED_BY(queue_mutex_) = false;

  // Tick thread -> ReadStatus: the entry being ticked.
  intrinsic::AsyncBuffer<TickEntry> tick_buffer_;
  // ApplyCommand -> tick thread: the command of the tick.
  intrinsic::AsyncBuffer<Command> command_buffer_;

  // ICON runtime threads only (ReadStatus and ApplyCommand run in sequence).
  TickEntry previous_entry_;
  bool has_previous_entry_ = false;
  int64_t current_stamp_ns_ = 0;
  TaringState taring_;
  uint32_t read_baseline_epoch_ = 0;
  bool last_retare_ = false;
  bool reset_fault_active_ = false;

  // Epoch (decision 12): written by the ROS thread and ClearFaults.
  std::atomic<bool> epoch_known_{false};
  std::atomic<uint32_t> latest_epoch_{0};
  std::atomic<uint32_t> baseline_epoch_{0};

  // Counters for the status timer's log lines (relaxed: statistics only).
  std::atomic<uint64_t> ticks_{0};
  std::atomic<uint64_t> dropped_{0};
  std::atomic<int64_t> last_state_ns_{0};
  std::atomic<uint64_t> reset_faults_{0};
  std::atomic<uint64_t> taring_starts_{0};
  std::atomic<uint64_t> taring_ends_{0};
};

}  // namespace ros2_hwm

#endif  // EXT_ROS2_HWM_ROS2_HWM_H_
