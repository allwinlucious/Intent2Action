#include "ext/ros2_hwm/ros2_hwm.h"

#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <optional>
#include <string>
#include <thread>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "intrinsic/icon/control/safety/extern/safety_status.fbs.h"
#include "intrinsic/icon/control/safety/safety_messages.fbs.h"
#include "intrinsic/icon/hal/default_hardware_interfaces.h"
#include "intrinsic/icon/hal/hardware_interface_registry.h"
#include "intrinsic/icon/hal/hardware_interface_traits.h"
#include "intrinsic/icon/hal/hardware_module_registry.h"
#include "intrinsic/icon/hal/interfaces/gripper_utils.h"
#include "intrinsic/icon/hal/interfaces/payload_state_utils.h"
#include "intrinsic/icon/hal/interfaces/robot_payload_utils.h"
#include "intrinsic/icon/hal/module_config.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/robot_payload/robot_payload_base.h"

// The gripper interface types are registered only in ICON's gripper part
// (Inv A.4); the module must register them too, before advertising them.
namespace intrinsic::icon::hardware_interface_traits {
INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::GripperStatus,
                                 intrinsic_fbs::BuildGripperStatus,
                                 "intrinsic_fbs.GripperStatus")
INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::GripperCommand,
                                 intrinsic_fbs::BuildGripperCommand,
                                 "intrinsic_fbs.GripperCommand")
}  // namespace intrinsic::icon::hardware_interface_traits

namespace ros2_hwm {
namespace {

using ::intrinsic::icon::OkStatus;
using ::intrinsic::icon::RealtimeStatus;

constexpr absl::Duration TICK_TIMEOUT = absl::Seconds(10);
constexpr absl::Duration CLOCK_RESET_TIMEOUT = absl::Seconds(10);
constexpr absl::Duration TICK_RETRY_SLEEP = absl::Milliseconds(100);
constexpr int TICK_RETRIES = 3;
constexpr absl::Duration QUEUE_WAIT = absl::Seconds(1);
constexpr std::chrono::seconds STATUS_PERIOD(1);
constexpr int64_t NO_STATE_NS = 1'000'000'000;
constexpr int64_t SUMMARY_PERIOD_NS = 10'000'000'000;
constexpr double MAX_GRIPPER_WIDTH_M = 0.08;
constexpr char COMPONENT[] = "ros2_hwm";

int64_t steadyNowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

int64_t stampNs(const builtin_interfaces::msg::Time& stamp) {
  return int64_t{stamp.sec} * 1'000'000'000 + stamp.nanosec;
}

builtin_interfaces::msg::Time toStamp(int64_t ns) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(ns / 1'000'000'000);
  stamp.nanosec = static_cast<uint32_t>(ns % 1'000'000'000);
  return stamp;
}

}  // namespace

Ros2HardwareModule::~Ros2HardwareModule() {
  stopTickThread();
  stopRos();
}

absl::Status Ros2HardwareModule::Init(
    intrinsic::icon::HardwareModuleInitContext& init_context) {
  const intrinsic::icon::ModuleConfig& module_config =
      init_context.GetModuleConfig();
  INTR_ASSIGN_OR_RETURN(config_, module_config.GetConfig<Ros2HwmConfig>());
  if (config_.arm_joint_names_size() != ARM_DOF) {
    return absl::InvalidArgumentError(absl::StrCat(
        "arm_joint_names must list ", ARM_DOF, " joints, got ",
        config_.arm_joint_names_size()));
  }
  realtime_clock_ = module_config.GetRealtimeClock();
  if (realtime_clock_ == nullptr) {
    return absl::FailedPreconditionError(
        "ros2_hwm ticks ICON per simulation step and needs "
        "drives_realtime_clock: true");
  }
  pairer_.emplace(std::llround(config_.stamp_tolerance_s() * 1e9));

  intrinsic::icon::HardwareInterfaceRegistry& registry =
      init_context.GetInterfaceRegistry();
  INTR_ASSIGN_OR_RETURN(
      joint_position_command_,
      registry.AdvertiseStrictInterface<intrinsic_fbs::JointPositionCommand>(
          "joint_position_command", ARM_DOF));
  INTR_ASSIGN_OR_RETURN(
      joint_position_state_,
      registry.AdvertiseMutableInterface<intrinsic_fbs::JointPositionState>(
          "joint_position_state", ARM_DOF));
  INTR_ASSIGN_OR_RETURN(
      joint_velocity_state_,
      registry.AdvertiseMutableInterface<intrinsic_fbs::JointVelocityState>(
          "joint_velocity_state", ARM_DOF));
  INTR_ASSIGN_OR_RETURN(
      joint_acceleration_state_,
      registry
          .AdvertiseMutableInterface<intrinsic_fbs::JointAccelerationState>(
              "joint_acceleration_state", ARM_DOF));
  INTR_ASSIGN_OR_RETURN(
      joint_torque_state_,
      registry.AdvertiseMutableInterface<intrinsic_fbs::JointTorqueState>(
          "joint_torque_state", ARM_DOF));
  INTR_ASSIGN_OR_RETURN(
      payload_command_,
      registry.AdvertiseInterface<intrinsic_fbs::PayloadCommand>(
          "payload_command"));
  INTR_ASSIGN_OR_RETURN(
      payload_state_,
      registry.AdvertiseMutableInterface<intrinsic_fbs::PayloadState>(
          "payload_state"));
  INTR_ASSIGN_OR_RETURN(
      safety_status_,
      registry.AdvertiseMutableInterface<intrinsic_fbs::SafetyStatusMessage>(
          "safety_status",
          /*mode_of_safe_operation=*/intrinsic_fbs::ModeOfSafeOperation::
              AUTOMATIC,
          /*estop_button_status=*/intrinsic_fbs::ButtonStatus::NOT_AVAILABLE,
          /*enable_button_status=*/intrinsic_fbs::ButtonStatus::NOT_AVAILABLE,
          /*requested_behavior=*/
          intrinsic_fbs::RequestedBehavior::NORMAL_OPERATION));
  INTR_ASSIGN_OR_RETURN(
      gripper_status_,
      registry.AdvertiseMutableInterface<intrinsic_fbs::GripperStatus>(
          "gripper_status"));
  INTR_ASSIGN_OR_RETURN(
      gripper_command_,
      registry.AdvertiseInterface<intrinsic_fbs::GripperCommand>(
          "gripper_command"));
  INTR_ASSIGN_OR_RETURN(
      force_torque_status_,
      registry.AdvertiseMutableInterface<intrinsic_fbs::ForceTorqueStatus>(
          "force_torque_status"));
  INTR_ASSIGN_OR_RETURN(
      force_torque_command_,
      registry.AdvertiseInterface<intrinsic_fbs::ForceTorqueCommand>(
          "force_torque_command"));

  gripper_status_->mutate_max_width(MAX_GRIPPER_WIDTH_M);
  gripper_status_->mutate_in_error(false);
  force_torque_status_->mutate_retare_completed(true);

  arm_joint_names_.assign(config_.arm_joint_names().begin(),
                          config_.arm_joint_names().end());
  finger_joint_name_ = config_.finger_joint_name();
  joint_command_msg_.name = arm_joint_names_;
  joint_command_msg_.position.resize(ARM_DOF);
  joint_command_msg_.velocity.resize(ARM_DOF);
  gripper_command_msg_.name = {config_.finger_joint_name()};
  gripper_command_msg_.position.resize(1);
  gripper_command_msg_.effort.resize(1);

  // ROS can throw; turn that into a status at this boundary.
  try {
    if (!rclcpp::ok()) {
      // ICON's hardware_module_main owns SIGINT/SIGTERM.
      rclcpp::init(0, nullptr, rclcpp::InitOptions(),
                   rclcpp::SignalHandlerOptions::None);
    }
    node_ = std::make_shared<rclcpp::Node>(COMPONENT);
    const rclcpp::QoS qos = rclcpp::QoS(1).reliable().durability_volatile();
    const rclcpp::QoS latched = rclcpp::QoS(1).reliable().transient_local();
    state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        config_.joint_states_topic(), qos,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          onJointState(*msg);
        });
    wrench_sub_ =
        node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
            config_.wrench_topic(), qos,
            [this](geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
              onWrench(*msg);
            });
    epoch_sub_ = node_->create_subscription<std_msgs::msg::UInt32>(
        config_.epoch_topic(), latched,
        [this](std_msgs::msg::UInt32::ConstSharedPtr msg) { onEpoch(*msg); });
    joint_command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        config_.joint_command_topic(), qos);
    gripper_command_pub_ =
        node_->create_publisher<sensor_msgs::msg::JointState>(
            config_.gripper_command_topic(), qos);
    status_timer_ = node_->create_wall_timer(STATUS_PERIOD,
                                             [this] { onStatusTimer(); });
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
  } catch (const std::exception& e) {
    return absl::InternalError(absl::StrCat("ROS 2 set-up failed: ", e.what()));
  }
  last_summary_ns_ = steadyNowNs();
  ros_thread_ = std::thread([this] { executor_->spin(); });
  LOG(INFO) << COMPONENT << " initialized states=" << config_.joint_states_topic()
            << " wrench=" << config_.wrench_topic()
            << " command=" << config_.joint_command_topic()
            << " tolerance_s=" << config_.stamp_tolerance_s();
  return absl::OkStatus();
}

absl::Status Ros2HardwareModule::Prepare() {
  // Gazebo HWM pattern: Reset must not run while a tick is in progress.
  stopTickThread();
  {
    absl::MutexLock lock(&queue_mutex_);
    queue_.clear();
  }
  INTR_RETURN_IF_ERROR(realtime_clock_->Reset(CLOCK_RESET_TIMEOUT));
  has_previous_entry_ = false;
  velocity_estimator_.reset();
  // Isaac reopens the fingers on restart, so the next command must be sent.
  last_gripper_.reset();
  startTickThread();
  LOG(INFO) << COMPONENT << " prepared";
  return absl::OkStatus();
}

RealtimeStatus Ros2HardwareModule::Activate() {
  {
    absl::MutexLock lock(&queue_mutex_);
    active_ = true;
  }
  LOG(INFO) << COMPONENT << " activated";
  return OkStatus();
}

RealtimeStatus Ros2HardwareModule::Deactivate() {
  {
    absl::MutexLock lock(&queue_mutex_);
    active_ = false;
    queue_.clear();
  }
  LOG(INFO) << COMPONENT << " deactivated";
  return OkStatus();
}

absl::Status Ros2HardwareModule::EnableMotion() {
  // As loopback: the commanded payload becomes the state on enable (ICON's
  // SetPayload disables and re-enables to apply it).
  std::optional<intrinsic::RobotPayloadBase> full_payload;
  INTR_RETURN_IF_ERROR(
      intrinsic::icon::CopyTo(*payload_command_->full_payload(), full_payload));
  INTR_RETURN_IF_ERROR(intrinsic_fbs::CopyTo(
      full_payload, *payload_state_->mutable_full_payload()));
  LOG(INFO) << COMPONENT << " motion enabled payload="
            << (full_payload.has_value() ? "set" : "none");
  return absl::OkStatus();
}

absl::Status Ros2HardwareModule::DisableMotion() {
  LOG(INFO) << COMPONENT << " motion disabled";
  return absl::OkStatus();
}

absl::Status Ros2HardwareModule::ClearFaults() {
  const uint32_t epoch = latest_epoch_.load(std::memory_order_acquire);
  const uint32_t previous =
      baseline_epoch_.exchange(epoch, std::memory_order_acq_rel);
  LOG(INFO) << COMPONENT << " faults cleared epoch_baseline=" << epoch
            << " previous_baseline=" << previous;
  return absl::OkStatus();
}

absl::Status Ros2HardwareModule::Shutdown() {
  LOG(INFO) << COMPONENT << " shutting down";
  {
    absl::MutexLock lock(&queue_mutex_);
    active_ = false;
  }
  stopTickThread();
  stopRos();
  return absl::OkStatus();
}

RealtimeStatus Ros2HardwareModule::ReadStatus() {
  if (epoch_known_.load(std::memory_order_acquire) &&
      latest_epoch_.load(std::memory_order_acquire) !=
          baseline_epoch_.load(std::memory_order_acquire)) {
    if (!reset_fault_active_) {
      reset_fault_active_ = true;
      reset_faults_.fetch_add(1, std::memory_order_relaxed);
    }
    return intrinsic::icon::InternalError("simulation reset");
  }
  reset_fault_active_ = false;
  // After a reset the wrench is untared until the next retare (decision 24).
  if (const uint32_t baseline = baseline_epoch_.load(std::memory_order_acquire);
      baseline != read_baseline_epoch_) {
    read_baseline_epoch_ = baseline;
    taring_ = {};
    velocity_estimator_.reset();
  }

  TickEntry* entry = nullptr;
  if (!tick_buffer_.GetActiveBuffer(&entry)) {
    return OkStatus();  // Same entry as last time (no new tick data).
  }
  const StateSample& state = entry->state;
  const double dt_s =
      has_previous_entry_
          ? (state.stamp_ns - previous_entry_.state.stamp_ns) * 1e-9
          : 0.0;
  const std::array<double, ARM_DOF> previous_velocity =
      velocity_estimator_.velocity();
  const std::array<double, ARM_DOF>& velocity =
      velocity_estimator_.update(state.stamp_ns, state.position);
  for (int i = 0; i < ARM_DOF; ++i) {
    joint_position_state_->mutable_position()->Mutate(i, state.position[i]);
    joint_velocity_state_->mutable_velocity()->Mutate(i, velocity[i]);
    joint_acceleration_state_->mutable_acceleration()->Mutate(
        i, finiteDifference(velocity[i], previous_velocity[i], dt_s));
    joint_torque_state_->mutable_torque()->Mutate(i, state.effort[i]);
  }
  gripper_status_->mutate_width(gripperWidth(state.finger_position));

  const bool was_taring = !taring_.completed;
  const Wrench wrench = applyTaring(taring_, entry->wrench);
  if (was_taring && taring_.completed) {
    taring_ends_.fetch_add(1, std::memory_order_relaxed);
  }
  intrinsic_fbs::Wrench* out = force_torque_status_->mutable_wrench();
  out->mutate_x(wrench[0]);
  out->mutate_y(wrench[1]);
  out->mutate_z(wrench[2]);
  out->mutate_rx(wrench[3]);
  out->mutate_ry(wrench[4]);
  out->mutate_rz(wrench[5]);
  force_torque_status_->mutate_status_code(
      intrinsic_fbs::ForceSensorStatusCode::Ok);
  force_torque_status_->mutate_enabled(true);
  force_torque_status_->mutate_retare_completed(taring_.completed);

  current_stamp_ns_ = state.stamp_ns;
  previous_entry_ = *entry;
  has_previous_entry_ = true;
  return OkStatus();
}

RealtimeStatus Ros2HardwareModule::ApplyCommand() {
  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto arm, joint_position_command_.Value());
  Command* command = command_buffer_.GetFreeBuffer();
  command->stamp_ns = current_stamp_ns_;
  for (int i = 0; i < ARM_DOF; ++i) {
    command->position[i] = arm->position()->Get(i);
    command->velocity[i] = arm->velocity_feedforward()->Get(i);
  }
  command->gripper = {.width = gripper_command_->width(),
                      .force = gripper_command_->force()};
  command_buffer_.CommitFreeBuffer();

  const bool retare = force_torque_command_->retare();
  if (retare && !last_retare_) {
    startTaring(taring_, force_torque_command_->num_taring_cycles());
    force_torque_status_->mutate_retare_completed(false);
    taring_starts_.fetch_add(1, std::memory_order_relaxed);
  }
  last_retare_ = retare;
  return OkStatus();
}

void Ros2HardwareModule::onJointState(const sensor_msgs::msg::JointState& msg) {
  if (!indices_known_) {
    std::array<int, 1> finger_index{};
    if (!mapJointIndices(msg.name, arm_joint_names_, arm_indices_) ||
        !mapJointIndices(msg.name, std::span(&finger_joint_name_, 1),
                         finger_index)) {
      LOG_FIRST_N(ERROR, 1) << COMPONENT << " joint names missing in "
                            << config_.joint_states_topic();
      return;
    }
    finger_index_ = finger_index[0];
    indices_known_ = true;
  }
  StateSample state;
  state.stamp_ns = stampNs(msg.header.stamp);
  const size_t n = msg.name.size();
  for (int i = 0; i < ARM_DOF; ++i) {
    const int j = arm_indices_[i];
    state.position[i] = msg.position[j];
    state.velocity[i] = msg.velocity.size() == n ? msg.velocity[j] : 0.0;
    state.effort[i] = msg.effort.size() == n ? msg.effort[j] : 0.0;
  }
  state.finger_position = msg.position[finger_index_];
  last_state_ns_.store(steadyNowNs(), std::memory_order_relaxed);
  if (auto entry = pairer_->addState(state)) enqueue(*entry);
}

void Ros2HardwareModule::onWrench(
    const geometry_msgs::msg::WrenchStamped& msg) {
  const auto& f = msg.wrench.force;
  const auto& t = msg.wrench.torque;
  const Wrench flange = rotateHandToFlange({f.x, f.y, f.z, t.x, t.y, t.z});
  if (auto entry = pairer_->addWrench(stampNs(msg.header.stamp), flange)) {
    enqueue(*entry);
  }
}

void Ros2HardwareModule::onEpoch(const std_msgs::msg::UInt32& msg) {
  if (!epoch_known_.load(std::memory_order_acquire)) {
    latest_epoch_.store(msg.data, std::memory_order_release);
    baseline_epoch_.store(msg.data, std::memory_order_release);
    epoch_known_.store(true, std::memory_order_release);
    LOG(INFO) << COMPONENT << " epoch baseline epoch=" << msg.data;
    return;
  }
  const uint32_t previous =
      latest_epoch_.exchange(msg.data, std::memory_order_acq_rel);
  if (previous != msg.data) {
    LOG(INFO) << COMPONENT << " epoch changed from=" << previous
              << " to=" << msg.data;
  }
}

void Ros2HardwareModule::enqueue(const TickEntry& entry) {
  absl::MutexLock lock(&queue_mutex_);
  if (!active_) return;
  if (queue_.push(entry)) dropped_.fetch_add(1, std::memory_order_relaxed);
}

void Ros2HardwareModule::onStatusTimer() {
  const int64_t now = steadyNowNs();
  const bool flowing =
      now - last_state_ns_.load(std::memory_order_relaxed) <= NO_STATE_NS;
  if (flowing != states_flowing_) {
    states_flowing_ = flowing;
    LOG(INFO) << COMPONENT
              << (flowing ? " states arriving (lockstep)"
                          : " no state for > 1 s (simulation paused or absent)");
    // A restarted Isaac starts its sim time near 0 again; old unpaired stamps
    // must not pair with new ones.
    if (!flowing) pairer_->clear();
  }
  if (const uint64_t n = reset_faults_.load(std::memory_order_relaxed);
      n != logged_reset_faults_) {
    logged_reset_faults_ = n;
    LOG(WARNING) << COMPONENT << " fault reason=\"simulation reset\" epoch="
                 << latest_epoch_.load(std::memory_order_relaxed)
                 << " baseline=" << baseline_epoch_.load(std::memory_order_relaxed);
  }
  if (const uint64_t n = taring_starts_.load(std::memory_order_relaxed);
      n != logged_taring_starts_) {
    logged_taring_starts_ = n;
    LOG(INFO) << COMPONENT << " taring started";
  }
  if (const uint64_t n = taring_ends_.load(std::memory_order_relaxed);
      n != logged_taring_ends_) {
    logged_taring_ends_ = n;
    LOG(INFO) << COMPONENT << " taring done";
  }
  if (now - last_summary_ns_ >= SUMMARY_PERIOD_NS) {
    const uint64_t ticks = ticks_.load(std::memory_order_relaxed);
    const uint64_t dropped = dropped_.load(std::memory_order_relaxed);
    LOG(INFO) << COMPONENT << " summary ticks=" << ticks - summary_ticks_
              << " dropped=" << dropped - summary_dropped_
              << " ticks_total=" << ticks << " dropped_total=" << dropped
              << " epoch=" << latest_epoch_.load(std::memory_order_relaxed);
    last_summary_ns_ = now;
    summary_ticks_ = ticks;
    summary_dropped_ = dropped;
  }
}

bool Ros2HardwareModule::queueReadyOrStopping() const {
  return tick_stopping_ || !queue_.empty();
}

bool Ros2HardwareModule::tickStopping() {
  absl::MutexLock lock(&queue_mutex_);
  return tick_stopping_;
}

void Ros2HardwareModule::startTickThread() {
  {
    absl::MutexLock lock(&queue_mutex_);
    tick_stopping_ = false;
  }
  tick_thread_ = std::thread([this] { tickLoop(); });
}

void Ros2HardwareModule::stopTickThread() {
  {
    absl::MutexLock lock(&queue_mutex_);
    tick_stopping_ = true;
  }
  if (tick_thread_.joinable()) tick_thread_.join();
}

void Ros2HardwareModule::stopRos() {
  if (executor_ != nullptr) executor_->cancel();
  if (ros_thread_.joinable()) ros_thread_.join();
  if (rclcpp::ok()) rclcpp::shutdown();
}

void Ros2HardwareModule::tickLoop() {
  uint32_t tick_baseline_epoch = baseline_epoch_.load(std::memory_order_acquire);
  while (true) {
    // Isaac reopens the fingers on reset, so the next command must be sent.
    if (const uint32_t baseline =
            baseline_epoch_.load(std::memory_order_acquire);
        baseline != tick_baseline_epoch) {
      tick_baseline_epoch = baseline;
      last_gripper_.reset();
    }
    TickEntry entry;
    {
      absl::MutexLock lock(&queue_mutex_);
      if (!queue_mutex_.AwaitWithTimeout(
              absl::Condition(this, &Ros2HardwareModule::queueReadyOrStopping),
              QUEUE_WAIT)) {
        continue;
      }
      if (tick_stopping_) return;
      entry = queue_.front();
      queue_.pop();
    }
    if (!tickWithRetries(entry)) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      continue;
    }
    ticks_.fetch_add(1, std::memory_order_relaxed);
    Command* command = nullptr;
    if (command_buffer_.GetActiveBuffer(&command) &&
        command->stamp_ns == entry.state.stamp_ns) {
      publishCommand(*command);
    }
  }
}

bool Ros2HardwareModule::tickWithRetries(const TickEntry& entry) {
  for (int attempt = 0; attempt <= TICK_RETRIES; ++attempt) {
    *tick_buffer_.GetFreeBuffer() = entry;
    tick_buffer_.CommitFreeBuffer();
    const RealtimeStatus status = realtime_clock_->TickBlockingWithTimeout(
        intrinsic::Clock::FromNSec(entry.state.stamp_ns), TICK_TIMEOUT);
    if (status.ok()) return true;
    LOG_EVERY_N_SEC(WARNING, 5) << COMPONENT << " tick failed stamp_ns="
                                << entry.state.stamp_ns
                                << " attempt=" << attempt + 1 << " error=\""
                                << status.message() << "\"";
    if (tickStopping()) return false;
    if (const RealtimeStatus reset = realtime_clock_->Reset(CLOCK_RESET_TIMEOUT);
        !reset.ok()) {
      LOG(WARNING) << COMPONENT << " clock reset failed error=\""
                   << reset.message() << "\"";
    }
    absl::SleepFor(TICK_RETRY_SLEEP);
    if (tickStopping()) return false;
  }
  LOG(WARNING) << COMPONENT << " tick entry dropped stamp_ns="
               << entry.state.stamp_ns << " after " << TICK_RETRIES + 1
               << " attempts";
  return false;
}

void Ros2HardwareModule::publishCommand(const Command& command) {
  joint_command_msg_.header.stamp = toStamp(command.stamp_ns);
  for (int i = 0; i < ARM_DOF; ++i) {
    joint_command_msg_.position[i] = command.position[i];
    joint_command_msg_.velocity[i] = command.velocity[i];
  }
  joint_command_pub_->publish(joint_command_msg_);
  // Force <= 0 is ICON's unset default; Isaac would set the finger max effort
  // to it and leave the fingers without drive.
  if (command.gripper.force > 0.0 && last_gripper_ != command.gripper) {
    gripper_command_msg_.header.stamp = joint_command_msg_.header.stamp;
    gripper_command_msg_.position[0] = fingerPosition(command.gripper.width);
    gripper_command_msg_.effort[0] = command.gripper.force;
    gripper_command_pub_->publish(gripper_command_msg_);
    last_gripper_ = command.gripper;
    LOG(INFO) << COMPONENT << " gripper command width=" << command.gripper.width
              << " force=" << command.gripper.force;
  }
}

}  // namespace ros2_hwm

REGISTER_HARDWARE_MODULE(ros2_hwm::Ros2HardwareModule)
