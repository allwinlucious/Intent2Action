#ifndef EXT_ROS2_HWM_ROS2_HWM_LOGIC_H_
#define EXT_ROS2_HWM_ROS2_HWM_LOGIC_H_

// Pure, allocation-free logic of the ros2_hwm hardware module (phase_3.md,
// decisions 9-11): no ROS, no ICON, so it is unit-tested on its own.

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace ros2_hwm {

inline constexpr int ARM_DOF = 7;

// Force x, y, z (N), then torque x, y, z (N m).
using Wrench = std::array<double, 6>;

struct StateSample {
  int64_t stamp_ns = 0;
  std::array<double, ARM_DOF> position{};
  std::array<double, ARM_DOF> velocity{};
  std::array<double, ARM_DOF> effort{};
  double finger_position = 0.0;
};

// One ICON tick: a /joint_states and a /wrist_wrench with matching stamps.
struct TickEntry {
  StateSample state;
  Wrench wrench{};  // In the flange frame (rotateHandToFlange).
};

struct GripperCommand {
  double width = 0.0;
  double force = 0.0;
  friend bool operator==(const GripperCommand&,
                         const GripperCommand&) = default;
};

// Isaac's panda_hand frame is the flange rotated -pi/4 about z, so a vector
// in it is v_flange = Rz(-pi/4) * v (decision 9). Sign is unchanged.
Wrench rotateHandToFlange(const Wrench& hand);

inline double gripperWidth(double finger_position) {
  return 2.0 * finger_position;
}
inline double fingerPosition(double gripper_width) {
  return 0.5 * gripper_width;
}

// (velocity - previous_velocity) / dt, 0 when dt is not positive.
double finiteDifference(double velocity, double previous_velocity,
                        double dt_s);

inline bool stampsMatch(int64_t a_ns, int64_t b_ns, int64_t tolerance_ns) {
  const int64_t d = a_ns - b_ns;
  return d <= tolerance_ns && -d <= tolerance_ns;
}

// For each name in `wanted`, its index in `names` (written to `indices`).
// Returns false if one is missing.
bool mapJointIndices(std::span<const std::string> names,
                     std::span<const std::string> wanted,
                     std::span<int> indices);

// Pairs states and wrenches by stamp; keeps the latest CAPACITY unpaired ones
// of each kind (older unpaired ones can never pair any more).
class StampPairer {
 public:
  static constexpr size_t CAPACITY = 4;

  explicit StampPairer(int64_t tolerance_ns) : tolerance_ns_(tolerance_ns) {}

  std::optional<TickEntry> addState(const StateSample& state);
  std::optional<TickEntry> addWrench(int64_t stamp_ns, const Wrench& wrench);
  void clear();

 private:
  struct StampedWrench {
    int64_t stamp_ns = 0;
    Wrench wrench{};
  };

  int64_t tolerance_ns_;
  std::array<StateSample, CAPACITY> states_{};
  std::array<bool, CAPACITY> state_used_{};
  size_t next_state_ = 0;
  std::array<StampedWrench, CAPACITY> wrenches_{};
  std::array<bool, CAPACITY> wrench_used_{};
  size_t next_wrench_ = 0;
};

// Fixed-size FIFO; pushing into a full queue drops the oldest entry.
template <typename T, size_t N>
class DropOldestQueue {
 public:
  // Returns true if an entry was dropped to make room.
  bool push(const T& value) {
    const bool full = size_ == N;
    if (full) {
      head_ = (head_ + 1) % N;
      --size_;
    }
    items_[(head_ + size_) % N] = value;
    ++size_;
    return full;
  }
  const T& front() const { return items_[head_]; }
  void pop() {
    head_ = (head_ + 1) % N;
    --size_;
  }
  bool empty() const { return size_ == 0; }
  size_t size() const { return size_; }
  void clear() { head_ = size_ = 0; }

 private:
  std::array<T, N> items_{};
  size_t head_ = 0;
  size_t size_ = 0;
};

// Device-side taring (decision 10): averages the next `cycles` readings into a
// bias, which is subtracted from every reading from then on.
struct TaringState {
  bool completed = true;
  int remaining = 0;
  int cycles = 0;
  Wrench sum{};
  Wrench bias{};
};

void startTaring(TaringState& taring, int num_cycles);

// Feeds one reading; returns reading - bias (the bias in effect after it).
Wrench applyTaring(TaringState& taring, const Wrench& reading);

}  // namespace ros2_hwm

#endif  // EXT_ROS2_HWM_ROS2_HWM_LOGIC_H_
