#include "ext/ros2_hwm/ros2_hwm_logic.h"

#include <numbers>

namespace ros2_hwm {
namespace {

constexpr double SQRT1_2 = std::numbers::sqrt2 / 2.0;

}  // namespace

Wrench rotateHandToFlange(const Wrench& hand) {
  // Rz(-pi/4) = [[c, s, 0], [-s, c, 0], [0, 0, 1]] with c = s = 1/sqrt(2).
  Wrench flange{};
  for (int offset : {0, 3}) {
    const double x = hand[offset], y = hand[offset + 1];
    flange[offset] = SQRT1_2 * (x + y);
    flange[offset + 1] = SQRT1_2 * (y - x);
    flange[offset + 2] = hand[offset + 2];
  }
  return flange;
}

double finiteDifference(double velocity, double previous_velocity,
                        double dt_s) {
  return dt_s > 0.0 ? (velocity - previous_velocity) / dt_s : 0.0;
}

const std::array<double, ARM_DOF>& VelocityEstimator::update(
    int64_t stamp_ns, const std::array<double, ARM_DOF>& position) {
  if (has_previous_ && stamp_ns > previous_stamp_ns_) {
    const double dt_s = (stamp_ns - previous_stamp_ns_) * 1e-9;
    for (int i = 0; i < ARM_DOF; ++i) {
      velocity_[i] = 0.5 * velocity_[i] +
                     0.5 * (position[i] - previous_position_[i]) / dt_s;
    }
  }
  if (!has_previous_ || stamp_ns > previous_stamp_ns_) {
    previous_stamp_ns_ = stamp_ns;
    previous_position_ = position;
    has_previous_ = true;
  }
  return velocity_;
}

void VelocityEstimator::reset() {
  has_previous_ = false;
  velocity_.fill(0.0);
}

bool mapJointIndices(std::span<const std::string> names,
                     std::span<const std::string> wanted,
                     std::span<int> indices) {
  for (size_t w = 0; w < wanted.size(); ++w) {
    indices[w] = -1;
    for (size_t n = 0; n < names.size(); ++n) {
      if (names[n] == wanted[w]) {
        indices[w] = static_cast<int>(n);
        break;
      }
    }
    if (indices[w] < 0) return false;
  }
  return true;
}

std::optional<TickEntry> StampPairer::addState(const StateSample& state) {
  for (size_t i = 0; i < CAPACITY; ++i) {
    if (wrench_used_[i] &&
        stampsMatch(wrenches_[i].stamp_ns, state.stamp_ns, tolerance_ns_)) {
      wrench_used_[i] = false;
      return TickEntry{.state = state, .wrench = wrenches_[i].wrench};
    }
  }
  states_[next_state_] = state;
  state_used_[next_state_] = true;
  next_state_ = (next_state_ + 1) % CAPACITY;
  return std::nullopt;
}

std::optional<TickEntry> StampPairer::addWrench(int64_t stamp_ns,
                                                const Wrench& wrench) {
  for (size_t i = 0; i < CAPACITY; ++i) {
    if (state_used_[i] &&
        stampsMatch(states_[i].stamp_ns, stamp_ns, tolerance_ns_)) {
      state_used_[i] = false;
      return TickEntry{.state = states_[i], .wrench = wrench};
    }
  }
  wrenches_[next_wrench_] = {.stamp_ns = stamp_ns, .wrench = wrench};
  wrench_used_[next_wrench_] = true;
  next_wrench_ = (next_wrench_ + 1) % CAPACITY;
  return std::nullopt;
}

void StampPairer::clear() {
  state_used_.fill(false);
  wrench_used_.fill(false);
}

void startTaring(TaringState& taring, int num_cycles) {
  taring.completed = false;
  taring.cycles = num_cycles > 0 ? num_cycles : 1;
  taring.remaining = taring.cycles;
  taring.sum.fill(0.0);
}

Wrench applyTaring(TaringState& taring, const Wrench& reading) {
  if (!taring.completed) {
    for (size_t i = 0; i < reading.size(); ++i) taring.sum[i] += reading[i];
    if (--taring.remaining == 0) {
      for (size_t i = 0; i < reading.size(); ++i) {
        taring.bias[i] = taring.sum[i] / taring.cycles;
      }
      taring.completed = true;
    }
  }
  Wrench tared{};
  for (size_t i = 0; i < reading.size(); ++i) {
    tared[i] = reading[i] - taring.bias[i];
  }
  return tared;
}

}  // namespace ros2_hwm
