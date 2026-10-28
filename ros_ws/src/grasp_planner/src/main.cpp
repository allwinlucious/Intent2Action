// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Ported from intrinsic-moveit v0.0.4 moveit_planning_service/src/
// moveit_planning_node.cpp (goal, cancel, accept and execute handlers; node
// set-up) for ROS 2 Humble, without the Intrinsic World synchronizer: the
// client applies the planning scene before every goal (docs/designs/phases/
// phase_5c.md decision 2).

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_planning_interfaces/action/plan_grasps.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <thread>
#include <vector>

#include "moveit_planning_service/grasp_planning_pipeline.hpp"

using PlanGraspsAction = moveit_planning_interfaces::action::PlanGrasps;
using PlanGraspsGoalHandle = rclcpp_action::ServerGoalHandle<PlanGraspsAction>;
using RankedGrasp = moveit_planning_interfaces::msg::RankedGrasp;
using GetPlanningScene = moveit_msgs::srv::GetPlanningScene;

namespace {

rclcpp::Logger Logger() { return rclcpp::get_logger("grasp_planner"); }

// One line per goal: target, candidates, grasps returned, duration, error code.
void LogGoal(const PlanGraspsAction::Goal& goal,
             const PlanGraspsAction::Result& result,
             std::chrono::steady_clock::time_point start,
             const char* outcome) {
  std::string targets;
  for (const auto& candidate : goal.candidates) {
    targets += (targets.empty() ? "" : ",") + candidate.target.id;
  }
  const double duration_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
  RCLCPP_INFO(Logger(),
              "goal %s target=%s candidates=%zu grasps=%zu duration_s=%.3f "
              "error_code=%d",
              outcome, targets.c_str(), goal.candidates.size(),
              result.ranked_grasps.size(), duration_s, result.error_code.val);
}

std::optional<std::vector<RankedGrasp>> PlanGraspsForCandidate(
    const PlanGraspsAction::Goal& goal,
    const moveit_planning_interfaces::msg::GraspCandidate& candidate,
    uint32_t candidate_index,
    const std::shared_ptr<moveit_planning_service::GraspPlanningPipeline>&
        grasp_pipeline,
    std::chrono::milliseconds service_call_timeout) {
  PlanGraspsAction::Result candidate_result;
  grasp_pipeline->PlanGrasps(goal, candidate, candidate_index, candidate_result,
                             service_call_timeout);

  if (candidate_result.error_code.val !=
      moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_WARN(Logger(),
                "Grasp planning failed for candidate '%s' with error code %d.",
                candidate.target.id.c_str(), candidate_result.error_code.val);
    return std::nullopt;
  }

  return std::move(candidate_result.ranked_grasps);
}

rclcpp_action::GoalResponse HandlePlanGraspsGoal(
    const std::shared_ptr<std::atomic<bool>>& scene_ready,
    const rclcpp_action::GoalUUID&,
    const std::shared_ptr<const PlanGraspsAction::Goal>& goal) {
  if (!scene_ready->load()) {
    RCLCPP_WARN(Logger(),
                "Rejecting grasp planning goal: collision scene is not ready.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->candidates.empty()) {
    RCLCPP_WARN(Logger(),
                "Rejecting grasp planning goal: candidates cannot be empty.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  for (const auto& candidate : goal->candidates) {
    if (candidate.target.id.empty()) {
      RCLCPP_WARN(Logger(),
                  "Rejecting grasp planning goal: every candidate must have a "
                  "target ID.");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  RCLCPP_INFO(Logger(),
              "Accepted grasp planning goal with %zu candidate objects",
              goal->candidates.size());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse HandlePlanGraspsCancel(
    const std::shared_ptr<PlanGraspsGoalHandle>&) {
  RCLCPP_INFO(Logger(), "Received request to cancel grasp planning goal.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ExecutePlanGraspsAction(
    const std::shared_ptr<PlanGraspsGoalHandle>& goal_handle,
    const std::shared_ptr<moveit_planning_service::GraspPlanningPipeline>&
        grasp_pipeline,
    std::chrono::milliseconds service_call_timeout) {
  const auto start = std::chrono::steady_clock::now();
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<PlanGraspsAction::Result>();
  auto feedback = std::make_shared<PlanGraspsAction::Feedback>();

  feedback->total_candidates = static_cast<uint32_t>(goal->candidates.size());
  feedback->completed_candidates = 0;

  for (std::size_t candidate_index = 0;
       candidate_index < goal->candidates.size(); ++candidate_index) {
    if (goal_handle->is_canceling()) {
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
      goal_handle->canceled(result);
      LogGoal(*goal, *result, start, "canceled");
      return;
    }

    auto candidate_grasps =
        PlanGraspsForCandidate(*goal, goal->candidates[candidate_index],
                               static_cast<uint32_t>(candidate_index),
                               grasp_pipeline, service_call_timeout);

    if (!candidate_grasps.has_value()) {
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      goal_handle->abort(result);
      LogGoal(*goal, *result, start, "aborted");
      return;
    }

    result->ranked_grasps.insert(result->ranked_grasps.end(),
                                 candidate_grasps->begin(),
                                 candidate_grasps->end());

    feedback->completed_candidates = static_cast<uint32_t>(candidate_index + 1);
    goal_handle->publish_feedback(feedback);
  }

  std::sort(result->ranked_grasps.begin(), result->ranked_grasps.end(),
            [](const RankedGrasp& lhs, const RankedGrasp& rhs) {
              return lhs.grasp.grasp_quality > rhs.grasp.grasp_quality;
            });

  const std::size_t max_num_grasps =
      goal->max_num_grasps > 0 ? static_cast<std::size_t>(goal->max_num_grasps)
                               : 1;

  if (result->ranked_grasps.size() > max_num_grasps) {
    result->ranked_grasps.resize(max_num_grasps);
  }

  if (result->ranked_grasps.empty()) {
    result->error_code.val =
        moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    goal_handle->abort(result);
    LogGoal(*goal, *result, start, "aborted");
    return;
  }

  result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  goal_handle->succeed(result);
  LogGoal(*goal, *result, start, "succeeded");
}

void HandlePlanGraspsAccepted(
    const std::shared_ptr<PlanGraspsGoalHandle>& goal_handle,
    const std::shared_ptr<moveit_planning_service::GraspPlanningPipeline>&
        grasp_pipeline,
    std::chrono::milliseconds service_call_timeout) {
  std::thread([goal_handle, grasp_pipeline, service_call_timeout]() {
    ExecutePlanGraspsAction(goal_handle, grasp_pipeline, service_call_timeout);
  }).detach();
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Humble's MoveIt reads the launch-provided parameters (robot description,
  // kinematics, OMPL pipeline) only if they are declared.
  auto node = rclcpp::Node::make_shared(
      "grasp_planner",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  if (!node->has_parameter("ros_service_call_timeout_sec")) {
    rcl_interfaces::msg::ParameterDescriptor double_desc;
    double_desc.dynamic_typing = true;
    node->declare_parameter("ros_service_call_timeout_sec",
                            rclcpp::ParameterValue(5.0), double_desc);
  }

  // robot_service applies the planning scene before every goal.
  auto scene_ready = std::make_shared<std::atomic<bool>>(true);

  // Dedicated callback group & executor for /get_planning_scene queries
  auto planning_scene_callback_group = node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);

  auto planning_scene_client = node->create_client<GetPlanningScene>(
      "/get_planning_scene", rmw_qos_profile_services_default,
      planning_scene_callback_group);

  auto planning_scene_executor =
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  planning_scene_executor->add_callback_group(planning_scene_callback_group,
                                              node->get_node_base_interface());
  std::thread planning_scene_executor_thread(
      [planning_scene_executor]() { planning_scene_executor->spin(); });

  auto planning_services_callback_group =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto grasp_pipeline =
      std::make_shared<moveit_planning_service::GraspPlanningPipeline>(
          node, planning_scene_client);

  auto grasp_action_server = rclcpp_action::create_server<PlanGraspsAction>(
      node, "grasp_planning/plan_grasps",
      [scene_ready](const rclcpp_action::GoalUUID& uuid,
                    const std::shared_ptr<const PlanGraspsAction::Goal>& goal) {
        return HandlePlanGraspsGoal(scene_ready, uuid, goal);
      },
      [](const std::shared_ptr<PlanGraspsGoalHandle>& goal_handle) {
        return HandlePlanGraspsCancel(goal_handle);
      },
      [node, grasp_pipeline](
          const std::shared_ptr<PlanGraspsGoalHandle>& goal_handle) {
        double timeout_sec = 5.0;
        node->get_parameter("ros_service_call_timeout_sec", timeout_sec);
        const auto service_call_timeout = std::chrono::milliseconds(
            static_cast<int64_t>(timeout_sec * 1000.0));
        HandlePlanGraspsAccepted(goal_handle, grasp_pipeline,
                                 service_call_timeout);
      },
      rcl_action_server_get_default_options(),
      planning_services_callback_group);

  RCLCPP_INFO(node->get_logger(),
              "grasp planner started: grasp_planning/plan_grasps (action)");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  planning_scene_executor->cancel();
  if (planning_scene_executor_thread.joinable()) {
    planning_scene_executor_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}
