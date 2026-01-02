#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/utils/moveit_error_code.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("rm_planner");

bool setRandomValidTarget(moveit::planning_interface::MoveGroupInterface& move_group,
                          const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
{
  const std::string group_name = move_group.getName();  // planning group

  planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor);  // safe access
  if (!scene)
  {
    RCLCPP_ERROR(LOGGER, "No planning scene.");
    return false;
  }

  // get a copy of current state
  moveit::core::RobotState state(scene->getCurrentState());
  const moveit::core::JointModelGroup* joint_model_group = state.getJointModelGroup(group_name);
  if (!joint_model_group)
  {
    RCLCPP_ERROR(LOGGER, "Group %s not found in robot state.", group_name.c_str());
    return false;
  }

  constexpr int MAX_ATTEMPTS = 100;
  for (int i = 0; i < MAX_ATTEMPTS; ++i)
  {
    state.setToRandomPositions(joint_model_group);
    state.update();

    if (scene->isStateValid(state, group_name))
    {
      // hand the sampled joint state to MoveGroup
      move_group.setJointValueTarget(state);
      return true;
    }
  }

  RCLCPP_WARN(LOGGER, "Unable to find a random collision-free configuration after %d attempts", MAX_ATTEMPTS);
  return false;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rm_planner");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  const std::string PLANNING_GROUP = "lite6";  // change to your group
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  move_group.setPlanningTime(5.0);

  // new-style construction in Humble
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  auto robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(node, "robot_description");

  auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node, robot_model_loader, tf_buffer, "planning_scene_monitor");

  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor();
  planning_scene_monitor->startStateMonitor();

  if (!setRandomValidTarget(move_group, planning_scene_monitor))
  {
    RCLCPP_ERROR(LOGGER, "Failed to set random valid target.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "Plan success. Executing.");
    move_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning failed.");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
