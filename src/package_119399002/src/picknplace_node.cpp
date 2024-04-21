/**
 * @file picknplace_node.cpp
 * @author Abhishekh Reddy (areddy42@umd.edu)
 * @brief MoveIt! node for Project 4
 * @version 1.0
 * @date 2024-04-21
 *
 * @copyright Copyright (c) 2024 Abhishekh
 *
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("picknplace");

int main(int argc, char** argv)
{
  // Initialize and spin a ROS Node.
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node {
    rclcpp::Node::make_shared("picknplace_node", node_options)};

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Initialization - Declare some reusable variables and open the hand
  moveit::planning_interface::MoveGroupInterface panda_arm_group {
    move_group_node, "panda_arm"};

  moveit::planning_interface::MoveGroupInterface hand_group {
    move_group_node, "hand"};

  std::vector<geometry_msgs::msg::Pose> cartesian_waypoints;
  moveit_msgs::msg::RobotTrajectory cartesian_trajectory;

  hand_group.setNamedTarget("open_hand");
  hand_group.move();

  // Move to goal 1   ----------------------------------------------------------
  RCLCPP_INFO(LOGGER, " -- Moving to goal 1 -- ");

  cartesian_waypoints.clear();
  auto goal1 {panda_arm_group.getCurrentPose().pose};
  goal1.position.x += 0.1;
  goal1.position.y -= 0.2;
  goal1.position.z -= 0.5;
  cartesian_waypoints.push_back(goal1);

  panda_arm_group.computeCartesianPath(
    cartesian_waypoints, 0.01, 0.0, cartesian_trajectory);

  panda_arm_group.execute(cartesian_trajectory);

  // Close hand at goal 1   ----------------------------------------------------
  hand_group.setNamedTarget("close_hand");
  hand_group.move();

  // Move to goal 2   ----------------------------------------------------------
  RCLCPP_INFO(LOGGER, " -- Moving to goal 2 -- ");

  cartesian_waypoints.clear();
  auto goal2_mid {panda_arm_group.getCurrentPose().pose};
  goal2_mid.position.y += 0.2;
  goal2_mid.position.z += 0.5;
  cartesian_waypoints.push_back(goal2_mid);

  auto goal2 {goal2_mid};
  goal2.position.x += 0.2;
  goal2.position.y += 0.3;
  goal2.position.z -= 0.4;
  {
    tf2::Quaternion tf2_orientation;
    tf2_orientation.setRPY(0.0, (90 * M_PI / 180), 0.0);

    goal2.orientation = tf2::toMsg(tf2_orientation);
  }
  cartesian_waypoints.push_back(goal2);

  panda_arm_group.computeCartesianPath(
    cartesian_waypoints, 0.01, 0.0, cartesian_trajectory);

  panda_arm_group.execute(cartesian_trajectory);

  // Open hand at goal 2
  hand_group.setNamedTarget("open_hand");
  hand_group.move();

  // Back to initial pose
  panda_arm_group.setNamedTarget("home_pose");
  panda_arm_group.move();

  hand_group.setNamedTarget("close_hand");
  hand_group.move();

  rclcpp::shutdown();
  return 0;
}
