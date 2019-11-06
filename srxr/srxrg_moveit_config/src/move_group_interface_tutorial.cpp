/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: WangXF siasun */

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "dh_gripper_services/CtrlGripper.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "arms";
  moveit::planning_interface::MoveGroup move_group(PLANNING_GROUP);

  static const std::string PLANNER = "RRTConnect";
  move_group.setPlannerId(PLANNER);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  ros::ServiceClient client = node_handle.serviceClient<dh_gripper_services::CtrlGripper>("/ctrlGripper");

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in Rviz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link", "/moveit_visual_markers");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in Rviz
  visual_tools.loadRemoteControl();

  // Rviz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.5; // above head of PR2
  visual_tools.publishText(text_pose, "MoveGroup Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to Rviz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 1.8;
  primitive.dimensions[1] = 1.8;
  primitive.dimensions[2] = 0.02;
  shape_msgs::SolidPrimitive primitive2;
  primitive2.type = primitive.BOX;
  primitive2.dimensions.resize(3);
  primitive2.dimensions[0] = 1.8;
  primitive2.dimensions[1] = 0.5;
  primitive2.dimensions[2] = 0.5;
  shape_msgs::SolidPrimitive primitive3;
  primitive3.type = primitive.BOX;
  primitive3.dimensions.resize(3);
  primitive3.dimensions[0] = 0.3;
  primitive3.dimensions[1] = 0.3;
  primitive3.dimensions[2] = 0.3;

  //Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.0;
  box_pose.position.y = 0.0;
  box_pose.position.z = -0.01;
  geometry_msgs::Pose box_pose2;
  box_pose2.orientation.x = 0.0;
  box_pose2.orientation.y = 0.0;
  box_pose2.orientation.z = 0.4226;
  box_pose2.orientation.w = -0.9063;
  box_pose2.position.x = 0.7;
  box_pose2.position.y = 0.7;
  box_pose2.position.z = 0.25;
  geometry_msgs::Pose box_pose3;
  box_pose3.orientation.x = 0.0;
  box_pose3.orientation.y = 0.0;
  box_pose3.orientation.z = 0.4226;
  box_pose3.orientation.w = -0.9063;
  box_pose3.position.x = 0.35;
  box_pose3.position.y = -0.55;
  box_pose3.position.z = 0.15;

  collision_object.primitives.push_back(primitive);
  collision_object.primitives.push_back(primitive2);
  collision_object.primitives.push_back(primitive3);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.primitive_poses.push_back(box_pose2);
  collision_object.primitive_poses.push_back(box_pose3);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  dh_gripper_services::CtrlGripper srv;
  srv.request.force = 50;
  srv.request.pos = 60;
  if (client.call(srv)){ ROS_INFO("Open Gripper!");}
  else
  {
   ROS_ERROR("Failed to call service dh_gripper");
//   return 1;
  }
  // home
  int times = 0;
  while(node_handle.ok())
{
  times++;
  ROS_INFO("times=%d", times);
  //visual_tools.prompt("next step");
//  ros::Duration(2).sleep();

  bool success;
  moveit::planning_interface::MoveGroup::Plan my_plan;

   /*****************FIRST*******************/
  #if 1
  move_group.setNamedTarget("home");

  //success = move_group.plan(my_plan);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 0 (pose goal) %s", success ? "" : "FAILED");

  //for(int i = 0; i < my_plan.trajectory_.joint_trajectory.points.size(); i++)
  //{
  //ROS_INFO("[%d]=%f", i, my_plan.trajectory_.joint_trajectory.points[i].positions[0]);
  //}
  //success = move_group.execute(my_plan);
  success = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "execute_home 0 %s", success ? "" : "FAILED");
  if (!success)
    break;
  ros::Duration(2).sleep();
#endif
  //move_group.move();
  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  //visual_tools.prompt("next step");

  /*****************END*******************/


  /************joint value POINT****************/

#if 1
  //PLAN  JOINT SPACE
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = joint_group_positions[0]+0.5;  // radians
  //joint_group_positions[1] = 0.2;  // radians
  //joint_group_positions[2] = -0.7;  // radians
  //joint_group_positions[3] = 0.2; 
  //joint_group_positions[4] = 0.1; 
  //joint_group_positions[5] = 0.13; 
  move_group.setJointValueTarget(joint_group_positions);

  //success = move_group.plan(my_plan);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  //visual_tools.prompt("next step");

  //move_group.move();
  //for(int i = 0; i < my_plan.trajectory_.joint_trajectory.points.size(); i++)
  //{
  //ROS_INFO("[%d]=%f", i, my_plan.trajectory_.joint_trajectory.points[i].positions[0]);
  //}
  //success = move_group.execute(my_plan);
  success = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "execute Joint Space %s", success ? "SUCCESS" : "FAILED");
  if (!success)
    break;
  ros::Duration(2).sleep();
  //visual_tools.prompt("next step");

#endif

#if 1
  //ros::Duration(2).sleep();
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = 0.5;
  target_pose1.orientation.y = 0.5;
  target_pose1.orientation.z = -0.5;
  target_pose1.orientation.w = -0.5;
  target_pose1.position.x = 0.4;
  target_pose1.position.y = 0.0;
  target_pose1.position.z = 0.4;

  move_group.setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (!success)
    break;
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in Rviz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal1", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  //visual_tools.prompt("next step");
  success = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (!success)
    break;
  ros::Duration(2).sleep();

  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "Finger_base";
  ocm.header.frame_id = "base_link";
  ocm.orientation.x = 0.5;
  ocm.orientation.y = 0.5;
  ocm.orientation.z = -0.5;
  ocm.orientation.w = -0.5;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);

  geometry_msgs::Pose target_pose2;
  target_pose2.orientation.x = 0.5;
  target_pose2.orientation.y = 0.5;
  target_pose2.orientation.z = -0.5;
  target_pose2.orientation.w = -0.5;
  target_pose2.position.x = 0.4;
  target_pose2.position.y = 0;
  target_pose2.position.z = 0.3;

//  robot_state::RobotState start_state(*move_group.getCurrentState());
//  start_state.setFromIK(joint_model_group, target_pose1);
//  move_group.setStartState(start_state);

  move_group.setPoseTarget(target_pose2);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.

  move_group.setPlanningTime(10.0);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (!success)
    break;
//  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in Rviz.
//  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
//  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(target_pose1, "target_pose1");
  visual_tools.publishAxisLabeled(target_pose2, "target_pose2");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  success = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "constraints execute position %s", success ? "" : "FAILED");

  if (!success)
    break;

  srv.request.force = 50;
  srv.request.pos = 0;
  if (client.call(srv)){ ROS_INFO("Open Gripper!");}
  else
  {
   ROS_ERROR("Failed to call service dh_gripper");
//   return 1;
  }
  ros::Duration(2).sleep();

  move_group.clearPathConstraints();
  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // Moving to a pose goal is similar to the step above
  // except we now use the move() function. Note that
  // the pose goal we had set earlier is still active
  // and so the robot will try to move to that goal. We will
  // not use that function in this tutorial since it is
  // a blocking function and requires a controller to be active
  // and report success on execution of a trajectory.
#endif

}
  // home
  //move_group.setNamedTarget("home");
  //move_group.move();

  ros::shutdown();
  return 0;
}
