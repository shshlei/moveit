/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Shi Shenglei */

#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>

#include <moveit/kinematic_constraints/utils.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_arm_kinematics");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

  const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
  robot_state.setToRandomPositions();
  std::vector<double> joint_values = { 0.0, 0.0, 0.0, -2.9, 0.0, 1.4, 0.0 };
  const moveit::core::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("panda_arm");
  robot_state.setJointGroupPositions(joint_model_group, joint_values);
  ROS_INFO_STREAM("Test: Current state " << (robot_state.satisfiesBounds(joint_model_group) ?
                                                 "satisfies bounds" :
                                                 "does not satisfies bounds"));

  collision_detection::CollisionEnvBullet env(robot_model);

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.contacts = true;
  collision_request.max_contacts = 1000;

  collision_detection::AllowedCollisionMatrix acm =
      collision_detection::AllowedCollisionMatrix(*robot_model->getSRDF().get());

  env.checkCollision(collision_request, collision_result, *(robot_state.get()), acm);
  ROS_INFO_STREAM("Test 5: Current state is " << (collision_result.collision ? "in" : "not in") << " collision");

  ros::shutdown();
  return 0;
}
