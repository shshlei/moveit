/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#include <moveit/ompl_interface/detail/safety_certificate.h>
#include <moveit/ompl_interface/model_based_planning_context.h>

namespace ompl_interface
{
constexpr char LOGNAME[] = "safety_certificate";
}

ompl_interface::SafetyCertificate::SafetyCertificate(const ModelBasedPlanningContext* pc)
  : planning_context_(pc)
  , group_name_(pc->getGroupName())
  , tss_(pc->getCompleteInitialRobotState())
  , cdata_(planning_context_->getPlanningScene()->getSafetyDistance(),
           planning_context_->getPlanningScene()->getContactDistanceThreshold(), 1.0,
           &planning_context_->getPlanningScene()->getAllowedCollisionMatrix(), &req_, &res_)
{
  collision_safety_margin_ = planning_context_->getPlanningScene()->getSafetyDistance();
  certificateDim_ = planning_context_->getJointModelGroup()->getJointModelNames().size();

  if (planning_context_->getPlanningScene()->getCollisionEnv()->getCollisionName() == "Bullet")
    manager_bullet_ = std::make_shared<collision_detection_bullet::BulletDiscreteBVHManager>();
  else if (planning_context_->getPlanningScene()->getCollisionEnv()->getCollisionName() == "FCL")
    manager_fcl_ = std::make_shared<collision_detection_fcl::FclDiscreteBVHManager>();
}

bool ompl_interface::SafetyCertificate::collisionCertificate(
    const ompl::base::State* state, const std::vector<ompl::base::SafetyCertificate*>& ocv) const
{
  moveit::core::RobotState* robot_state = tss_.getStateStorage();
  planning_context_->getOMPLStateSpace()->copyToRobotState(*robot_state, state);

  bool osc = false;

  for (const auto& oc : ocv)
  {
    const auto& ocontact_vector = *(oc->contact);
    bool check = true;
    for (const auto& res : ocontact_vector)
    {
      if (res.distance < 0.0 || check)
      {
        std::string name_a = res.link_names[0], name_b = res.link_names[1];
        int type_id_a = res.type_id[0], type_id_b = res.type_id[1];

        Eigen::Isometry3d link_transform_a = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d link_transform_b = Eigen::Isometry3d::Identity();

        if (type_id_a == 0)
          link_transform_a = planning_context_->getPlanningScene()->getWorld()->getTransform(name_a);
        else if (type_id_a == 1)
          link_transform_a = robot_state->getGlobalLinkTransform(name_a);
        else if (type_id_a == 2)
          link_transform_a = robot_state->getAttachedBody(name_a)->getGlobalPose();

        if (type_id_b == 0)
          link_transform_b = planning_context_->getPlanningScene()->getWorld()->getTransform(name_b);
        else if (type_id_b == 1)
          link_transform_b = robot_state->getGlobalLinkTransform(name_b);
        else if (type_id_b == 2)
          link_transform_b = robot_state->getAttachedBody(name_b)->getGlobalPose();

        if (res.distance < 0.0)
        {
          Eigen::Vector3d pointa =
              type_id_a == 0 ? res.nearest_points[1] : link_transform_a * res.nearest_points_local2[0];
          Eigen::Vector3d pointb =
              type_id_b == 0 ? res.nearest_points[1] : link_transform_b * res.nearest_points_local[1];

          Eigen::Vector3d pointa_2 =
              type_id_a == 0 ? res.nearest_points[0] : link_transform_a * res.nearest_points_local[0];
          Eigen::Vector3d pointb_2 =
              type_id_b == 0 ? res.nearest_points[0] : link_transform_b * res.nearest_points_local2[1];

          double dist = (pointa - pointb).norm(), dist_2 = (pointa_2 - pointb_2).norm();
          if (dist + res.distance <= 2.0 * collision_safety_margin_ &&
              dist_2 + res.distance <= 2.0 * collision_safety_margin_)
          {
            osc = true;
            break;
          }
        }
        else
        {
          Eigen::Vector3d pointa =
              type_id_a == 0 ? res.nearest_points[0] : link_transform_a * res.nearest_points_local[0];
          Eigen::Vector3d pointb =
              type_id_b == 0 ? res.nearest_points[1] : link_transform_b * res.nearest_points_local[1];

          double dist = (pointa - pointb).norm();
          if (dist <= 2.0 * collision_safety_margin_)
          {
            osc = true;
            break;
          }
        }

        if (check && (manager_fcl_ || manager_bullet_))
        {
          auto manager = planning_context_->getPlanningScene()->getCollisionEnv()->getCollisionBVHManager().get();

          if (manager_fcl_)
          {
            auto object_a = manager->as<collision_detection_fcl::FclDiscreteBVHManager>()->getCollisionObject(name_a);
            auto object_b = manager->as<collision_detection_fcl::FclDiscreteBVHManager>()->getCollisionObject(name_b);

            if (object_a->getShapeCount() > 1u)
            {
              if (res.shape_id[0] != -1)
                manager_fcl_->addCollisionObject(object_a->createSubObject(res.shape_id[0], type_id_a != 0));
              else
                continue;
            }
            else
            {
              manager_fcl_->addCollisionObject(object_a);
            }

            if (object_b->getShapeCount() > 1u)
            {
              if (res.shape_id[1] != -1)
                manager_fcl_->addCollisionObject(object_b->createSubObject(res.shape_id[1], type_id_b != 0));
              else
              {
                manager_fcl_->removeCollisionObject(name_a);
                continue;
              }
            }
            else
              manager_fcl_->addCollisionObject(object_b);

            manager_fcl_->setCollisionObjectsTransform(name_a, link_transform_a);
            manager_fcl_->setCollisionObjectsTransform(name_b, link_transform_b);

            res_.clear();
            manager_fcl_->contactTest(cdata_, type_id_a != 0 && type_id_b != 0);

            manager_fcl_->removeCollisionObject(name_a);
            manager_fcl_->removeCollisionObject(name_b);
          }
          else
          {
            auto object_a =
                manager->as<collision_detection_bullet::BulletDiscreteBVHManager>()->getCollisionObject(name_a);
            auto object_b =
                manager->as<collision_detection_bullet::BulletDiscreteBVHManager>()->getCollisionObject(name_b);

            if (object_a->getShapeCount() > 1u)
            {
              if (res.shape_id[0] != -1)
                manager_bullet_->addCollisionObject(object_a->createSubObject(res.shape_id[0], type_id_a != 0));
              else
                continue;
            }
            else
            {
              manager_bullet_->addCollisionObject(object_a);
            }

            if (object_b->getShapeCount() > 1u)
            {
              if (res.shape_id[1] != -1)
                manager_bullet_->addCollisionObject(object_b->createSubObject(res.shape_id[1], type_id_b != 0));
              else
              {
                manager_bullet_->removeCollisionObject(name_a);
                continue;
              }
            }
            else
              manager_bullet_->addCollisionObject(object_b);

            manager_bullet_->setCollisionObjectsTransform(name_a, link_transform_a);
            manager_bullet_->setCollisionObjectsTransform(name_b, link_transform_b);

            res_.clear();
            manager_bullet_->contactTest(cdata_, type_id_a != 0 && type_id_b != 0);

            manager_bullet_->removeCollisionObject(name_a);
            manager_bullet_->removeCollisionObject(name_b);
          }

          check = false;
          if (res_.collision)
          {
            osc = true;
            break;
          }
        }
      }
    }

    if (osc)
      break;
  }

  return osc;
}

bool ompl_interface::SafetyCertificate::safetyCertificate(const ompl::base::State* state,
                                                          const ompl::base::SafetyCertificate* sc,
                                                          std::vector<double>& dist) const
{
  bool fsc = true;

  dist = distanceCertificate(sc->state, state);

  for (std::size_t i = 0; i < certificateDim_; i++)
  {
    if (dist[i] > sc->confidence_[i])
    {
      fsc = false;
      break;
    }
  }

  return fsc;
}

std::vector<double> ompl_interface::SafetyCertificate::distanceCertificate(const ompl::base::State* a,
                                                                           const ompl::base::State* b) const
{
  moveit::core::RobotState* robot_state = tss_.getStateStorage();

  moveit::core::RobotState robot_state_a(*robot_state), robot_state_b(*robot_state);
  planning_context_->getOMPLStateSpace()->copyToRobotStateWithoutUpdate(robot_state_a, a);
  planning_context_->getOMPLStateSpace()->copyToRobotStateWithoutUpdate(robot_state_b, b);

  std::vector<double> dist(certificateDim_);

  const auto joints = planning_context_->getJointModelGroup()->getJointModels();

  for (std::size_t i = 0; i < certificateDim_; i++)
  {
    dist[i] = robot_state_a.distance(robot_state_b, joints[i]);
  }

  return dist;
}
