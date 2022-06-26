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

#pragma once

#include <moveit/collision_detection_fcl_new/fcl_integration/fcl_discrete_bvh_manager.h>
#include <moveit/collision_detection_bullet/bullet_integration/bullet_discrete_bvh_manager.h>

#include <moveit/ompl_interface/detail/threadsafe_state_storage.h>
#include <ompl/base/SafetyCertificate.h>

namespace ompl_interface
{
class ModelBasedPlanningContext;

MOVEIT_CLASS_FORWARD(SafetyCertificate);  // Defines SafetyCertificatePtr, ConstPtr, WeakPtr... etc

/** @class SafetyCertificate
  @brief Realize the safety certificate method*/
class SafetyCertificate
{
public:
  using SafetyCertificateChecker = std::function<bool(
      const ompl::base::State* state, const ompl::base::SafetyCertificate* sc, std::vector<double>& dist)>;

  using CollisionCertificateChecker =
      std::function<bool(const ompl::base::State* state, const std::vector<ompl::base::SafetyCertificate*>& ocv)>;

  using DistanceCertificate =
      std::function<std::vector<double>(const ompl::base::State* a, const ompl::base::State* b)>;

  SafetyCertificate(const ModelBasedPlanningContext* planning_context);

  bool collisionCertificate(const ompl::base::State* state,
                            const std::vector<ompl::base::SafetyCertificate*>& ocv) const;

  bool safetyCertificate(const ompl::base::State* state, const ompl::base::SafetyCertificate* sc,
                         std::vector<double>& dist) const;

  std::vector<double> distanceCertificate(const ompl::base::State* a, const ompl::base::State* b) const;

  CollisionCertificateChecker getCollisonCertificateChecker() const
  {
    return collisionCertificateChecker_;
  }

  SafetyCertificateChecker getSafetyCertificateChecker() const
  {
    return safetyCertificateChecker_;
  }

  DistanceCertificate getDistanceCertificate() const
  {
    return distanceCertificate_;
  }

  unsigned int getCertificateDim() const
  {
    return certificateDim_;
  }

protected:
  const ModelBasedPlanningContext* planning_context_;
  std::string group_name_;
  TSStateStorage tss_;

  collision_detection_fcl::FclDiscreteBVHManagerPtr manager_fcl_;
  collision_detection_bullet::BulletDiscreteBVHManagerPtr manager_bullet_;
  collision_detection::CollisionRequest req_;
  mutable collision_detection::CollisionResult res_;
  mutable collision_detection::ContactTestData cdata_;

private:
  SafetyCertificateChecker safetyCertificateChecker_ = std::bind(
      &SafetyCertificate::safetyCertificate, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

  CollisionCertificateChecker collisionCertificateChecker_ =
      std::bind(&SafetyCertificate::collisionCertificate, this, std::placeholders::_1, std::placeholders::_2);

  DistanceCertificate distanceCertificate_ =
      std::bind(&SafetyCertificate::distanceCertificate, this, std::placeholders::_1, std::placeholders::_2);

  double collision_safety_margin_;
  unsigned int certificateDim_;
};
}  // namespace ompl_interface
