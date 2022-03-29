/*********************************************************************
 * Software License Agreement (BSD-2-Clause)
 *
 * Copyright (c) 2017, Southwest Research Institute
 * Copyright (c) 2013, John Schulman
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Shi Shenglei  */

#pragma once

#include <moveit/collision_detection/bvh_manager.h>
#include <moveit/collision_detection_fcl_new/fcl_integration/fcl_utils.h>

namespace collision_detection_fcl
{
MOVEIT_CLASS_FORWARD(FclBVHManager);

/** @brief A FCL implementation of the discrete contact manager */
class FclBVHManager : public collision_detection::BVHManager
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FclBVHManager();

  ~FclBVHManager() = default;

  FclBVHManagerPtr clone() const;

  bool addCollisionObject(const std::string& name, const collision_detection::BodyType& type_id,
                          const std::vector<shapes::ShapeConstPtr>& shapes,
                          const EigenSTL::vector_Isometry3d& shape_poses,
                          const std::vector<collision_detection::CollisionObjectType>& collision_object_types,
                          bool active = true) override;

  bool addCollisionObject(const std::string& name, const collision_detection::BodyType& type_id,
                          const std::vector<shapes::ShapeConstPtr>& shapes,
                          const EigenSTL::vector_Isometry3d& shape_poses,
                          const std::vector<collision_detection::CollisionObjectType>& collision_object_types,
                          const std::string& pname, const std::set<std::string>& touch_links) override;

  bool hasCollisionObject(const std::string& name) const override;

  bool removeCollisionObject(const std::string& name) override;

  bool enableCollisionObject(const std::string& name) override;

  bool disableCollisionObject(const std::string& name) override;

  bool isCollisionObjectEnabled(const std::string& name) const override;

  void setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose) override;

  void setCollisionObjectsTransforms(const std::vector<std::string>& names,
                                     const EigenSTL::vector_Isometry3d& poses) override;

  void setActiveCollisionObjects(const std::vector<std::string>& names) override;

  void setContactDistanceThreshold(double contact_distance) override
  {
    if (contact_distance_ != contact_distance)
    {
      contact_distance_ = contact_distance;
      for (auto it = link2cow_.begin(); it != link2cow_.end(); it++)
        it->second->setContactDistanceThreshold(contact_distance_);
      if (!static_manager_->empty())
        static_manager_->update();
      if (!dynamic_manager_->empty())
        dynamic_manager_->update();
    }
  }

  virtual void contactTest(collision_detection::ContactTestData& cdata, bool self) = 0;

  virtual void distanceTest(collision_detection::DistanceTestData& cdata, bool self) = 0;

  void addCollisionObject(const CollisionObjectWrapperPtr& cow);

  void addCollisionObjects(const std::vector<CollisionObjectWrapperPtr>& cows);

  const std::map<std::string, CollisionObjectWrapperPtr>& getCollisionObjects() const;

  const CollisionObjectWrapperPtr getCollisionObject(const std::string& name) const;

protected:
  /** @brief Broad-phase Collision Manager for active collision objects */
  std::unique_ptr<fcl::BroadPhaseCollisionManagerd> static_manager_;

  /** @brief Broad-phase Collision Manager for active collision objects */
  std::unique_ptr<fcl::BroadPhaseCollisionManagerd> dynamic_manager_;

  std::map<std::string, CollisionObjectWrapperPtr> link2cow_;
};
}  // namespace collision_detection_fcl
