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

/* Authors: Levi Armstrong, Shi Shenglei */

#include <moveit/collision_detection_fcl/fcl_integration/fcl_bvh_manager.h>

namespace collision_detection_fcl
{
FclBVHManager::FclBVHManager() : BVHManager()
{
  static_manager_ = std::make_unique<fcl::DynamicAABBTreeCollisionManagerd>();
  dynamic_manager_ = std::make_unique<fcl::DynamicAABBTreeCollisionManagerd>();
}

bool FclBVHManager::addCollisionObject(
    const std::string& name, const collision_detection::BodyType& type_id,
    const std::vector<shapes::ShapeConstPtr>& shapes, const EigenSTL::vector_Isometry3d& shape_poses,
    const std::vector<collision_detection::CollisionObjectType>& collision_object_types, bool active)
{
  if (hasCollisionObject(name))
    removeCollisionObject(name);

  auto cow = createCollisionObject(name, type_id, shapes, shape_poses, collision_object_types, active);
  if (cow)
  {
    cow->setContactDistanceThreshold(contact_distance_);
    addCollisionObject(cow);
    return true;
  }

  return false;
}

bool FclBVHManager::addCollisionObject(
    const std::string& name, const collision_detection::BodyType& type_id,
    const std::vector<shapes::ShapeConstPtr>& shapes, const EigenSTL::vector_Isometry3d& shape_poses,
    const std::vector<collision_detection::CollisionObjectType>& collision_object_types, const std::string& pname,
    const std::set<std::string>& touch_links)
{
  if (hasCollisionObject(name))
    removeCollisionObject(name);

  auto cow = createCollisionObject(name, type_id, shapes, shape_poses, collision_object_types, pname, touch_links);
  if (cow)
  {
    cow->setContactDistanceThreshold(contact_distance_);
    addCollisionObject(cow);
    return true;
  }

  return false;
}

bool FclBVHManager::hasCollisionObject(const std::string& name) const
{
  return (link2cow_.find(name) != link2cow_.end());
}

bool FclBVHManager::removeCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    std::vector<CollisionObjectRawPtr>& objects = it->second->getCollisionObjectsRaw();

    if (it->second->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
    {
      for (auto& co : objects)
      {
        static_manager_->unregisterObject(co);
      }
    }
    else
    {
      for (auto& co : objects)
      {
        dynamic_manager_->unregisterObject(co);
      }
    }

    link2cow_.erase(name);

    return true;
  }

  return false;
}

bool FclBVHManager::enableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = true;
    return true;
  }

  return false;
}

bool FclBVHManager::disableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = false;
    return true;
  }

  return false;
}

bool FclBVHManager::isCollisionObjectEnabled(const std::string& name) const
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
    return it->second->m_enabled;

  return false;
}

void FclBVHManager::setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    const Eigen::Isometry3d& cur_tf = it->second->getCollisionObjectsTransform();
    // Note: If the transform has not changed do not updated to prevent unnecessary re-balancing of the BVH tree
    if (!cur_tf.translation().isApprox(pose.translation(), 1e-8) || !cur_tf.rotation().isApprox(pose.rotation(), 1e-8))
    {
      it->second->setCollisionObjectsTransform(pose);

      if (it->second->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
      {
        // Note: Calling update causes a re-balance of the AABB tree, which is expensive
        static_manager_->update(it->second->getCollisionObjectsRaw());
      }
      else
      {
        // Note: Calling update causes a re-balance of the AABB tree, which is expensive
        dynamic_manager_->update(it->second->getCollisionObjectsRaw());
      }
    }
  }
}

void FclBVHManager::setCollisionObjectsTransforms(const std::vector<std::string>& names,
                                                  const EigenSTL::vector_Isometry3d& poses)
{
  assert(names.size() == poses.size());

  std::vector<CollisionObjectRawPtr> static_update, dynamic_update;

  for (std::size_t i = 0; i < names.size(); ++i)
  {
    auto it = link2cow_.find(names[i]);
    if (it != link2cow_.end())
    {
      const Eigen::Isometry3d& cur_tf = it->second->getCollisionObjectsTransform();
      // Note: If the transform has not changed do not updated to prevent unnecessary re-balancing of the BVH tree
      if (!cur_tf.translation().isApprox(poses[i].translation(), 1e-8) ||
          !cur_tf.rotation().isApprox(poses[i].rotation(), 1e-8))
      {
        it->second->setCollisionObjectsTransform(poses[i]);

        std::vector<CollisionObjectRawPtr>& co = it->second->getCollisionObjectsRaw();

        if (it->second->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
        {
          static_update.insert(static_update.end(), co.begin(), co.end());
        }
        else
        {
          dynamic_update.insert(dynamic_update.end(), co.begin(), co.end());
        }
      }
    }
  }

  // This is because FCL supports batch update which only re-balances the tree once
  if (!static_update.empty())
    static_manager_->update(static_update);

  if (!dynamic_update.empty())
    dynamic_manager_->update(dynamic_update);
}

void FclBVHManager::setActiveCollisionObjects(const std::vector<std::string>& names)
{
  active_ = names;

  for (auto& co : link2cow_)
    updateCollisionObjectFilters(active_, co.second, static_manager_, dynamic_manager_);
}

void FclBVHManager::addCollisionObject(const CollisionObjectWrapperPtr& cow)
{
  link2cow_[cow->getName()] = cow;

  std::vector<CollisionObjectRawPtr>& objects = cow->getCollisionObjectsRaw();
  if (cow->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
  {
    static_manager_->registerObjects(objects);
  }
  else
  {
    dynamic_manager_->registerObjects(objects);
  }
}

void FclBVHManager::addCollisionObjects(const std::vector<CollisionObjectWrapperPtr>& cows)
{
  std::vector<CollisionObjectRawPtr> static_objects, dynamic_objects;

  for (auto& cow : cows)
  {
    link2cow_[cow->getName()] = cow;

    std::vector<CollisionObjectRawPtr>& objects = cow->getCollisionObjectsRaw();
    if (cow->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
    {
      static_objects.insert(static_objects.end(), objects.begin(), objects.end());
    }
    else
    {
      dynamic_objects.insert(dynamic_objects.end(), objects.begin(), objects.end());
    }
  }

  if (!static_objects.empty())
  {
    static_manager_->registerObjects(static_objects);
  }

  if (!dynamic_objects.empty())
  {
    dynamic_manager_->registerObjects(dynamic_objects);
  }
}

const std::map<std::string, CollisionObjectWrapperPtr>& FclBVHManager::getCollisionObjects() const
{
  return link2cow_;
}

const CollisionObjectWrapperPtr FclBVHManager::getCollisionObject(const std::string& name) const
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
    return it->second;

  return std::make_shared<CollisionObjectWrapper>();
}
}  // namespace collision_detection_fcl
