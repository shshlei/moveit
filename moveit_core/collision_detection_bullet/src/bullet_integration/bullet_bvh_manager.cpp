/*********************************************************************
 * Software License Agreement (BSD-2-Clause)
 *
 * Copyright (c) 2017, Southwest Research Institute
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

/* Authors: Levi Armstrong, Jens Petit */

#include <moveit/collision_detection_bullet/bullet_integration/bullet_bvh_manager.h>

namespace collision_detection_bullet
{
BulletBVHManager::BulletBVHManager() : BVHManager()
{
  dispatcher_ = std::make_unique<btCollisionDispatcher>(&coll_config_);

  dispatcher_->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE, BOX_SHAPE_PROXYTYPE,
                                           coll_config_.getCollisionAlgorithmCreateFunc(CONVEX_SHAPE_PROXYTYPE,
                                                                                        CONVEX_SHAPE_PROXYTYPE));

  dispatcher_->setDispatcherFlags(dispatcher_->getDispatcherFlags() &
                                  ~btCollisionDispatcher::CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD);

  broadphase_ = std::make_unique<btDbvtBroadphase>();

  broadphase_->getOverlappingPairCache()->setOverlapFilterCallback(&filter_callback_);

  contact_distance_ = BULLET_DEFAULT_CONTACT_DISTANCE;
}

BulletBVHManager::~BulletBVHManager()
{
  // clean up remaining objects
  for (const std::pair<const std::string, CollisionObjectWrapperPtr>& cow : link2cow_)
    removeCollisionObjectFromBroadphase(cow.second, broadphase_, dispatcher_);
}

bool BulletBVHManager::addCollisionObject(
    const std::string& name, const collision_detection::BodyType& type_id,
    const std::vector<shapes::ShapeConstPtr>& shapes, const EigenSTL::vector_Isometry3d& shape_poses,
    const std::vector<collision_detection::CollisionObjectType>& collision_object_types, bool active)
{
  if (hasCollisionObject(name))
    removeCollisionObject(name);

  auto cow = createCollisionObject(name, type_id, shapes, shape_poses, collision_object_types, active);
  if (cow)
  {
    cow->setContactProcessingThreshold(contact_distance_);
    addCollisionObject(cow);
    return true;
  }

  return false;
}

bool BulletBVHManager::addCollisionObject(
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
    cow->setContactProcessingThreshold(contact_distance_);
    addCollisionObject(cow);
    return true;
  }

  return false;
}

bool BulletBVHManager::hasCollisionObject(const std::string& name) const
{
  return (link2cow_.find(name) != link2cow_.end());
}

bool BulletBVHManager::removeCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    CollisionObjectWrapperPtr& cow = it->second;
    removeCollisionObjectFromBroadphase(cow, broadphase_, dispatcher_);
    link2cow_.erase(name);
    return true;
  }

  return false;
}

bool BulletBVHManager::enableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = true;

    // Need to clean the proxy from broadphase cache so BroadPhaseFilter gets called again.
    // The BroadPhaseFilter only gets called once, so if you change when two objects can be in collision, like filters
    // this must be called or contacts between shapes will be missed.
    if (it->second->getBroadphaseHandle())
      broadphase_->getOverlappingPairCache()->cleanProxyFromPairs(it->second->getBroadphaseHandle(), dispatcher_.get());

    return true;
  }

  return false;
}

bool BulletBVHManager::disableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = false;

    // Need to clean the proxy from broadphase cache so BroadPhaseFilter gets called again.
    // The BroadPhaseFilter only gets called once, so if you change when two objects can be in collision, like filters
    // this must be called or contacts between shapes will be missed.
    if (it->second->getBroadphaseHandle())
      broadphase_->getOverlappingPairCache()->cleanProxyFromPairs(it->second->getBroadphaseHandle(), dispatcher_.get());

    return true;
  }

  return false;
}

bool BulletBVHManager::isCollisionObjectEnabled(const std::string& name) const
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
    return it->second->m_enabled;

  return false;
}

void BulletBVHManager::setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose)
{
  // TODO(j-petit): Find a way to remove this check. Need to store information in CollisionEnv transforms with geometry
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    const Eigen::Isometry3d& cur_tf = it->second->getCollisionObjectsTransform();
    if (!cur_tf.translation().isApprox(pose.translation(), 1e-8) || !cur_tf.rotation().isApprox(pose.rotation(), 1e-8))
    {
      CollisionObjectWrapperPtr& cow = it->second;
      cow->setCollisionObjectsTransform(pose);

      // Now update Broadphase AABB (See BulletWorld updateSingleAabb function)
      if (cow->getBroadphaseHandle())
        updateBroadphaseAABB(cow, broadphase_, dispatcher_);
    }
  }
}

void BulletBVHManager::setActiveCollisionObjects(const std::vector<std::string>& names)
{
  active_ = names;

  // Now need to update the broadphase with correct aabb
  for (std::pair<const std::string, CollisionObjectWrapperPtr>& co : link2cow_)
  {
    CollisionObjectWrapperPtr& cow = co.second;
    updateCollisionObjectFilters(active_, *cow);

    // The broadphase tree structure has to be updated, therefore remove and add is necessary
    removeCollisionObjectFromBroadphase(cow, broadphase_, dispatcher_);
    addCollisionObjectToBroadphase(cow, broadphase_, dispatcher_);
  }
}

void BulletBVHManager::setContactDistanceThreshold(double contact_distance)
{
  if (contact_distance_ != contact_distance)
  {
    contact_distance_ = contact_distance;

    for (std::pair<const std::string, CollisionObjectWrapperPtr>& co : link2cow_)
    {
      CollisionObjectWrapperPtr& cow = co.second;
      cow->setContactProcessingThreshold(static_cast<btScalar>(contact_distance));
      if (cow->getBroadphaseHandle())
        updateBroadphaseAABB(cow, broadphase_, dispatcher_);
    }
  }
}

void BulletBVHManager::addCollisionObject(const CollisionObjectWrapperPtr& cow)
{
  link2cow_[cow->getName()] = cow;
  addCollisionObjectToBroadphase(cow, broadphase_, dispatcher_);
}

const std::map<std::string, CollisionObjectWrapperPtr>& BulletBVHManager::getCollisionObjects() const
{
  return link2cow_;
}

const CollisionObjectWrapperPtr BulletBVHManager::getCollisionObject(const std::string& name) const
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
    return it->second;

  return std::make_shared<CollisionObjectWrapper>();
}
}  // namespace collision_detection_bullet
