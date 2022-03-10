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

/* Author: Levi Armstrong, Jens Petit */

#include <moveit/collision_detection_bullet/bullet_integration/bullet_discrete_bvh_manager.h>

namespace collision_detection_bullet
{
BulletDiscreteBVHManagerPtr BulletDiscreteBVHManager::clone() const
{
  BulletDiscreteBVHManagerPtr manager(new BulletDiscreteBVHManager());

  manager->setActiveCollisionObjects(active_);
  manager->setContactDistanceThreshold(getContactDistanceThreshold());

  for (const std::pair<const std::string, CollisionObjectWrapperPtr>& cow : link2cow_)
  {
    CollisionObjectWrapperPtr new_cow = cow.second->clone();

    assert(new_cow->getCollisionShape());
    assert(new_cow->getCollisionShape()->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);
    manager->addCollisionObject(new_cow);
  }

  return manager;
}

void BulletDiscreteBVHManager::contactTest(collision_detection::ContactTestData& cdata, bool self)
{
  broadphase_->calculateOverlappingPairs(dispatcher_.get());
  btOverlappingPairCache* pair_cache = broadphase_->getOverlappingPairCache();
  ROS_DEBUG_STREAM_NAMED("collision_detection.bullet",
                         "Num overlapping candidates " << pair_cache->getNumOverlappingPairs());

  BroadphaseContactResultCallback cc(cdata, self);
  TesseractCollisionPairCallback collision_callback(dispatch_info_, dispatcher_.get(), cc);
  pair_cache->processAllOverlappingPairs(&collision_callback, dispatcher_.get());

  ROS_DEBUG_STREAM_NAMED("collision_detection.bullet", (cdata.res->collision ? "In" : "No")
                                                           << " collision with " << cdata.res->contact_count
                                                           << " collisions");
}
}  // namespace collision_detection_bullet
