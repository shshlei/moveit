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

/* Authors: Shi Shenglei */

#include <moveit/collision_detection_fcl_new/fcl_integration/fcl_discrete_bvh_manager.h>

namespace collision_detection_fcl
{
FclDiscreteBVHManagerPtr FclDiscreteBVHManager::clone() const
{
  auto manager = std::make_unique<FclDiscreteBVHManager>();

  manager->setActiveCollisionObjects(getActiveCollisionObjects());
  manager->setContactDistanceThreshold(getContactDistanceThreshold());

  std::vector<CollisionObjectWrapperPtr> cows;
  for (const auto& cow : link2cow_)
    cows.push_back(cow.second->clone());

  manager->addCollisionObjects(cows);

  return manager;
}

void FclDiscreteBVHManager::contactTest(collision_detection::ContactTestData& cdata, bool self)
{
  if (self)
  {
    if (!dynamic_manager_->empty())
      dynamic_manager_->collide(&cdata, &collisionCallback);
  }
  else if (!static_manager_->empty())
  {
    std::vector<fcl::CollisionObjectd*> cos;
    dynamic_manager_->getObjects(cos);
    for (auto& co : cos)
      static_manager_->collide(co, &cdata, &collisionCallback);

    //                static_manager_->collide(dynamic_manager_.get(), &cdata, &collisionCallback);
  }
}

void FclDiscreteBVHManager::distanceTest(collision_detection::DistanceTestData& cdata, bool self)
{
  if (self)
  {
    if (!dynamic_manager_->empty())
      dynamic_manager_->distance(&cdata, &distanceCallback);
  }
  else if (!static_manager_->empty())
  {
    static_manager_->distance(dynamic_manager_.get(), &cdata, &distanceCallback);
  }
}
}  // namespace collision_detection_fcl
