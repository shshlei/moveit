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

#include <moveit/collision_detection_fcl_new/fcl_integration/fcl_collision_object_wrapper.h>

namespace collision_detection_fcl
{
FCLCollisionObjectWrapper::FCLCollisionObjectWrapper(const std::shared_ptr<fcl::CollisionGeometry<double>>& cgeom)
  : fcl::CollisionObject<double>(cgeom)
{
}

FCLCollisionObjectWrapper::FCLCollisionObjectWrapper(const std::shared_ptr<fcl::CollisionGeometry<double>>& cgeom,
                                                     const fcl::Transform3<double>& tf)
  : fcl::CollisionObject<double>(cgeom, tf)
{
}

FCLCollisionObjectWrapper::FCLCollisionObjectWrapper(const std::shared_ptr<fcl::CollisionGeometry<double>>& cgeom,
                                                     const fcl::Matrix3<double>& R, const fcl::Vector3<double>& T)
  : fcl::CollisionObject<double>(cgeom, R, T)
{
}

void FCLCollisionObjectWrapper::setContactDistanceThreshold(double contact_distance)
{
  if (contact_distance_ != contact_distance)
  {
    contact_distance_ = contact_distance;
    updateAABB();
  }
}

double FCLCollisionObjectWrapper::getContactDistanceThreshold() const
{
  return contact_distance_;
}

void FCLCollisionObjectWrapper::updateAABB()
{
  if (t.linear().isIdentity())
  {
    aabb = translate(cgeom->aabb_local, t.translation());
    fcl::Vector3<double> delta = fcl::Vector3<double>::Constant(contact_distance_);
    aabb.min_ -= delta;
    aabb.max_ += delta;
  }
  else
  {
    fcl::Vector3<double> center = t * cgeom->aabb_center;
    fcl::Vector3<double> delta = fcl::Vector3<double>::Constant(cgeom->aabb_radius + contact_distance_);
    aabb.min_ = center - delta;
    aabb.max_ = center + delta;
  }
}
}  // namespace collision_detection_fcl
