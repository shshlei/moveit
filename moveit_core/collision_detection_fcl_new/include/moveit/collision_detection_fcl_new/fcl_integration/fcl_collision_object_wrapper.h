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

#include <fcl/narrowphase/collision_object-inl.h>

namespace collision_detection_fcl
{
/**
 * @brief This is a wrapper around FCL Collision Object Class which allows you to expand the AABB by the contact dist.
 *
 * This significantly improves performance when making distance requests if performing a contact tests type FIRST.
 */
class FCLCollisionObjectWrapper : public fcl::CollisionObject<double>
{
public:
  FCLCollisionObjectWrapper(const std::shared_ptr<fcl::CollisionGeometry<double>>& cgeom);

  FCLCollisionObjectWrapper(const std::shared_ptr<fcl::CollisionGeometry<double>>& cgeom,
                            const fcl::Transform3<double>& tf);

  FCLCollisionObjectWrapper(const std::shared_ptr<fcl::CollisionGeometry<double>>& cgeom, const fcl::Matrix3<double>& R,
                            const fcl::Vector3<double>& T);

  /**
   * @brief Set the collision objects contact distance threshold.
   *
   * This automatically calls updateAABB() which increases the AABB by the contact distance.
   * @param contact_distance The contact distance threshold.
   */
  void setContactDistanceThreshold(double contact_distance);

  /**
   * @brief Get the collision objects contact distance threshold.
   * @return The contact distance threshold.
   */
  double getContactDistanceThreshold() const;

  /**
   * @brief Update the internal AABB. This must be called instead of the base class computeAABB().
   *
   * After setting the collision objects transform this must be called.
   */
  void updateAABB();

protected:
  double contact_distance_{ 0.0 }; /**< @brief The contact distance threshold. */
};
}  // namespace collision_detection_fcl
