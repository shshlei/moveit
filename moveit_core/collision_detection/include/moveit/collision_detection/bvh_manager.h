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

#include <moveit/macros/class_forward.h>
#include <moveit/collision_detection/collision_common.h>

namespace collision_detection
{
MOVEIT_CLASS_FORWARD(BVHManager);

/** @brief An abstract collision detection manager */
class BVHManager
{
public:
  BVHManager()
  {
    contact_distance_ = 0.0;
  }

  ~BVHManager() = default;

  BVHManagerPtr clone() const;

  /** \brief Cast this instance to a desired type. */
  template <class T>
  T* as()
  {
    /** \brief Make sure the type we are casting to is indeed a planner */
    BOOST_CONCEPT_ASSERT((boost::Convertible<T*, BVHManager*>));

    return static_cast<T*>(this);
  }

  /** \brief Cast this instance to a desired type. */
  template <class T>
  const T* as() const
  {
    /** \brief Make sure the type we are casting to is indeed a Planner */
    BOOST_CONCEPT_ASSERT((boost::Convertible<T*, BVHManager*>));

    return static_cast<const T*>(this);
  }

  virtual bool addCollisionObject(const std::string& name, const collision_detection::BodyType& type_id,
                                  const std::vector<shapes::ShapeConstPtr>& shapes,
                                  const EigenSTL::vector_Isometry3d& shape_poses,
                                  const std::vector<collision_detection::CollisionObjectType>& collision_object_types,
                                  bool active = true) = 0;

  virtual bool addCollisionObject(const std::string& name, const collision_detection::BodyType& type_id,
                                  const std::vector<shapes::ShapeConstPtr>& shapes,
                                  const EigenSTL::vector_Isometry3d& shape_poses,
                                  const std::vector<collision_detection::CollisionObjectType>& collision_object_types,
                                  const std::string& pname, const std::set<std::string>& touch_links) = 0;

  virtual bool hasCollisionObject(const std::string& name) const = 0;

  virtual bool removeCollisionObject(const std::string& name) = 0;

  virtual bool enableCollisionObject(const std::string& name) = 0;

  virtual bool disableCollisionObject(const std::string& name) = 0;

  virtual bool isCollisionObjectEnabled(const std::string& name) const = 0;

  virtual void setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose) = 0;

  virtual void setCollisionObjectsTransforms(const std::vector<std::string>& names,
                                             const EigenSTL::vector_Isometry3d& poses)
  {
    assert(names.size() == poses.size());
    for (std::size_t i = 0; i < names.size(); i++)
      setCollisionObjectsTransform(names[i], poses[i]);
  }

  virtual void setActiveCollisionObjects(const std::vector<std::string>& names)
  {
    active_ = names;
  }

  const std::vector<std::string>& getActiveCollisionObjects() const
  {
    return active_;
  }

  virtual void setContactDistanceThreshold(double contact_distance)
  {
    contact_distance_ = contact_distance;
  }

  double getContactDistanceThreshold() const
  {
    return contact_distance_;
  }

protected:
  /** @brief The contact distance threshold */
  double contact_distance_;

  std::vector<std::string> active_; /**< @brief A list of the active collision objects */
};
}  // namespace collision_detection
