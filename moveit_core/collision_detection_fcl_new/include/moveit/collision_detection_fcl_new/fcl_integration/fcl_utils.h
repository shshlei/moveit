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
#include <moveit/collision_detection/collision_utils.h>
#include <moveit/collision_detection_fcl_new/fcl_integration/fcl_collision_object_wrapper.h>

#include <fcl/broadphase/broadphase_dynamic_AABB_tree-inl.h>
#include <fcl/narrowphase/collision-inl.h>
#include <fcl/narrowphase/distance-inl.h>

#include <memory>
#include <set>

namespace collision_detection_fcl
{
using CollisionGeometryPtr = std::shared_ptr<fcl::CollisionGeometryd>;
using CollisionObjectPtr = std::shared_ptr<FCLCollisionObjectWrapper>;
using CollisionObjectRawPtr = fcl::CollisionObjectd*;

enum CollisionFilterGroups
{
  DefaultFilter = 1,
  StaticFilter = 2,
  KinematicFilter = 4,
  AllFilter = -1  // all bits sets: DefaultFilter | StaticFilter | KinematicFilter
};

MOVEIT_CLASS_FORWARD(CollisionObjectWrapper);

/**
 * @brief This is a Tesseract link collision object wrapper which add items specific to tesseract. It is a wrapper
 * around a tesseract link which may contain several collision objects.
 */
class CollisionObjectWrapper
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CollisionObjectWrapper() = default;

  CollisionObjectWrapper(const std::string& name, const collision_detection::BodyType& type_id,
                         const std::vector<shapes::ShapeConstPtr>& shapes,
                         const EigenSTL::vector_Isometry3d& shape_poses,
                         const std::vector<collision_detection::CollisionObjectType>& collision_object_types,
                         bool active = true);

  /** \brief Constructor for attached robot objects
   *
   *  \param shape_poses These poses are in the global (planning) frame */
  CollisionObjectWrapper(const std::string& name, const collision_detection::BodyType& type_id,
                         const std::vector<shapes::ShapeConstPtr>& shapes,
                         const EigenSTL::vector_Isometry3d& shape_poses,
                         const std::vector<collision_detection::CollisionObjectType>& collision_object_types,
                         const std::string& pname, const std::set<std::string>& touch_links);

  short int m_collisionFilterGroup;

  short int m_collisionFilterMask;

  bool m_enabled{ true };

  std::set<std::string> m_touch_links;

  const std::string& getName() const
  {
    return m_name;
  }

  const std::string& getPName() const
  {
    return m_pname;
  }

  /** @brief Get a user defined type */
  const collision_detection::BodyType& getTypeID() const
  {
    return m_type_id;
  }

  /** \brief Check if two objects point to the same source object */
  bool sameObject(const CollisionObjectWrapper& other) const
  {
    return m_name == other.m_name && m_pname == other.m_pname && m_type_id == other.m_type_id &&
           m_shapes.size() == other.m_shapes.size() && m_shape_poses.size() == other.m_shape_poses.size() &&
           m_collision_object_types == other.m_collision_object_types && m_touch_links == other.m_touch_links &&
           std::equal(m_shapes.begin(), m_shapes.end(), other.m_shapes.begin()) &&
           std::equal(m_shape_poses.begin(), m_shape_poses.end(), other.m_shape_poses.begin(),
                      [](const Eigen::Isometry3d& t1, const Eigen::Isometry3d& t2) { return t1.isApprox(t2); });
  }

  void setCollisionObjectsTransform(const Eigen::Isometry3d& pose)
  {
    m_world_pose = pose;
    for (std::size_t i = 0; i < m_collision_objects.size(); ++i)
    {
      CollisionObjectPtr& co = m_collision_objects[i];
      co->setTransform(pose * m_shape_poses[i]);
      co->updateAABB();  // This a tesseract function that updates abb to take into account contact distance
    }
  }

  const Eigen::Isometry3d& getCollisionObjectsTransform() const
  {
    return m_world_pose;
  }

  void setContactDistanceThreshold(double contact_distance)
  {
    if (m_contact_distance != contact_distance)
    {
      m_contact_distance = contact_distance;
      for (auto& co : m_collision_objects)
        co->setContactDistanceThreshold(m_contact_distance);
    }
  }

  double getContactDistanceThreshold() const
  {
    return m_contact_distance;
  }

  const std::vector<CollisionObjectPtr>& getCollisionObjects() const
  {
    return m_collision_objects;
  }

  std::vector<CollisionObjectPtr>& getCollisionObjects()
  {
    return m_collision_objects;
  }

  const std::vector<CollisionObjectRawPtr>& getCollisionObjectsRaw() const
  {
    return m_collision_objects_raw;
  }

  std::vector<CollisionObjectRawPtr>& getCollisionObjectsRaw()
  {
    return m_collision_objects_raw;
  }

  std::shared_ptr<CollisionObjectWrapper> clone() const
  {
    auto clone_cow = std::make_shared<CollisionObjectWrapper>();
    clone_cow->m_name = m_name;
    clone_cow->m_pname = m_pname;
    clone_cow->m_type_id = m_type_id;
    clone_cow->m_shapes = m_shapes;
    clone_cow->m_shape_poses = m_shape_poses;
    clone_cow->m_collision_object_types = m_collision_object_types;
    clone_cow->m_shape_id_index = m_shape_id_index;
    clone_cow->m_collision_geometries = m_collision_geometries;

    clone_cow->m_collision_objects.reserve(m_collision_objects.size());
    clone_cow->m_collision_objects_raw.reserve(m_collision_objects.size());
    for (const auto& co : m_collision_objects)
    {
      assert(std::dynamic_pointer_cast<FCLCollisionObjectWrapper>(co) != nullptr);

      auto collObj =
          std::make_shared<FCLCollisionObjectWrapper>(*std::static_pointer_cast<FCLCollisionObjectWrapper>(co));
      collObj->setUserData(clone_cow.get());
      clone_cow->m_collision_objects.push_back(collObj);
      clone_cow->m_collision_objects_raw.push_back(collObj.get());
    }

    clone_cow->setCollisionObjectsTransform(m_world_pose);
    clone_cow->setContactDistanceThreshold(m_contact_distance);
    clone_cow->m_collisionFilterGroup = m_collisionFilterGroup;
    clone_cow->m_collisionFilterMask = m_collisionFilterMask;
    clone_cow->m_enabled = m_enabled;
    clone_cow->m_touch_links = m_touch_links;
    return clone_cow;
  }

  /**
   * @brief Given fcl collision shape get the index to the links collision shape
   * @param co fcl collision shape
   * @return links collision shape index
   */
  int getShapeIndex(const fcl::CollisionObjectd* co) const;

  int getShapeIndex(const fcl::CollisionGeometryd* co) const;

  std::size_t getShapeCount() const
  {
    return m_shape_id_index.size();
  }

  CollisionObjectWrapperPtr createSubObject(int shape_index, bool active) const;

protected:
  /** \brief The name of the object, must be unique. */
  std::string m_name;

  /** \brief The parent link name of the attached object, must be unique. */
  std::string m_pname;

  collision_detection::BodyType m_type_id;

  /** @brief The shapes that define the collison object */
  std::vector<shapes::ShapeConstPtr> m_shapes;

  /** @brief The poses of the shapes, must be same length as m_shapes */
  EigenSTL::vector_Isometry3d m_shape_poses;

  /** @brief The shape collision object type to be used. */
  std::vector<collision_detection::CollisionObjectType> m_collision_object_types;

  std::vector<CollisionGeometryPtr> m_collision_geometries;

  std::vector<CollisionObjectPtr> m_collision_objects;
  /**
   * @brief The raw pointer is also stored because FCL accepts vectors for batch process.
   * Note: They are updating the API to Shared Pointers but the broadphase has not been updated yet.
   */
  std::vector<CollisionObjectRawPtr> m_collision_objects_raw;

  Eigen::Isometry3d m_world_pose{ Eigen::Isometry3d::Identity() }; /**< @brief Collision Object World Transformation */

  double m_contact_distance{ 0.0 }; /**< @brief The contact distance threshold */

  std::vector<int> m_shape_id_index;
};

CollisionGeometryPtr createShapePrimitive(const shapes::ShapeConstPtr& geom,
                                          const collision_detection::CollisionObjectType& collision_object_type);

inline CollisionObjectWrapperPtr
createCollisionObject(const std::string& name, const collision_detection::BodyType& type_id,
                      const std::vector<shapes::ShapeConstPtr>& shapes, const EigenSTL::vector_Isometry3d& shape_poses,
                      const std::vector<collision_detection::CollisionObjectType>& collision_object_types,
                      bool active = true)
{
  auto cow =
      std::make_shared<CollisionObjectWrapper>(name, type_id, shapes, shape_poses, collision_object_types, active);
  return cow;
}

inline CollisionObjectWrapperPtr
createCollisionObject(const std::string& name, const collision_detection::BodyType& type_id,
                      const std::vector<shapes::ShapeConstPtr>& shapes, const EigenSTL::vector_Isometry3d& shape_poses,
                      const std::vector<collision_detection::CollisionObjectType>& collision_object_types,
                      const std::string& pname, const std::set<std::string>& touch_links)
{
  auto cow = std::make_shared<CollisionObjectWrapper>(name, type_id, shapes, shape_poses, collision_object_types, pname,
                                                      touch_links);
  return cow;
}

/**
 * @brief Update collision objects filters
 * @param active The active collision objects
 * @param cow The collision object to update
 * @param static_manager Broadphasse manager for static objects
 * @param dynamic_manager Broadphase manager for dynamic objects
 */
inline void updateCollisionObjectFilters(const std::vector<std::string>& active, const CollisionObjectWrapperPtr cow,
                                         const std::unique_ptr<fcl::BroadPhaseCollisionManagerd>& static_manager,
                                         const std::unique_ptr<fcl::BroadPhaseCollisionManagerd>& dynamic_manager)
{
  // For descrete checks we can check static to kinematic and kinematic to
  // kinematic
  if (!collision_detection::isLinkActive(active, cow->getName()))
  {
    if (cow->m_collisionFilterGroup != CollisionFilterGroups::StaticFilter)
    {
      std::vector<CollisionObjectPtr>& objects = cow->getCollisionObjects();
      // This link was dynamic but is now static
      for (auto& co : objects)
        dynamic_manager->unregisterObject(co.get());

      for (auto& co : objects)
        static_manager->registerObject(co.get());
    }

    cow->m_collisionFilterGroup = CollisionFilterGroups::StaticFilter;
  }
  else
  {
    if (cow->m_collisionFilterGroup != CollisionFilterGroups::KinematicFilter)
    {
      std::vector<CollisionObjectPtr>& objects = cow->getCollisionObjects();
      // This link was static but is now dynamic
      for (auto& co : objects)
        static_manager->unregisterObject(co.get());

      for (auto& co : objects)
        dynamic_manager->registerObject(co.get());
    }

    cow->m_collisionFilterGroup = CollisionFilterGroups::KinematicFilter;
  }

  // If the group is StaticFilter then the Mask is KinematicFilter, then StaticFilter groups can only collide with
  // KinematicFilter groups. If the group is KinematicFilter then the mask is StaticFilter and KinematicFilter meaning
  // that KinematicFilter groups can collide with both StaticFilter and KinematicFilter groups.
  if (cow->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
  {
    cow->m_collisionFilterMask = CollisionFilterGroups::KinematicFilter;
  }
  else
  {
    cow->m_collisionFilterMask = CollisionFilterGroups::StaticFilter | CollisionFilterGroups::KinematicFilter;
  }
}

bool collisionCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data);

bool distanceCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data, double& min_dist);
}  // namespace collision_detection_fcl
