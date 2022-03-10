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

#include <fcl/geometry/bvh/BVH_model-inl.h>
#include <fcl/geometry/shape/box-inl.h>
#include <fcl/geometry/shape/cylinder-inl.h>
#include <fcl/geometry/shape/convex-inl.h>
#include <fcl/geometry/shape/plane-inl.h>
#include <fcl/geometry/shape/sphere-inl.h>
#include <fcl/geometry/shape/cone-inl.h>
#include <fcl/geometry/shape/capsule-inl.h>
#include <fcl/geometry/octree/octree-inl.h>

#include <moveit/collision_detection_fcl/fcl_integration/fcl_utils.h>

#include <moveit/collision_detection/collision_utils.h>

namespace collision_detection_fcl
{
CollisionGeometryPtr createShapePrimitive(const shapes::Box* geom,
                                          const collision_detection::CollisionObjectType& collision_object_type)
{
  (void)(collision_object_type);
  assert(collision_object_type == collision_detection::CollisionObjectType::USE_SHAPE_TYPE);
  const double* size = geom->size;
  return std::make_shared<fcl::Boxd>(size[0], size[1], size[2]);
}

CollisionGeometryPtr createShapePrimitive(const shapes::Sphere* geom,
                                          const collision_detection::CollisionObjectType& collision_object_type)
{
  (void)(collision_object_type);
  assert(collision_object_type == collision_detection::CollisionObjectType::USE_SHAPE_TYPE);
  return std::make_shared<fcl::Sphered>(geom->radius);
}

CollisionGeometryPtr createShapePrimitive(const shapes::Cylinder* geom,
                                          const collision_detection::CollisionObjectType& collision_object_type)
{
  (void)(collision_object_type);
  assert(collision_object_type == collision_detection::CollisionObjectType::USE_SHAPE_TYPE);
  return std::make_shared<fcl::Cylinderd>(geom->radius, geom->length);
}

CollisionGeometryPtr createShapePrimitive(const shapes::Cone* geom,
                                          const collision_detection::CollisionObjectType& collision_object_type)
{
  (void)(collision_object_type);
  assert(collision_object_type == collision_detection::CollisionObjectType::USE_SHAPE_TYPE);
  return std::make_shared<fcl::Coned>(geom->radius, geom->length);
}

CollisionGeometryPtr createShapePrimitive(const shapes::Plane* geom,
                                          const collision_detection::CollisionObjectType& collision_object_type)
{
  (void)(collision_object_type);
  assert(collision_object_type == collision_detection::CollisionObjectType::USE_SHAPE_TYPE);
  return std::make_shared<fcl::Planed>(geom->a, geom->b, geom->c, geom->d);
}

CollisionGeometryPtr createShapePrimitive(const shapes::Mesh* geom,
                                          const collision_detection::CollisionObjectType& collision_object_type)
{
  assert(collision_object_type == collision_detection::CollisionObjectType::USE_SHAPE_TYPE);

  if (geom->vertex_count > 0 && geom->triangle_count > 0)
  {
    // convert the mesh to the assigned collision object type
    switch (collision_object_type)
    {
        //                case collision_detection::CollisionObjectType::CONVEX_HULL:
        //                {
        //                    // Create a convex hull shape to approximate Trimesh
        //                    std::vector<Eigen::Vector3d> input;
        //                    input.reserve(geom->vertex_count);
        //                    for (std::size_t i = 0; i < geom->vertex_count; ++i)
        //                        input.push_back(Eigen::Vector3d(geom->vertices[3 * i], geom->vertices[3 * i + 1], geom->vertices[3
        //                        * i + 2]));
        //
        //                    std::vector<Eigen::Vector3d> vertices;
        //                    std::vector<int> faces;
        //
        //                    int face_count = collision_detection_fcl::createConvexHull(vertices, faces, input);
        //                    if (face_count < 0)
        //                        return nullptr;
        //
        //                    return std::make_shared<fcl::Convexd>(std::make_shared<std::vector<Eigen::Vector3d>>(vertices),
        //                    face_count, std::make_shared<std::vector<int>>(faces));
        //                }
      case collision_detection::CollisionObjectType::USE_SHAPE_TYPE:
      {
        std::vector<fcl::Vector3d> vertices;
        vertices.reserve(geom->vertex_count);
        for (std::size_t i = 0; i < geom->vertex_count; ++i)
          vertices.push_back(
              Eigen::Vector3d(geom->vertices[3 * i], geom->vertices[3 * i + 1], geom->vertices[3 * i + 2]));

        std::vector<fcl::Triangle> tri_indices(static_cast<std::size_t>(geom->triangle_count));

        for (std::size_t i = 0; i < geom->triangle_count; ++i)
        {
          tri_indices[i] = fcl::Triangle(static_cast<std::size_t>(geom->triangles[3 * i]),
                                         static_cast<std::size_t>(geom->triangles[3 * i + 1]),
                                         static_cast<std::size_t>(geom->triangles[3 * i + 2]));
        }

        auto g = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();

        g->beginModel();
        g->addSubModel(vertices, tri_indices);
        g->endModel();

        return g;
      }
      default:
      {
        ROS_ERROR("This fcl shape type (%d) is not supported for geometry meshs",
                  static_cast<int>(collision_object_type));
        return nullptr;
      }
    }
  }

  ROS_ERROR_NAMED("collision_detection.fcl", "The mesh is empty!");
  return nullptr;
}

CollisionGeometryPtr createShapePrimitive(const shapes::OcTree* geom,
                                          const collision_detection::CollisionObjectType& collision_object_type)
{
  (void)(collision_object_type);
  assert(collision_object_type == collision_detection::CollisionObjectType::USE_SHAPE_TYPE);
  return std::make_shared<fcl::OcTreed>(geom->octree);
}

CollisionGeometryPtr createShapePrimitive(const shapes::ShapeConstPtr& geom,
                                          const collision_detection::CollisionObjectType& collision_object_type)
{
  CollisionGeometryPtr shape;

  switch (geom->type)
  {
    case shapes::BOX:
    {
      shape = createShapePrimitive(static_cast<const shapes::Box*>(geom.get()), collision_object_type);
      break;
    }
    case shapes::SPHERE:
    {
      shape = createShapePrimitive(static_cast<const shapes::Sphere*>(geom.get()), collision_object_type);
      break;
    }
    case shapes::CYLINDER:
    {
      shape = createShapePrimitive(static_cast<const shapes::Cylinder*>(geom.get()), collision_object_type);
      break;
    }
    case shapes::CONE:
    {
      shape = createShapePrimitive(static_cast<const shapes::Cone*>(geom.get()), collision_object_type);
      break;
    }
    case shapes::PLANE:
    {
      shape = createShapePrimitive(static_cast<const shapes::Plane*>(geom.get()), collision_object_type);
      break;
    }
    case shapes::MESH:
    {
      shape = createShapePrimitive(static_cast<const shapes::Mesh*>(geom.get()), collision_object_type);
      break;
    }
    case shapes::OCTREE:
    {
      shape = createShapePrimitive(static_cast<const shapes::OcTree*>(geom.get()), collision_object_type);
      break;
    }
    default:
    {
      ROS_ERROR("This geometric shape type (%d) is not supported using FCL yet", static_cast<int>(geom->type));
      break;
    }
  }

  return shape;
}

CollisionObjectWrapper::CollisionObjectWrapper(
    const std::string& name, const collision_detection::BodyType& type_id,
    const std::vector<shapes::ShapeConstPtr>& shapes, const EigenSTL::vector_Isometry3d& shape_poses,
    const std::vector<collision_detection::CollisionObjectType>& collision_object_types, bool active)
  : m_name(name)
  , m_type_id(type_id)
  , m_shapes(shapes)
  , m_shape_poses(shape_poses)
  , m_collision_object_types(collision_object_types)
{
  assert(!m_name.empty());                                                                                // NOLINT
  assert(!m_shapes.empty());                                                                              // NOLINT
  assert(!m_shape_poses.empty());                                                                         // NOLINT
  assert(!m_collision_object_types.empty());                                                              // NOLINT
  assert(m_shapes.size() == m_shape_poses.size() && m_shapes.size() == m_collision_object_types.size());  // NOLINT

  if (active)
  {
    m_collisionFilterGroup = KinematicFilter;
    m_collisionFilterMask = KinematicFilter | StaticFilter;
  }
  else
  {
    m_collisionFilterGroup = StaticFilter;
    m_collisionFilterMask = KinematicFilter;
  }

  m_shape_id_index.reserve(m_shapes.size());
  m_collision_geometries.reserve(m_shapes.size());
  m_collision_objects.reserve(m_shapes.size());
  m_collision_objects_raw.reserve(m_shapes.size());
  for (std::size_t i = 0; i < m_shapes.size(); ++i)
  {
    CollisionGeometryPtr subshape = createShapePrimitive(m_shapes[i], m_collision_object_types[i]);
    if (subshape)
    {
      m_shape_id_index.push_back(static_cast<int>(i));
      m_collision_geometries.push_back(subshape);
      auto co = std::make_shared<FCLCollisionObjectWrapper>(subshape);
      co->setUserData(this);
      co->setTransform(m_shape_poses[i]);
      co->updateAABB();
      m_collision_objects.push_back(co);
      m_collision_objects_raw.push_back(co.get());
    }
  }
}

CollisionObjectWrapper::CollisionObjectWrapper(
    const std::string& name, const collision_detection::BodyType& type_id,
    const std::vector<shapes::ShapeConstPtr>& shapes, const EigenSTL::vector_Isometry3d& shape_poses,
    const std::vector<collision_detection::CollisionObjectType>& collision_object_types, const std::string& pname,
    const std::set<std::string>& touch_links)
  : CollisionObjectWrapper(name, type_id, shapes, shape_poses, collision_object_types, true)
{
  m_pname = pname;
  m_touch_links = touch_links;
}

int CollisionObjectWrapper::getShapeIndex(const fcl::CollisionObjectd* co) const
{
  auto it = std::find_if(m_collision_objects_raw.begin(), m_collision_objects_raw.end(),
                         [&co](const CollisionObjectRawPtr& c) { return c == co; });

  if (it != m_collision_objects_raw.end())
    return static_cast<int>(std::distance(m_collision_objects_raw.begin(), it));

  return -1;
}

int CollisionObjectWrapper::getShapeIndex(const fcl::CollisionGeometryd* co) const
{
  auto it = std::find_if(m_collision_geometries.begin(), m_collision_geometries.end(),
                         [&co](const CollisionGeometryPtr& c) { return c.get() == co; });

  if (it != m_collision_geometries.end())
    return static_cast<int>(std::distance(m_collision_geometries.begin(), it));

  return -1;
}

CollisionObjectWrapperPtr CollisionObjectWrapper::createSubObject(int shape_index, bool active) const
{
  int shape_id = m_shape_id_index[shape_index];
  shapes::ShapeConstPtr shape(m_shapes[shape_id].get());
  auto cow = std::make_shared<CollisionObjectWrapper>(
      m_name, m_type_id, std::vector<shapes::ShapeConstPtr>(1, shape),
      EigenSTL::vector_Isometry3d(1, m_shape_poses[shape_id]),
      std::vector<collision_detection::CollisionObjectType>(1, m_collision_object_types[shape_id]), active);
  cow->setContactDistanceThreshold(m_contact_distance);
  return cow;
}

bool collisionCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data)
{
  auto* cdata = reinterpret_cast<collision_detection::ContactTestData*>(data);

  if (cdata->done)
    return true;

  const auto* cd1 = static_cast<const CollisionObjectWrapper*>(o1->getUserData());
  const auto* cd2 = static_cast<const CollisionObjectWrapper*>(o2->getUserData());
  if (cd1->sameObject(*cd2))
    return false;

  bool needs_collision = cd1->m_enabled && cd2->m_enabled &&
                         (cd1->m_collisionFilterGroup & cd2->m_collisionFilterMask) &&  // NOLINT
                         (cd2->m_collisionFilterGroup & cd1->m_collisionFilterMask);

  if (!needs_collision)
    return false;

  // If active components are specified
  const std::string link_name1 =
      cd1->getTypeID() == collision_detection::BodyTypes::ROBOT_LINK ?
          cd1->getName() :
          (cd1->getTypeID() == collision_detection::BodyTypes::ROBOT_ATTACHED ? cd1->getPName() : "");
  const std::string link_name2 =
      cd2->getTypeID() == collision_detection::BodyTypes::ROBOT_LINK ?
          cd2->getName() :
          (cd2->getTypeID() == collision_detection::BodyTypes::ROBOT_ATTACHED ? cd2->getPName() : "");

  // If neither of the involved components is active
  if ((link_name1.empty() || !collision_detection::isLinkActive(cdata->active_components_only, link_name1)) &&
      (link_name2.empty() || !collision_detection::isLinkActive(cdata->active_components_only, link_name2)))
    return false;

  if (cd1->getTypeID() == collision_detection::BodyType::ROBOT_ATTACHED &&
      cd2->getTypeID() == collision_detection::BodyType::ROBOT_LINK)
  {
    if (cd1->m_touch_links.find(cd2->getName()) != cd1->m_touch_links.end())
      return false;
  }

  if (cd2->getTypeID() == collision_detection::BodyType::ROBOT_ATTACHED &&
      cd1->getTypeID() == collision_detection::BodyType::ROBOT_LINK)
  {
    if (cd2->m_touch_links.find(cd1->getName()) != cd2->m_touch_links.end())
      return false;
  }

  if (cd1->getTypeID() == collision_detection::BodyType::ROBOT_ATTACHED &&
      cd2->getTypeID() == collision_detection::BodyType::ROBOT_ATTACHED)
  {
    if (cd1->m_touch_links == cd2->m_touch_links)
      return false;
  }

  // use the collision matrix (if any) to avoid certain collision checks
  collision_detection::DecideContactFn dcf;
  if (cdata->acm)
  {
    collision_detection::AllowedCollision::Type type;
    bool found = cdata->acm->getAllowedCollision(cd1->getName(), cd2->getName(), type);

    if (found)
    {
      // if we have an entry in the collision matrix, we read it
      if (type == collision_detection::AllowedCollision::ALWAYS)
      {
        return false;
      }
      else if (type == collision_detection::AllowedCollision::CONDITIONAL)
      {
        cdata->acm->getAllowedCollision(cd1->getName(), cd2->getName(), dcf);
      }
    }
  }

  bool enable_contact = (cdata->req->contacts || cdata->req->distance);

  // see if we need to compute a contact
  std::size_t want_contact_count = 1;
  std::pair<std::string, std::string> pc = collision_detection::getObjectPairKey(cd1->getName(), cd2->getName());
  if (cdata->req->contacts && cdata->res->contact_count < cdata->req->max_contacts)
  {
    std::size_t have =
        cdata->res->contacts.find(pc) != cdata->res->contacts.end() ? cdata->res->contacts[pc].size() : 0;
    if (have < cdata->req->max_contacts_per_pair)
      want_contact_count =
          std::min(cdata->req->max_contacts_per_pair - have, cdata->req->max_contacts - cdata->res->contact_count);
  }

  if (dcf || cdata->negative_distance <= 0.0)
  {
    enable_contact = true;
    want_contact_count = std::numeric_limits<std::size_t>::max();
  }

  bool enable_cost = cdata->req->cost;
  std::size_t num_max_cost_sources = cdata->req->max_cost_sources;

  fcl::CollisionResultd col_result;
  if (fcl::collide(o1, o2, fcl::CollisionRequestd(want_contact_count, enable_contact, num_max_cost_sources, enable_cost),
                   col_result))
  {
    const Eigen::Isometry3d& tf1 = cd1->getCollisionObjectsTransform();
    const Eigen::Isometry3d& tf2 = cd2->getCollisionObjectsTransform();
    const Eigen::Isometry3d& tf1_inv = tf1.inverse();
    const Eigen::Isometry3d& tf2_inv = tf2.inverse();

    for (std::size_t i = 0; i < col_result.numContacts(); ++i)
    {
      const fcl::Contactd& fcl_contact = col_result.getContact(i);

      bool is_swapped = o1->collisionGeometry().get() != fcl_contact.o1;

      collision_detection::Contact contact;
      contact.body_name_1 = cd1->getName();
      contact.body_name_2 = cd2->getName();
      contact.body_type_1 = cd1->getTypeID();
      contact.body_type_2 = cd2->getTypeID();
      contact.transform[0] = tf1;
      contact.transform[1] = tf2;

      if (is_swapped)
      {
        contact.shape_id[0] = cd1->getShapeIndex(fcl_contact.o2);
        contact.shape_id[1] = cd2->getShapeIndex(fcl_contact.o1);
        if (contact.shape_id[0] == -1 || contact.shape_id[1] == -1)
        {
          ROS_WARN_NAMED("collision_detection_fcl",
                         "The shape id are invalid with swap. Without swap, the values are %d and %d",
                         cd1->getShapeIndex(fcl_contact.o1), cd2->getShapeIndex(fcl_contact.o2));
        }
        if (enable_contact)
        {
          contact.pos = fcl_contact.pos;
          contact.normal = -fcl_contact.normal;
          contact.depth = -fcl_contact.penetration_depth;
          contact.nearest_points[0] = contact.pos;
          contact.nearest_points[1] = fcl_contact.pos + fcl_contact.normal * fcl_contact.penetration_depth;
        }
      }
      else
      {
        contact.shape_id[0] = cd1->getShapeIndex(fcl_contact.o1);
        contact.shape_id[1] = cd2->getShapeIndex(fcl_contact.o2);
        if (contact.shape_id[0] == -1 || contact.shape_id[1] == -1)
        {
          ROS_WARN_NAMED("collision_detection_fcl",
                         "The shape id are invalid without swap. With swap, the values are %d and %d",
                         cd1->getShapeIndex(fcl_contact.o2), cd2->getShapeIndex(fcl_contact.o1));
        }
        if (enable_contact)
        {
          contact.pos = fcl_contact.pos + fcl_contact.normal * fcl_contact.penetration_depth;
          contact.normal = fcl_contact.normal;
          contact.depth = -fcl_contact.penetration_depth;
          contact.nearest_points[0] = contact.pos;
          contact.nearest_points[1] = fcl_contact.pos;
        }
      }

      if (enable_contact)
      {
        contact.nearest_points_local[0] = tf1_inv * contact.nearest_points[0];
        contact.nearest_points_local[1] = tf2_inv * contact.nearest_points[1];
        contact.nearest_points_local2[0] = tf1_inv * contact.nearest_points[1];
        contact.nearest_points_local2[1] = tf2_inv * contact.nearest_points[0];
      }

      if (dcf ? !dcf(contact) : true)
      {
        const auto& it = cdata->res->contacts.find(pc);
        bool found = (it != cdata->res->contacts.end());

        cdata->res->collision = true;
        processResult(*cdata, contact, pc, found);
      }
    }
  }

  if (enable_cost)
  {
    std::vector<fcl::CostSourced> cost_sources;
    col_result.getCostSources(cost_sources);

    for (auto& fcs : cost_sources)
    {
      collision_detection::CostSource cs;

      cs.aabb_min[0] = fcs.aabb_min[0];
      cs.aabb_min[1] = fcs.aabb_min[1];
      cs.aabb_min[2] = fcs.aabb_min[2];

      cs.aabb_max[0] = fcs.aabb_max[0];
      cs.aabb_max[1] = fcs.aabb_max[1];
      cs.aabb_max[2] = fcs.aabb_max[2];

      cs.cost = fcs.cost_density;

      cdata->res->cost_sources.insert(cs);
      while (cdata->res->cost_sources.size() > cdata->req->max_cost_sources)
        cdata->res->cost_sources.erase(--cdata->res->cost_sources.end());
    }
  }

  return cdata->done;
}

bool distanceCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data, double& /*min_dist*/)
{
  auto* cdata = reinterpret_cast<collision_detection::DistanceTestData*>(data);

  if (cdata->done)
    return true;

  // fcl::distance segfaults when given an octree with a null root pointer (using FCL 0.6.1)
  if ((o1->getObjectType() == fcl::OT_OCTREE &&
       !std::static_pointer_cast<const fcl::OcTreed>(o1->collisionGeometry())->getRoot()) ||
      (o2->getObjectType() == fcl::OT_OCTREE &&
       !std::static_pointer_cast<const fcl::OcTreed>(o2->collisionGeometry())->getRoot()))
  {
    return false;
  }

  const auto* cd1 = static_cast<const CollisionObjectWrapper*>(o1->getUserData());
  const auto* cd2 = static_cast<const CollisionObjectWrapper*>(o2->getUserData());
  if (cd1->sameObject(*cd2))
    return false;

  bool needs_collision = cd1->m_enabled && cd2->m_enabled &&
                         (cd1->m_collisionFilterGroup & cd2->m_collisionFilterMask) &&  // NOLINT
                         (cd2->m_collisionFilterGroup & cd1->m_collisionFilterMask);

  if (!needs_collision)
    return false;

  // If active components are specified
  const std::string link_name1 =
      cd1->getTypeID() == collision_detection::BodyTypes::ROBOT_LINK ?
          cd1->getName() :
          (cd1->getTypeID() == collision_detection::BodyTypes::ROBOT_ATTACHED ? cd1->getPName() : "");
  const std::string link_name2 =
      cd2->getTypeID() == collision_detection::BodyTypes::ROBOT_LINK ?
          cd2->getName() :
          (cd2->getTypeID() == collision_detection::BodyTypes::ROBOT_ATTACHED ? cd2->getPName() : "");

  // If neither of the involved components is active
  if ((link_name1.empty() || !collision_detection::isLinkActive(cdata->req->active_components_only, link_name1)) &&
      (link_name2.empty() || !collision_detection::isLinkActive(cdata->req->active_components_only, link_name2)))
    return false;

  if (cd1->getTypeID() == collision_detection::BodyType::ROBOT_ATTACHED &&
      cd2->getTypeID() == collision_detection::BodyType::ROBOT_LINK)
  {
    if (cd1->m_touch_links.find(cd2->getName()) != cd1->m_touch_links.end())
      return false;
  }

  if (cd2->getTypeID() == collision_detection::BodyType::ROBOT_ATTACHED &&
      cd1->getTypeID() == collision_detection::BodyType::ROBOT_LINK)
  {
    if (cd2->m_touch_links.find(cd1->getName()) != cd2->m_touch_links.end())
      return false;
  }

  if (cd1->getTypeID() == collision_detection::BodyType::ROBOT_ATTACHED &&
      cd2->getTypeID() == collision_detection::BodyType::ROBOT_ATTACHED)
  {
    if (cd1->m_touch_links == cd2->m_touch_links)
      return false;
  }

  if (cdata->req->acm)
  {
    collision_detection::AllowedCollision::Type type;
    bool found = cdata->req->acm->getAllowedCollision(cd1->getName(), cd2->getName(), type);
    if (found && type == collision_detection::AllowedCollision::ALWAYS)
    {
      return false;
    }
  }

  double dist_threshold = std::min(cdata->req->distance_threshold, cdata->res->minimum_distance.distance);

  const std::pair<std::string, std::string>& pc = collision_detection::getObjectPairKey(cd1->getName(), cd2->getName());
  collision_detection::DistanceMap::iterator it = cdata->res->distances.find(pc);

  // GLOBAL search: for efficiency, distance_threshold starts at the smallest distance between any pairs found so far
  if (cdata->req->type == collision_detection::DistanceRequestType::GLOBAL)
  {
    dist_threshold = cdata->res->minimum_distance.distance;
  }
  // Check if a distance between this pair has been found yet. Decrease threshold_distance if so, to narrow the search
  else if (it != cdata->res->distances.end())
  {
    dist_threshold = it->second[0].distance;
    if (cdata->req->type == collision_detection::DistanceRequestType::LIMITED &&
        it->second.size() >= cdata->req->max_contacts_per_body)
    {
      return cdata->done;
    }
  }

  fcl::DistanceResultd fcl_result;
  fcl_result.min_distance = dist_threshold;

  double distance = fcl::distance(
      o1, o2, fcl::DistanceRequestd(cdata->req->enable_nearest_points, cdata->req->enable_signed_distance), fcl_result);

  // Check if either object is already in the map. If not add it or if present
  // check to see if the new distance is closer. If closer remove the existing
  // one and add the new distance information.
  if (distance < dist_threshold)
  {
    collision_detection::DistanceResultsData dist_result;
    dist_result.distance = fcl_result.min_distance;

    if (cd1->getTypeID() == collision_detection::BodyTypes::ROBOT_ATTACHED ||
        cd2->getTypeID() == collision_detection::BodyTypes::ROBOT_ATTACHED)
    {
      if (dist_result.distance < 0.0)
      {
        cdata->res->collision = true;
      }
    }
    else if (dist_result.distance < cdata->safety_distance)
    {
      cdata->res->collision = true;
    }

    bool is_swapped = o1->collisionGeometry().get() != fcl_result.o1;

    const Eigen::Isometry3d& tf1 = cd1->getCollisionObjectsTransform();
    const Eigen::Isometry3d& tf2 = cd2->getCollisionObjectsTransform();
    const Eigen::Isometry3d& tf1_inv = tf1.inverse();
    const Eigen::Isometry3d& tf2_inv = tf2.inverse();

    dist_result.link_names[0] = cd1->getName();
    dist_result.link_names[1] = cd2->getName();
    dist_result.body_types[0] = cd1->getTypeID();
    dist_result.body_types[1] = cd2->getTypeID();
    dist_result.transform[0] = tf1;
    dist_result.transform[1] = tf2;

    if (is_swapped)
    {
      dist_result.shape_id[0] = cd1->getShapeIndex(fcl_result.o2);
      dist_result.shape_id[1] = cd2->getShapeIndex(fcl_result.o1);
      if (dist_result.shape_id[0] == -1 || dist_result.shape_id[1] == -1)
      {
        ROS_WARN_NAMED("collision_detection_fcl",
                       "The shape id are invalid with swap. Without swap, the values are %d and %d",
                       cd1->getShapeIndex(fcl_result.o1), cd2->getShapeIndex(fcl_result.o2));
      }
      if (cdata->req->enable_nearest_points)
      {
        dist_result.nearest_points[0] = fcl_result.nearest_points[1];
        dist_result.nearest_points[1] = fcl_result.nearest_points[0];
      }
    }
    else
    {
      dist_result.shape_id[0] = cd1->getShapeIndex(fcl_result.o1);
      dist_result.shape_id[1] = cd2->getShapeIndex(fcl_result.o2);
      if (dist_result.shape_id[0] == -1 || dist_result.shape_id[1] == -1)
      {
        ROS_WARN_NAMED("collision_detection_fcl",
                       "The shape id are invalid without swap. With swap, the values are %d and %d",
                       cd1->getShapeIndex(fcl_result.o2), cd2->getShapeIndex(fcl_result.o1));
      }
      if (cdata->req->enable_nearest_points)
      {
        dist_result.nearest_points[0] = fcl_result.nearest_points[0];
        dist_result.nearest_points[1] = fcl_result.nearest_points[1];
      }
    }

    if (cdata->req->enable_nearest_points)
    {
      dist_result.nearest_points_local[0] = tf1_inv * dist_result.nearest_points[0];
      dist_result.nearest_points_local[1] = tf2_inv * dist_result.nearest_points[1];
      dist_result.nearest_points_local2[0] = tf1_inv * dist_result.nearest_points[1];
      dist_result.nearest_points_local2[1] = tf2_inv * dist_result.nearest_points[0];
    }

    if (cdata->req->compute_gradient)
    {
      dist_result.normal = (dist_result.nearest_points[1] - dist_result.nearest_points[0]).normalized();
      if (distance < 0.0)
        dist_result.normal = -dist_result.normal;
    }

    if (dist_result.distance < cdata->res->minimum_distance.distance)
    {
      cdata->res->minimum_distance = dist_result;
    }

    if (cdata->req->type != collision_detection::DistanceRequestType::GLOBAL)
    {
      if (it == cdata->res->distances.end())
      {
        std::vector<collision_detection::DistanceResultsData> data;
        data.reserve(cdata->req->type == collision_detection::DistanceRequestType::SINGLE ?
                         1 :
                         cdata->req->max_contacts_per_body);
        data.push_back(dist_result);
        cdata->res->distances.insert(std::make_pair(pc, data));
      }
      else
      {
        if (cdata->req->type == collision_detection::DistanceRequestType::ALL)
        {
          it->second.push_back(dist_result);
        }
        else if (cdata->req->type == collision_detection::DistanceRequestType::SINGLE)
        {
          if (dist_result.distance < it->second[0].distance)
            it->second[0] = dist_result;
        }
        else if (cdata->req->type == collision_detection::DistanceRequestType::LIMITED)
        {
          assert(it->second.size() < cdata->req->max_contacts_per_body);
          it->second.push_back(dist_result);
        }
      }
    }

    if ((!cdata->req->enable_signed_distance && cdata->res->collision) ||
        (cdata->negative_distance <= 0.0 && dist_result.distance < cdata->negative_distance))
    {
      cdata->done = true;
    }
  }

  return cdata->done;
}
}  // namespace collision_detection_fcl
