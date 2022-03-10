/*********************************************************************
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2018, Southwest Research Institute
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************************************************************/

/* Author: Levi Armstrong, Shi Shenglei */

#pragma once

#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <ros/console.h>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_matrix.h>

namespace collision_detection
{
/**
 * @brief Get a key for two object to search the collision matrix
 * @param obj1 First collision object name
 * @param obj2 Second collision object name
 * @return The collision pair key
 */
inline std::pair<std::string, std::string> getObjectPairKey(const std::string& obj1, const std::string& obj2)
{
  return obj1 < obj2 ? std::make_pair(obj1, obj2) : std::make_pair(obj2, obj1);
}

/**
 * @brief This will check if a link is active provided a list. If the list is empty the link is considered active.
 * @param active List of active link names
 * @param name The name of link to check if it is active.
 */
inline bool isLinkActive(const std::vector<std::string>& active, const std::string& name)
{
  return active.empty() || (std::find(active.begin(), active.end(), name) != active.end());
}

/** \brief Allowed = true */
inline bool acmCheck(const std::string& body_1, const std::string& body_2, const AllowedCollisionMatrix* acm)
{
  AllowedCollision::Type allowed_type;

  if (acm != nullptr)
  {
    if (acm->getAllowedCollision(body_1, body_2, allowed_type))
    {
      if (allowed_type == AllowedCollision::Type::NEVER)
      {
        ROS_DEBUG_STREAM_NAMED("collision_detection", "Not allowed entry in ACM found, collision check between "
                                                          << body_1 << " and " << body_2);
        return false;
      }
      else
      {
        ROS_DEBUG_STREAM_NAMED("collision_detection", "Entry in ACM found, skipping collision check as allowed "
                                                          << body_1 << " and " << body_2);
        return true;
      }
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED("collision_detection",
                             "No entry in ACM found, collision check between " << body_1 << " and " << body_2);
      return false;
    }
  }
  else
  {
    ROS_DEBUG_STREAM_NAMED("collision_detection", "No ACM, collision check between " << body_1 << " and " << body_2);
    return false;
  }
}

/** \brief Stores a single contact result in the requested way.
 *   \param found Indicates if a contact for this pair of objects has already been found
 *   \return Pointer to the newly inserted contact */
inline Contact* processResult(ContactTestData& cdata, Contact& contact, const std::pair<std::string, std::string>& key,
                              bool found)
{
  // add deepest penetration / smallest distance to result
  if (cdata.req->distance)
  {
    if (contact.depth < cdata.res->distance)
    {
      cdata.res->distance = contact.depth;
    }
  }

  if (contact.depth < cdata.safety_distance)
  {
    cdata.res->collision = true;
  }

  if (cdata.negative_distance <= 0.0 && contact.depth < cdata.negative_distance)
  {
    cdata.done = true;
  }

  ROS_DEBUG_STREAM_NAMED("collision_detection",
                         "Contact btw " << key.first << " and " << key.second << " dist: " << contact.depth);
  // case if pair hasn't a contact yet
  if (!found)
  {
    std::vector<Contact> data;

    // if we dont want contacts we are done here
    if (!cdata.req->contacts)
    {
      if (!cdata.req->distance || cdata.res->collision)
      {
        cdata.done = true;

        if (cdata.negative_distance <= 0.0 && contact.depth > cdata.negative_distance)
        {
          cdata.done = false;
        }
      }
      return nullptr;
    }
    else
    {
      data.reserve(cdata.req->max_contacts_per_pair);
      data.emplace_back(contact);
      cdata.res->contact_count++;
    }

    if (cdata.res->contact_count >= cdata.req->max_contacts)
    {
      if (!cdata.req->distance || cdata.res->collision)
      {
        cdata.done = true;

        if (cdata.negative_distance <= 0.0 && contact.depth > cdata.negative_distance)
        {
          cdata.done = false;
        }
      }
    }

    if (cdata.req->max_contacts_per_pair == 1u && cdata.negative_distance > 0 &&
        (!cdata.req->distance || cdata.res->collision))
    {
      cdata.pair_done = true;
    }

    return &(cdata.res->contacts.insert(std::make_pair(key, data)).first->second.back());
  }
  else
  {
    std::vector<Contact>& dr = cdata.res->contacts[key];
    dr.emplace_back(contact);
    cdata.res->contact_count++;

    if (dr.size() >= cdata.req->max_contacts_per_pair && cdata.negative_distance > 0 &&
        (!cdata.req->distance || cdata.res->collision))
    {
      cdata.pair_done = true;
    }

    if (cdata.res->contact_count >= cdata.req->max_contacts)
    {
      if (!cdata.req->distance || cdata.res->collision)
      {
        cdata.done = true;

        if (cdata.negative_distance <= 0.0 && contact.depth > cdata.negative_distance)
        {
          cdata.done = false;
        }
      }
    }

    return &(dr.back());
  }

  if (!cdata.done && cdata.req->is_done)
  {
    cdata.done = cdata.req->is_done(*cdata.res);
  }

  return nullptr;
}

/** \brief Recursively traverses robot from root to get all active links
 *
 *   \param active_links Stores the active links
 *   \param urdf_link The current urdf link representation
 *   \param active Indicates if link is considered active */
static inline void getActiveLinkNamesRecursive(std::vector<std::string>& active_links,
                                               const urdf::LinkConstSharedPtr& urdf_link, bool active)
{
  if (active)
  {
    active_links.push_back(urdf_link->name);
    for (const auto& child_link : urdf_link->child_links)
    {
      getActiveLinkNamesRecursive(active_links, child_link, active);
    }
  }
  else
  {
    for (std::size_t i = 0; i < urdf_link->child_links.size(); ++i)
    {
      const urdf::LinkConstSharedPtr child_link = urdf_link->child_links[i];
      if ((child_link->parent_joint) && (child_link->parent_joint->type != urdf::Joint::FIXED))
        getActiveLinkNamesRecursive(active_links, child_link, true);
      else
        getActiveLinkNamesRecursive(active_links, child_link, active);
    }
  }
}

inline shapes::ShapePtr constructShape(const urdf::Geometry* geom)
{
  shapes::Shape* result = nullptr;
  switch (geom->type)
  {
    case urdf::Geometry::SPHERE:
      result = new shapes::Sphere(static_cast<const urdf::Sphere*>(geom)->radius);
      break;
    case urdf::Geometry::BOX:
    {
      urdf::Vector3 dim = static_cast<const urdf::Box*>(geom)->dim;
      result = new shapes::Box(dim.x, dim.y, dim.z);
    }
    break;
    case urdf::Geometry::CYLINDER:
      result = new shapes::Cylinder(static_cast<const urdf::Cylinder*>(geom)->radius,
                                    static_cast<const urdf::Cylinder*>(geom)->length);
      break;
    case urdf::Geometry::MESH:
    {
      const urdf::Mesh* mesh = static_cast<const urdf::Mesh*>(geom);
      if (!mesh->filename.empty())
      {
        Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
        shapes::Mesh* m = shapes::createMeshFromResource(mesh->filename, scale);
        result = m;
      }
    }
    break;
    default:
      ROS_ERROR("Unknown geometry type: %d", static_cast<int>(geom->type));
      break;
  }

  return shapes::ShapePtr(result);
}

inline Eigen::Isometry3d urdfPose2Eigen(const urdf::Pose& pose)
{
  Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
  Eigen::Isometry3d result;
  result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  result.linear() = q.toRotationMatrix();
  return result;
}
}  // namespace collision_detection
