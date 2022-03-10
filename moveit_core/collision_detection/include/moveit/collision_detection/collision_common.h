/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Shi Shenglei */

#pragma once

#include <boost/array.hpp>
#include <boost/function.hpp>
#include <vector>
#include <string>
#include <map>
#include <set>
#include <Eigen/Core>
#include <moveit/robot_model/robot_model.h>

namespace collision_detection
{
MOVEIT_CLASS_FORWARD(AllowedCollisionMatrix);  // Defines AllowedCollisionMatrixPtr, ConstPtr, WeakPtr... etc

/** \brief The types of bodies that are considered for collision */
namespace BodyTypes
{
/** \brief The types of bodies that are considered for collision */
enum Type
{
  /** \brief A link on the robot */
  ROBOT_LINK,

  /** \brief A body attached to a robot link */
  ROBOT_ATTACHED,

  /** \brief A body in the environment */
  WORLD_OBJECT
};
}  // namespace BodyTypes

/** \brief The types of bodies that are considered for collision */
using BodyType = BodyTypes::Type;

enum class ContinuousCollisionType
{
  CCTYPE_NONE,
  CCTYPE_TIME0,
  CCTYPE_TIME1,
  CCTYPE_BETWEEN
};

/** \brief Definition of a contact point */
struct Contact
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** \brief contact position */
  Eigen::Vector3d pos;

  /** \brief normal unit vector at contact */
  Eigen::Vector3d normal;

  /** \brief depth (penetration between bodies) */
  double depth;

  /** \brief The id of the first body involved in the contact */
  std::string body_name_1;

  /** \brief The type of the first body involved in the contact */
  BodyType body_type_1;

  /** \brief The id of the second body involved in the contact */
  std::string body_name_2;

  /** \brief The type of the second body involved in the contact */
  BodyType body_type_2;

  /** \brief The distance percentage between casted poses until collision.
   *
   *  If the value is 0, then the collision occured in the start pose. If the value is 1, then the collision occured in
   *  the end pose. */
  double percent_interpolation;

  /** \brief The two nearest points connecting the two bodies */
  Eigen::Vector3d nearest_points[2];

  Eigen::Vector3d nearest_points_local[2];

  Eigen::Vector3d nearest_points_local2[2];

  int shape_id[2];

  Eigen::Isometry3d transform[2];

  double cc_time[2];

  std::array<ContinuousCollisionType, 2> cc_type;

  Eigen::Isometry3d cc_transform[2];

  Contact()
  {
    clear();
  }

  void clear()
  {
    depth = std::numeric_limits<double>::max();
    pos.setZero();
    normal.setZero();
    body_name_1 = "";
    body_name_2 = "";
    body_type_1 = BodyType::WORLD_OBJECT;
    body_type_2 = BodyType::WORLD_OBJECT;
    percent_interpolation = -1.0;
    nearest_points[0].setZero();
    nearest_points[1].setZero();
    nearest_points_local[0].setZero();
    nearest_points_local[1].setZero();
    nearest_points_local2[0].setZero();
    nearest_points_local2[1].setZero();
    transform[0] = Eigen::Isometry3d::Identity();
    transform[1] = Eigen::Isometry3d::Identity();
    shape_id[0] = -1;
    shape_id[1] = -1;
    cc_time[0] = -1;
    cc_time[1] = -1;
    cc_type[0] = ContinuousCollisionType::CCTYPE_NONE;
    cc_type[1] = ContinuousCollisionType::CCTYPE_NONE;
    cc_transform[0] = Eigen::Isometry3d::Identity();
    cc_transform[1] = Eigen::Isometry3d::Identity();
  }
};

/** \brief When collision costs are computed, this structure contains information about the partial cost incurred in a
 * particular volume */
struct CostSource
{
  /// The minimum bound of the AABB defining the volume responsible for this partial cost
  boost::array<double, 3> aabb_min;

  /// The maximum bound of the AABB defining the volume responsible for this partial cost
  boost::array<double, 3> aabb_max;

  /// The partial cost (the probability of existance for the object there is a collision with)
  double cost{ std::numeric_limits<double>::max() };

  /// Get the volume of the AABB around the cost source
  double getVolume() const
  {
    return (aabb_max[0] - aabb_min[0]) * (aabb_max[1] - aabb_min[1]) * (aabb_max[2] - aabb_min[2]);
  }

  /// Order cost sources so that the most costly source is at the top
  bool operator<(const CostSource& other) const
  {
    double c1 = cost * getVolume();
    double c2 = other.cost * other.getVolume();
    if (c1 > c2)
      return true;
    if (c1 < c2)
      return false;
    if (cost < other.cost)
      return false;
    if (cost > other.cost)
      return true;
    return aabb_min < other.aabb_min;
  }
};

/** \brief Representation of a collision checking result */
struct CollisionResult
{
  CollisionResult() : collision(false), distance(std::numeric_limits<double>::max()), contact_count(0)
  {
  }
  using ContactMap = std::map<std::pair<std::string, std::string>, std::vector<Contact> >;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** \brief Clear a previously stored result */
  void clear()
  {
    collision = false;
    distance = std::numeric_limits<double>::max();
    contact_count = 0;
    contacts.clear();
    cost_sources.clear();
  }

  /** \brief Throttled warning printing the first collision pair, if any. All collisions are logged at DEBUG level */
  void print() const;

  /** \brief True if collision was found, false otherwise */
  bool collision;

  /** \brief Closest distance between two bodies */
  double distance;

  /** \brief Number of contacts returned */
  std::size_t contact_count;

  /** \brief A map returning the pairs of body ids in contact, plus their contact details */
  ContactMap contacts;

  /** \brief These are the individual cost sources when costs are computed */
  std::set<CostSource> cost_sources;
};

/** \brief Representation of a collision checking request */
struct CollisionRequest
{
  CollisionRequest()
    : group_name("")
    , distance(false)
    , cost(false)
    , contacts(false)
    , max_contacts(1)
    , max_contacts_per_pair(1)
    , max_cost_sources(1)
    , verbose(false)
  {
  }

  virtual ~CollisionRequest()
  {
  }

  /** \brief The group name to check collisions for (optional; if empty, assume the complete robot) */
  std::string group_name;

  /** \brief If true, compute proximity distance */
  bool distance;

  /** \brief If true, a collision cost is computed */
  bool cost;

  /** \brief If true, compute contacts. Otherwise only a binary collision yes/no is reported. */
  bool contacts;

  /** \brief Overall maximum number of contacts to compute */
  std::size_t max_contacts;

  /** \brief Maximum number of contacts to compute per pair of bodies (multiple bodies may be in contact at different
   * configurations) */
  std::size_t max_contacts_per_pair;

  /** \brief When costs are computed, this value defines how many of the top cost sources should be returned */
  std::size_t max_cost_sources;

  /** \brief Function call that decides whether collision detection should stop. */
  boost::function<bool(const CollisionResult&)> is_done;

  /** \brief Flag indicating whether information about detected collisions should be reported */
  bool verbose;
};

namespace DistanceRequestTypes
{
enum DistanceRequestType
{
  GLOBAL,   ///< Find the global minimum
  SINGLE,   ///< Find the global minimum for each pair
  LIMITED,  ///< Find a limited(max_contacts_per_body) set of contacts for a given pair
  ALL       ///< Find all the contacts for a given pair
};
}
using DistanceRequestType = DistanceRequestTypes::DistanceRequestType;

/** \brief Representation of a distance-reporting request */
struct DistanceRequest
{
  DistanceRequest()
    : group_name("")
    , enable_nearest_points(false)
    , enable_signed_distance(false)
    , type(DistanceRequestType::GLOBAL)
    , max_contacts_per_body(1)
    , acm(nullptr)
    , distance_threshold(std::numeric_limits<double>::max())
    , verbose(false)
    , compute_gradient(false)
  {
  }

  /// The group name
  std::string group_name;

  /// Compute \e active_components_only_ based on \e req
  void enableGroup(const moveit::core::RobotModelConstPtr& robot_model)
  {
    if (robot_model->hasJointModelGroup(group_name))
      active_components_only = robot_model->getJointModelGroup(group_name)->getUpdatedLinkModelsWithGeometryNames();
    else
      active_components_only.clear();
  }

  /// The set of active components to check
  std::vector<std::string> active_components_only;

  /// Indicate if nearest point information should be calculated
  bool enable_nearest_points;

  /// Indicate if a signed distance should be calculated in a collision.
  bool enable_signed_distance;

  /// Indicate the type of distance request. If using type=ALL, it is
  /// recommended to set max_contacts_per_body to the expected number
  /// of contacts per pair because it is used to reserve space.
  DistanceRequestType type;

  /// Maximum number of contacts to store for bodies (multiple bodies may be within distance threshold)
  std::size_t max_contacts_per_body;

  /// The allowed collision matrix used to filter checks
  const AllowedCollisionMatrix* acm;

  /// Only calculate distances for objects within this threshold to each other.
  /// If set, this can significantly reduce the number of queries.
  double distance_threshold;

  /// Log debug information
  bool verbose;

  /// Indicate if gradient should be calculated between each object.
  /// This is the normalized vector connecting the closest points on the two objects.
  bool compute_gradient;
};

/** \brief Generic representation of the distance information for a pair of objects */
struct DistanceResultsData  // NOLINT(readability-identifier-naming) - suppress spurious clang-tidy warning
{
  DistanceResultsData()
  {
    clear();
  }

  /// The distance between two objects. If two objects are in collision, distance <= 0.
  double distance;

  /// The nearest points
  Eigen::Vector3d nearest_points[2];

  Eigen::Vector3d nearest_points_local[2];

  Eigen::Vector3d nearest_points_local2[2];

  /// The object link names
  std::string link_names[2];

  /// The object body type
  BodyType body_types[2];

  /** Normalized vector connecting closest points (from link_names[0] to link_names[1])

    Usually, when checking convex to convex, the normal is connecting closest points.
    However, FCL in case of non-convex to non-convex or convex to non-convex returns
    the contact normal for one of the two triangles that are in contact. */
  Eigen::Vector3d normal;

  int shape_id[2];

  Eigen::Isometry3d transform[2];

  /// Clear structure data
  void clear()
  {
    distance = std::numeric_limits<double>::max();
    nearest_points[0].setZero();
    nearest_points[1].setZero();
    nearest_points_local[0].setZero();
    nearest_points_local[1].setZero();
    nearest_points_local2[0].setZero();
    nearest_points_local2[1].setZero();
    body_types[0] = BodyType::WORLD_OBJECT;
    body_types[1] = BodyType::WORLD_OBJECT;
    link_names[0] = "";
    link_names[1] = "";
    normal.setZero();
    shape_id[0] = -1;
    shape_id[1] = -1;
    transform[0] = Eigen::Isometry3d::Identity();
    transform[1] = Eigen::Isometry3d::Identity();
  }

  /// Compare if the distance is less than another
  bool operator<(const DistanceResultsData& other)
  {
    return (distance < other.distance);
  }

  /// Compare if the distance is greater than another
  bool operator>(const DistanceResultsData& other)
  {
    return (distance > other.distance);
  }
};

/** \brief Mapping between the names of the collision objects and the DistanceResultData. */
using DistanceMap = std::map<const std::pair<std::string, std::string>, std::vector<DistanceResultsData> >;

/** \brief Result of a distance request. */
struct DistanceResult
{
  DistanceResult() : collision(false)
  {
  }

  /// Indicates if two objects were in collision
  bool collision;

  /// ResultsData for the two objects with the minimum distance
  DistanceResultsData minimum_distance;

  /// A map of distance data for each link in the req.active_components_only
  DistanceMap distances;

  /// Clear structure data
  void clear()
  {
    collision = false;
    minimum_distance.clear();
    distances.clear();
  }
};

enum class CollisionObjectType
{
  USE_SHAPE_TYPE = 0, /**< @brief Infer the type from the type specified in the shapes::Shape class */

  // all of the following convert the meshes to custom collision objects
  CONVEX_HULL = 1,  /**< @brief Use the mesh in shapes::Shape but make it a convex hulls collision object (if not convex
                      it will be converted) */
  MULTI_SPHERE = 2, /**< @brief Use the mesh and represent it by multiple spheres collision object */
  SDF = 3           /**< @brief Use the mesh and rpresent it by a signed distance fields collision object */
};

/** \brief Bundles the data for a collision query */
struct ContactTestData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ContactTestData(double safety_distance, double contact_distance, double negative_distance,
                  const AllowedCollisionMatrix* acm, const CollisionRequest* req, CollisionResult* res)
    : safety_distance(safety_distance)
    , contact_distance(contact_distance)
    , negative_distance(negative_distance)
    , acm(acm)
    , req(req)
    , res(res)
    , done(false)
    , pair_done(false)
  {
  }

  void enableGroup(const moveit::core::RobotModelConstPtr& robot_model)
  {
    if (robot_model->hasJointModelGroup(req->group_name))
      active_components_only =
          robot_model->getJointModelGroup(req->group_name)->getUpdatedLinkModelsWithGeometryNames();
    else
      active_components_only.clear();
  }

  /** \brief  If the collision request includes a group name, this set contains the pointers to the link models that
   *  are considered for collision.
   *
   *  If the pointer is NULL, all collisions are considered. */
  std::vector<std::string> active_components_only;

  double safety_distance;

  /** \brief If after a positive broadphase check the distance is below this threshold, a contact is added. */
  double contact_distance;

  double negative_distance;

  /** \brief The user-specified collision matrix (may be NULL). */
  const AllowedCollisionMatrix* acm;

  const CollisionRequest* req;

  CollisionResult* res;

  /// Indicates if search is finished
  bool done;

  /// Indicates if search between a single pair is finished
  bool pair_done;
};

/** \brief Data structure which is passed to the distance callback function of the collision manager. */
struct DistanceTestData
{
  DistanceTestData(double safety_distance, double contact_distance, double negative_distance,
                   const DistanceRequest* req, DistanceResult* res)
    : safety_distance(safety_distance)
    , contact_distance(contact_distance)
    , negative_distance(negative_distance)
    , req(req)
    , res(res)
    , done(false)
  {
  }

  ~DistanceTestData()
  {
  }

  double safety_distance;

  /** \brief If after a positive broadphase check the distance is below this threshold, a contact is added. */
  double contact_distance;

  double negative_distance;

  /** \brief Distance query request information. */
  const DistanceRequest* req;

  /** \brief Distance query results information. */
  DistanceResult* res;

  /** \brief Indicates if distance query is finished. */
  bool done;
};
}  // namespace collision_detection
