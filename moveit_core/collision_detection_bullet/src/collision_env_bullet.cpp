/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Jens Petit
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
 *   * Neither the name of the copyright holder nor the names of its
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

/* Author: Jens Petit */

#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <functional>

namespace collision_detection
{
namespace
{
static const std::string NAME = "Bullet";
const double MAX_DISTANCE_MARGIN = 99;
constexpr char LOGNAME[] = "collision_detection.bullet";
}  // namespace

CollisionEnvBullet::CollisionEnvBullet(const moveit::core::RobotModelConstPtr& model, double padding, double scale)
  : CollisionEnv(model, padding, scale)
{
  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(
      std::bind(&CollisionEnvBullet::notifyObjectChange, this, std::placeholders::_1, std::placeholders::_2));

  for (const std::pair<const std::string, urdf::LinkSharedPtr>& link : robot_model_->getURDF()->links_)
  {
    addLinkAsCollisionObject(link.second);
  }
}

CollisionEnvBullet::CollisionEnvBullet(const moveit::core::RobotModelConstPtr& model, const WorldPtr& world,
                                       double padding, double scale)
  : CollisionEnv(model, world, padding, scale)
{
  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(
      std::bind(&CollisionEnvBullet::notifyObjectChange, this, std::placeholders::_1, std::placeholders::_2));

  for (const std::pair<const std::string, urdf::LinkSharedPtr>& link : robot_model_->getURDF()->links_)
  {
    addLinkAsCollisionObject(link.second);
  }

  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

CollisionEnvBullet::CollisionEnvBullet(const CollisionEnvBullet& other, const WorldPtr& world)
  : CollisionEnv(other, world)
{
  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(
      std::bind(&CollisionEnvBullet::notifyObjectChange, this, std::placeholders::_1, std::placeholders::_2));

  for (const std::pair<const std::string, urdf::LinkSharedPtr>& link : other.robot_model_->getURDF()->links_)
  {
    addLinkAsCollisionObject(link.second);
  }

  // get notifications any objects already in the new world
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

CollisionEnvBullet::~CollisionEnvBullet()
{
  getWorld()->removeObserver(observer_handle_);
}

void CollisionEnvBullet::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                            const moveit::core::RobotState& state) const
{
  checkCollisionHelper(req, res, state, nullptr, true);
}

void CollisionEnvBullet::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                            const moveit::core::RobotState& state,
                                            const AllowedCollisionMatrix& acm) const
{
  checkCollisionHelper(req, res, state, &acm, true);
}

void CollisionEnvBullet::checkCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                              const moveit::core::RobotState& state, const AllowedCollisionMatrix* acm,
                                              bool self) const
{
  std::lock_guard<std::mutex> guard(collision_env_mutex_);

  std::vector<collision_detection_bullet::CollisionObjectWrapperPtr> cows;
  addAttachedOjects(state, cows);

  for (const collision_detection_bullet::CollisionObjectWrapperPtr& cow : cows)
  {
    manager_->addCollisionObject(cow);
  }

  // updating link positions with the current robot state
  updateTransformsFromState(state, manager_);

  ContactTestData cdata(safety_distance_, contact_distance_, negative_distance_, acm, &req, &res);
  cdata.enableGroup(getRobotModel());

  manager_->contactTest(cdata, self);

  if (req.distance && !res.collision)
  {
    double d = 0.0;
    if (self)
      d = approximate_contact_distance_ > 0.0 ? approximate_contact_distance_ : 0.05;
    else
      d = approximate_contact_distance_robot_ > 0.0 ? approximate_contact_distance_robot_ : 0.5;
    if (d <= contact_distance_)
      d = 2.0 * contact_distance_;
    while (res.contact_count == 0)
    {
      cdata.contact_distance = d;
      manager_->setContactDistanceThreshold(d);
      manager_->contactTest(cdata, self);
      d *= 2.0;
    }
    assert(!res.collision);
  }

  if (res.distance > 0.0 && res.contact_count > 0)
  {
    if (self)
      approximate_contact_distance_ = 0.5 * approximate_contact_distance_ + res.distance;
    else
      approximate_contact_distance_robot_ = 0.5 * approximate_contact_distance_robot_ + res.distance;
  }

  for (const collision_detection_bullet::CollisionObjectWrapperPtr& cow : cows)
  {
    manager_->removeCollisionObject(cow->getName());
  }

  if (req.distance)
  {
    manager_->setContactDistanceThreshold(contact_distance_);
  }
}

void CollisionEnvBullet::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                             const moveit::core::RobotState& state) const
{
  checkCollisionHelper(req, res, state, nullptr, false);
}

void CollisionEnvBullet::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                             const moveit::core::RobotState& state,
                                             const AllowedCollisionMatrix& acm) const
{
  checkCollisionHelper(req, res, state, &acm, false);
}

void CollisionEnvBullet::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                             const moveit::core::RobotState& state1,
                                             const moveit::core::RobotState& state2) const
{
  checkRobotCollisionHelperCCD(req, res, state1, state2, nullptr);
}

void CollisionEnvBullet::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                             const moveit::core::RobotState& state1,
                                             const moveit::core::RobotState& state2,
                                             const AllowedCollisionMatrix& acm) const
{
  checkRobotCollisionHelperCCD(req, res, state1, state2, &acm);
}

void CollisionEnvBullet::checkRobotCollisionHelperCCD(const CollisionRequest& req, CollisionResult& res,
                                                      const moveit::core::RobotState& state1,
                                                      const moveit::core::RobotState& state2,
                                                      const AllowedCollisionMatrix* acm) const
{
  std::lock_guard<std::mutex> guard(collision_env_mutex_);

  std::vector<collision_detection_bullet::CollisionObjectWrapperPtr> attached_cows;
  addAttachedOjects(state1, attached_cows);

  for (const collision_detection_bullet::CollisionObjectWrapperPtr& cow : attached_cows)
  {
    manager_CCD_->addCollisionObject(cow);
    manager_CCD_->setCastCollisionObjectsTransform(cow->getName(),
                                                   state1.getAttachedBody(cow->getName())->getGlobalPose(),
                                                   state2.getAttachedBody(cow->getName())->getGlobalPose());
  }

  for (const std::string& link : active_)
  {
    manager_CCD_->setCastCollisionObjectsTransform(link, state1.getGlobalLinkTransform(link),
                                                   state2.getGlobalLinkTransform(link));
  }

  ContactTestData cdata(safety_distance_, contact_distance_, negative_distance_, acm, &req, &res);
  cdata.enableGroup(getRobotModel());

  manager_CCD_->contactTest(cdata, false);

  if (req.distance && !res.collision)
  {
    double d = approximate_contact_distance_robot_ > 0.0 ? approximate_contact_distance_robot_ : 0.5;
    if (d <= contact_distance_)
      d = 2.0 * contact_distance_;
    while (res.contact_count == 0)
    {
      cdata.contact_distance = d;
      manager_CCD_->setContactDistanceThreshold(d);
      manager_CCD_->contactTest(cdata, false);
      d *= 2.0;
    }
    assert(!res.collision);
  }

  if (res.distance > 0.0 && res.contact_count > 0)
  {
    approximate_contact_distance_robot_ = 0.5 * approximate_contact_distance_robot_ + res.distance;
  }

  for (const collision_detection_bullet::CollisionObjectWrapperPtr& cow : attached_cows)
  {
    manager_CCD_->removeCollisionObject(cow->getName());
  }

  if (req.distance)
  {
    manager_CCD_->setContactDistanceThreshold(contact_distance_);
  }
}

void CollisionEnvBullet::distanceSelf(const DistanceRequest& req, DistanceResult& res,
                                      const moveit::core::RobotState& state) const
{
  CollisionRequest creq;
  CollisionResult cres;
  creq.group_name = req.group_name;
  creq.distance = true;
  creq.contacts = true;

  checkSelfCollision(creq, cres, state, *req.acm);

  std::vector<Contact> contacts;
  contacts.reserve(cres.contacts.size());
  for (const auto& mv : cres.contacts)
    std::copy(mv.second.begin(), mv.second.end(), std::back_inserter(contacts));

  std::size_t min_index = 0;
  for (std::size_t i = 1; i < contacts.size(); i++)
  {
    if (contacts[i].depth < contacts[min_index].depth)
      min_index = i;
  }

  res.clear();
  res.collision = cres.collision;
  res.minimum_distance.distance = contacts[min_index].depth;
  res.minimum_distance.nearest_points[0] = contacts[min_index].nearest_points[0];
  res.minimum_distance.nearest_points[1] = contacts[min_index].nearest_points[1];
  res.minimum_distance.nearest_points_local[0] = contacts[min_index].nearest_points_local[0];
  res.minimum_distance.nearest_points_local[1] = contacts[min_index].nearest_points_local[1];
  res.minimum_distance.nearest_points_local2[0] = contacts[min_index].nearest_points_local2[0];
  res.minimum_distance.nearest_points_local2[1] = contacts[min_index].nearest_points_local2[1];
  res.minimum_distance.body_types[0] = contacts[min_index].body_type_1;
  res.minimum_distance.body_types[1] = contacts[min_index].body_type_2;
  res.minimum_distance.link_names[0] = contacts[min_index].body_name_1;
  res.minimum_distance.link_names[1] = contacts[min_index].body_name_2;
  res.minimum_distance.normal = contacts[min_index].normal;
  res.minimum_distance.shape_id[0] = contacts[min_index].shape_id[0];
  res.minimum_distance.shape_id[1] = contacts[min_index].shape_id[1];
  res.minimum_distance.transform[0] = contacts[min_index].transform[0];
  res.minimum_distance.transform[1] = contacts[min_index].transform[1];
}

void CollisionEnvBullet::distanceRobot(const DistanceRequest& req, DistanceResult& res,
                                       const moveit::core::RobotState& state) const
{
  CollisionRequest creq;
  CollisionResult cres;
  creq.group_name = req.group_name;
  creq.distance = true;
  creq.contacts = true;

  checkRobotCollision(creq, cres, state, *req.acm);

  std::vector<Contact> contacts;
  contacts.reserve(cres.contacts.size());
  for (const auto& mv : cres.contacts)
    std::copy(mv.second.begin(), mv.second.end(), std::back_inserter(contacts));

  std::size_t min_index = 0;
  for (std::size_t i = 1; i < contacts.size(); i++)
  {
    if (contacts[i].depth < contacts[min_index].depth)
      min_index = i;
  }

  res.clear();
  res.collision = cres.collision;
  res.minimum_distance.distance = contacts[min_index].depth;
  res.minimum_distance.nearest_points[0] = contacts[min_index].nearest_points[0];
  res.minimum_distance.nearest_points[1] = contacts[min_index].nearest_points[1];
  res.minimum_distance.nearest_points_local[0] = contacts[min_index].nearest_points_local[0];
  res.minimum_distance.nearest_points_local[1] = contacts[min_index].nearest_points_local[1];
  res.minimum_distance.nearest_points_local2[0] = contacts[min_index].nearest_points_local2[0];
  res.minimum_distance.nearest_points_local2[1] = contacts[min_index].nearest_points_local2[1];
  res.minimum_distance.body_types[0] = contacts[min_index].body_type_1;
  res.minimum_distance.body_types[1] = contacts[min_index].body_type_2;
  res.minimum_distance.link_names[0] = contacts[min_index].body_name_1;
  res.minimum_distance.link_names[1] = contacts[min_index].body_name_2;
  res.minimum_distance.normal = contacts[min_index].normal;
  res.minimum_distance.shape_id[0] = contacts[min_index].shape_id[0];
  res.minimum_distance.shape_id[1] = contacts[min_index].shape_id[1];
  res.minimum_distance.transform[0] = contacts[min_index].transform[0];
  res.minimum_distance.transform[1] = contacts[min_index].transform[1];
}

void CollisionEnvBullet::setWorld(const WorldPtr& world)
{
  if (world == getWorld())
    return;

  getWorld()->notifyObserverAllObjects(observer_handle_, World::DESTROY);

  // turn off notifications about old world
  getWorld()->removeObserver(observer_handle_);

  CollisionEnv::setWorld(world);

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(
      std::bind(&CollisionEnvBullet::notifyObjectChange, this, std::placeholders::_1, std::placeholders::_2));

  // get notifications any objects already in the new world
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

void CollisionEnvBullet::notifyObjectChange(const ObjectConstPtr& obj, World::Action action)
{
  std::lock_guard<std::mutex> guard(collision_env_mutex_);
  if (action == World::DESTROY)
  {
    manager_->removeCollisionObject(obj->id_);
    manager_CCD_->removeCollisionObject(obj->id_);
  }
  else
  {
    updateManagedObject(obj->id_);
  }
}

void CollisionEnvBullet::updateManagedObject(const std::string& id)
{
  if (getWorld()->hasObject(id))
  {
    auto it = getWorld()->find(id);
    if (manager_->hasCollisionObject(id))
    {
      manager_->removeCollisionObject(id);
      manager_CCD_->removeCollisionObject(id);
    }

    addToManager(it->second.get());
  }
  else
  {
    if (manager_->hasCollisionObject(id))
    {
      manager_->removeCollisionObject(id);
      manager_CCD_->removeCollisionObject(id);
    }
  }
}

void CollisionEnvBullet::addToManager(const World::Object* obj)
{
  if (!obj->shapes_.empty())
  {
    std::vector<CollisionObjectType> collision_object_types;

    for (const shapes::ShapeConstPtr& shape : obj->shapes_)
    {
      if (shape->type == shapes::MESH)
        collision_object_types.push_back(CollisionObjectType::CONVEX_HULL);
      else
        collision_object_types.push_back(CollisionObjectType::USE_SHAPE_TYPE);
    }

    auto cow = collision_detection_bullet::createCollisionObject(obj->id_, collision_detection::BodyType::WORLD_OBJECT,
                                                                 obj->shapes_, obj->shape_poses_,
                                                                 collision_object_types, false);
    cow->setCollisionObjectsTransform(obj->pose_);
    cow->setContactProcessingThreshold(getContactDistanceThreshold());
    manager_->addCollisionObject(cow);
    manager_CCD_->addCollisionObject(cow->clone());
  }
}

void CollisionEnvBullet::addAttachedOjects(const moveit::core::RobotState& state,
                                           std::vector<collision_detection_bullet::CollisionObjectWrapperPtr>& cows) const
{
  std::vector<const moveit::core::AttachedBody*> attached_bodies;
  state.getAttachedBodies(attached_bodies);

  for (const moveit::core::AttachedBody*& body : attached_bodies)
  {
    const EigenSTL::vector_Isometry3d& attached_body_transform = body->getShapePoses();

    std::vector<CollisionObjectType> collision_object_types;
    collision_object_types.reserve(attached_body_transform.size());
    for (const shapes::ShapeConstPtr& shape : body->getShapes())
    {
      if (shape->type == shapes::MESH)
        collision_object_types.push_back(CollisionObjectType::CONVEX_HULL);
      else
        collision_object_types.push_back(CollisionObjectType::USE_SHAPE_TYPE);
    }

    try
    {
      auto cow = collision_detection_bullet::createCollisionObject(
          body->getName(), collision_detection::BodyType::ROBOT_ATTACHED, body->getShapes(), attached_body_transform,
          collision_object_types, body->getAttachedLinkName(), body->getTouchLinks());
      cow->setCollisionObjectsTransform(body->getGlobalPose());
      cow->setContactProcessingThreshold(getContactDistanceThreshold());
      cows.push_back(cow);
    }
    catch (std::exception&)
    {
      ROS_ERROR_STREAM_NAMED("collision_detetction.bullet",
                             "Not adding " << body->getName() << " due to bad arguments.");
    }
  }
}

void CollisionEnvBullet::updatedPaddingOrScaling(const std::vector<std::string>& links)
{
  for (const std::string& link : links)
  {
    if (robot_model_->getURDF()->links_.find(link) != robot_model_->getURDF()->links_.end())
    {
      addLinkAsCollisionObject(robot_model_->getURDF()->links_[link]);
    }
    else
    {
      ROS_ERROR_NAMED("collision_detection.bullet", "Updating padding or scaling for unknown link: '%s'", link.c_str());
    }
  }
}

void CollisionEnvBullet::updateTransformsFromState(
    const moveit::core::RobotState& state, const collision_detection_bullet::BulletDiscreteBVHManagerPtr& manager) const
{
  // updating link positions with the current robot state
  for (const std::string& link : active_)
  {
    // select the first of the transformations for each link (composed of multiple shapes...)
    manager->setCollisionObjectsTransform(link, state.getGlobalLinkTransform(link));
  }
}

void CollisionEnvBullet::addLinkAsCollisionObject(const urdf::LinkSharedPtr& link)
{
  if (!link->collision_array.empty() || link->collision)
  {
    const std::vector<urdf::CollisionSharedPtr>& col_array =
        link->collision_array.empty() ? std::vector<urdf::CollisionSharedPtr>(1, link->collision) :
                                        link->collision_array;

    std::vector<shapes::ShapeConstPtr> shapes;
    EigenSTL::vector_Isometry3d shape_poses;
    std::vector<CollisionObjectType> collision_object_types;

    for (const auto& i : col_array)
    {
      if (i && i->geometry)
      {
        shapes::ShapePtr shape = constructShape(i->geometry.get());

        if (shape)
        {
          if (fabs(getLinkScale(link->name) - 1.0) >= std::numeric_limits<double>::epsilon() ||
              fabs(getLinkPadding(link->name)) >= std::numeric_limits<double>::epsilon())
          {
            shape->scaleAndPadd(getLinkScale(link->name), getLinkPadding(link->name));
          }

          shapes.push_back(shape);
          shape_poses.push_back(urdfPose2Eigen(i->origin));

          if (shape->type == shapes::MESH)
          {
            collision_object_types.push_back(CollisionObjectType::CONVEX_HULL);
          }
          else
          {
            collision_object_types.push_back(CollisionObjectType::USE_SHAPE_TYPE);
          }
        }
      }
    }

    if (manager_->hasCollisionObject(link->name))
    {
      manager_->removeCollisionObject(link->name);
      manager_CCD_->removeCollisionObject(link->name);
      active_.erase(std::find(active_.begin(), active_.end(), link->name));
    }

    try
    {
      auto cow = collision_detection_bullet::createCollisionObject(
          link->name, collision_detection::BodyType::ROBOT_LINK, shapes, shape_poses, collision_object_types, true);
      cow->setContactProcessingThreshold(getContactDistanceThreshold());
      manager_->addCollisionObject(cow);
      manager_CCD_->addCollisionObject(cow->clone());
      active_.push_back(cow->getName());
    }
    catch (std::exception&)
    {
      ROS_ERROR_STREAM_NAMED("collision_detetction.bullet", "Not adding " << link->name << " due to bad arguments.");
    }
  }
}

const std::string CollisionEnvBullet::getCollisionName() const
{
  return NAME;
}

const BVHManagerConstPtr CollisionEnvBullet::getCollisionBVHManager() const
{
  return manager_;
}

const std::string& CollisionDetectorAllocatorBullet::getName() const
{
  return NAME;
}
}  // namespace collision_detection
