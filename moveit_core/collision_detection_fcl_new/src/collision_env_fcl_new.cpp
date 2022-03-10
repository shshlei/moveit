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

/* Author: Shi Shenglei */

#include <moveit/collision_detection_fcl_new/collision_env_fcl_new.h>
#include <moveit/collision_detection_fcl_new/collision_detector_allocator_fcl_new.h>
#include <moveit/collision_detection/collision_utils.h>
#include <functional>

namespace collision_detection
{
namespace
{
static const std::string NAME = "FCL";
constexpr char LOGNAME[] = "collision_detection.fcl";
}  // namespace

CollisionEnvFCLNew::CollisionEnvFCLNew(const moveit::core::RobotModelConstPtr& model, double padding, double scale)
  : CollisionEnv(model, padding, scale)
{
  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(
      std::bind(&CollisionEnvFCLNew::notifyObjectChange, this, std::placeholders::_1, std::placeholders::_2));

  std::vector<urdf::LinkSharedPtr> links;
  robot_model_->getURDF()->getLinks(links);
  addLinksAsCollisionObjects(links);
}

CollisionEnvFCLNew::CollisionEnvFCLNew(const moveit::core::RobotModelConstPtr& model, const WorldPtr& world,
                                       double padding, double scale)
  : CollisionEnv(model, world, padding, scale)
{
  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(
      std::bind(&CollisionEnvFCLNew::notifyObjectChange, this, std::placeholders::_1, std::placeholders::_2));

  std::vector<urdf::LinkSharedPtr> links;
  robot_model_->getURDF()->getLinks(links);
  addLinksAsCollisionObjects(links);

  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

CollisionEnvFCLNew::CollisionEnvFCLNew(const CollisionEnvFCLNew& other, const WorldPtr& world)
  : CollisionEnv(other, world)
{
  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(
      std::bind(&CollisionEnvFCLNew::notifyObjectChange, this, std::placeholders::_1, std::placeholders::_2));

  std::vector<urdf::LinkSharedPtr> links;
  robot_model_->getURDF()->getLinks(links);
  addLinksAsCollisionObjects(links);

  // get notifications any objects already in the new world
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

CollisionEnvFCLNew::~CollisionEnvFCLNew()
{
  getWorld()->removeObserver(observer_handle_);
}

void CollisionEnvFCLNew::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                            const moveit::core::RobotState& state) const
{
  checkSelfCollisionHelper(req, res, state, nullptr);
}

void CollisionEnvFCLNew::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                            const moveit::core::RobotState& state,
                                            const AllowedCollisionMatrix& acm) const
{
  checkSelfCollisionHelper(req, res, state, &acm);
}

void CollisionEnvFCLNew::checkSelfCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                  const moveit::core::RobotState& state,
                                                  const AllowedCollisionMatrix* acm) const
{
  std::lock_guard<std::mutex> guard(collision_env_mutex_);

  std::vector<collision_detection_fcl::CollisionObjectWrapperPtr> cows;
  addAttachedOjects(state, cows);
  manager_->addCollisionObjects(cows);

  // updating link positions with the current robot state
  updateTransformsFromState(state, manager_);

  ContactTestData cdata(safety_distance_, contact_distance_, negative_distance_, acm, &req, &res);
  cdata.enableGroup(getRobotModel());

  manager_->contactTest(cdata, true);

  if (req.distance && !res.collision)
  {
    DistanceRequest dreq;
    DistanceResult dres;

    dreq.group_name = req.group_name;
    dreq.acm = acm;
    dreq.enableGroup(getRobotModel());

    DistanceTestData dcdata(safety_distance_, contact_distance_, negative_distance_, &dreq, &dres);

    manager_->distanceTest(dcdata, true);

    res.distance = dres.minimum_distance.distance;
  }

  for (const collision_detection_fcl::CollisionObjectWrapperPtr& cow : cows)
  {
    manager_->removeCollisionObject(cow->getName());
  }
}

void CollisionEnvFCLNew::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                             const moveit::core::RobotState& state) const
{
  checkRobotCollisionHelper(req, res, state, nullptr);
}

void CollisionEnvFCLNew::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                             const moveit::core::RobotState& state,
                                             const AllowedCollisionMatrix& acm) const
{
  checkRobotCollisionHelper(req, res, state, &acm);
}

void CollisionEnvFCLNew::checkRobotCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                   const moveit::core::RobotState& state,
                                                   const AllowedCollisionMatrix* acm) const
{
  std::lock_guard<std::mutex> guard(collision_env_mutex_);

  std::vector<collision_detection_fcl::CollisionObjectWrapperPtr> cows;
  addAttachedOjects(state, cows);
  manager_->addCollisionObjects(cows);

  // updating link positions with the current robot state
  updateTransformsFromState(state, manager_);

  ContactTestData cdata(safety_distance_, contact_distance_, negative_distance_, acm, &req, &res);
  cdata.enableGroup(getRobotModel());

  manager_->contactTest(cdata, false);

  if ((req.distance || safety_distance_ > 0.0) && !res.collision)
  {
    DistanceRequest dreq;
    DistanceResult dres;

    dreq.group_name = req.group_name;
    dreq.acm = acm;
    dreq.enableGroup(getRobotModel());

    if (!req.distance)
    {
      dreq.distance_threshold = safety_distance_;
      dres.minimum_distance.distance = safety_distance_;
    }

    DistanceTestData dcdata(safety_distance_, contact_distance_, negative_distance_, &dreq, &dres);

    manager_->distanceTest(dcdata, false);

    if (req.distance)
      res.distance = dres.minimum_distance.distance;
    res.collision = dres.collision;
  }

  for (const collision_detection_fcl::CollisionObjectWrapperPtr& cow : cows)
  {
    manager_->removeCollisionObject(cow->getName());
  }
}

void CollisionEnvFCLNew::checkRobotCollision(const CollisionRequest& /*req*/, CollisionResult& /*res*/,
                                             const moveit::core::RobotState& /*state1*/,
                                             const moveit::core::RobotState& /*state2*/) const
{
  ROS_ERROR_NAMED(LOGNAME, "Continuous collision not implemented");
}

void CollisionEnvFCLNew::checkRobotCollision(const CollisionRequest& /*req*/, CollisionResult& /*res*/,
                                             const moveit::core::RobotState& /*state1*/,
                                             const moveit::core::RobotState& /*state2*/,
                                             const AllowedCollisionMatrix& /*acm*/) const
{
  ROS_ERROR_NAMED(LOGNAME, "Continuous collision not implemented");
}

void CollisionEnvFCLNew::distanceSelf(const DistanceRequest& req, DistanceResult& res,
                                      const moveit::core::RobotState& state) const
{
  std::lock_guard<std::mutex> guard(collision_env_mutex_);

  std::vector<collision_detection_fcl::CollisionObjectWrapperPtr> cows;
  addAttachedOjects(state, cows);
  manager_->addCollisionObjects(cows);

  // updating link positions with the current robot state
  updateTransformsFromState(state, manager_);

  DistanceTestData cdata(safety_distance_, contact_distance_, negative_distance_, &req, &res);

  manager_->distanceTest(cdata, true);

  for (const collision_detection_fcl::CollisionObjectWrapperPtr& cow : cows)
  {
    manager_->removeCollisionObject(cow->getName());
  }
}

void CollisionEnvFCLNew::distanceRobot(const DistanceRequest& req, DistanceResult& res,
                                       const moveit::core::RobotState& state) const
{
  std::lock_guard<std::mutex> guard(collision_env_mutex_);

  std::vector<collision_detection_fcl::CollisionObjectWrapperPtr> cows;
  addAttachedOjects(state, cows);
  manager_->addCollisionObjects(cows);

  // updating link positions with the current robot state
  updateTransformsFromState(state, manager_);

  DistanceTestData cdata(safety_distance_, contact_distance_, negative_distance_, &req, &res);

  manager_->distanceTest(cdata, false);

  for (const collision_detection_fcl::CollisionObjectWrapperPtr& cow : cows)
  {
    manager_->removeCollisionObject(cow->getName());
  }
}

void CollisionEnvFCLNew::setWorld(const WorldPtr& world)
{
  if (world == getWorld())
    return;

  getWorld()->notifyObserverAllObjects(observer_handle_, World::DESTROY);

  // turn off notifications about old world
  getWorld()->removeObserver(observer_handle_);

  CollisionEnv::setWorld(world);

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(
      std::bind(&CollisionEnvFCLNew::notifyObjectChange, this, std::placeholders::_1, std::placeholders::_2));

  // get notifications any objects already in the new world
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

void CollisionEnvFCLNew::notifyObjectChange(const ObjectConstPtr& obj, World::Action action)
{
  std::lock_guard<std::mutex> guard(collision_env_mutex_);
  if (action == World::DESTROY)
  {
    manager_->removeCollisionObject(obj->id_);
  }
  else
  {
    updateManagedObject(obj->id_);
  }
}

void CollisionEnvFCLNew::updateManagedObject(const std::string& id)
{
  if (getWorld()->hasObject(id))
  {
    auto it = getWorld()->find(id);
    if (manager_->hasCollisionObject(id))
    {
      manager_->removeCollisionObject(id);
    }

    addToManager(it->second.get());
  }
  else
  {
    if (manager_->hasCollisionObject(id))
    {
      manager_->removeCollisionObject(id);
    }
  }
}

void CollisionEnvFCLNew::addToManager(const World::Object* obj)
{
  if (!obj->shapes_.empty())
  {
    std::vector<CollisionObjectType> collision_object_types(obj->shapes_.size(), CollisionObjectType::USE_SHAPE_TYPE);
    //            std::fill(collision_object_types.begin(), collision_object_types.end(),
    //            CollisionObjectType::USE_SHAPE_TYPE); for (const shapes::ShapeConstPtr& shape : obj->shapes_)
    //            {
    //                collision_object_types.push_back(CollisionObjectType::USE_SHAPE_TYPE);
    //            }

    auto cow =
        collision_detection_fcl::createCollisionObject(obj->id_, collision_detection::BodyType::WORLD_OBJECT,
                                                       obj->shapes_, obj->shape_poses_, collision_object_types, false);
    cow->setCollisionObjectsTransform(obj->pose_);
    cow->setContactDistanceThreshold(getContactDistanceThreshold());
    manager_->addCollisionObject(cow);
  }
}

void CollisionEnvFCLNew::addAttachedOjects(const moveit::core::RobotState& state,
                                           std::vector<collision_detection_fcl::CollisionObjectWrapperPtr>& cows) const
{
  std::vector<const moveit::core::AttachedBody*> attached_bodies;
  state.getAttachedBodies(attached_bodies);

  for (const moveit::core::AttachedBody*& body : attached_bodies)
  {
    const EigenSTL::vector_Isometry3d& attached_body_transform = body->getShapePoses();
    std::vector<CollisionObjectType> collision_object_types(attached_body_transform.size(),
                                                            CollisionObjectType::USE_SHAPE_TYPE);
    try
    {
      auto cow = collision_detection_fcl::createCollisionObject(
          body->getName(), collision_detection::BodyType::ROBOT_ATTACHED, body->getShapes(), attached_body_transform,
          collision_object_types, body->getAttachedLinkName(), body->getTouchLinks());
      cow->setCollisionObjectsTransform(body->getGlobalPose());
      cow->setContactDistanceThreshold(getContactDistanceThreshold());
      cows.push_back(cow);
    }
    catch (std::exception&)
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Not adding " << body->getName() << " due to bad arguments.");
    }
  }
}

void CollisionEnvFCLNew::updatedPaddingOrScaling(const std::vector<std::string>& links)
{
  for (const std::string& link : links)
  {
    if (robot_model_->getURDF()->links_.find(link) != robot_model_->getURDF()->links_.end())
    {
      addLinkAsCollisionObject(robot_model_->getURDF()->links_[link]);
    }
    else
    {
      ROS_ERROR_NAMED(LOGNAME, "Updating padding or scaling for unknown link: '%s'", link.c_str());
    }
  }
}

void CollisionEnvFCLNew::updateTransformsFromState(
    const moveit::core::RobotState& state, const collision_detection_fcl::FclDiscreteBVHManagerPtr& manager) const
{
  EigenSTL::vector_Isometry3d poses;
  poses.reserve(active_.size());

  // updating link positions with the current robot state
  for (const std::string& link : active_)
  {
    poses.push_back(state.getGlobalLinkTransform(link));
    // select the first of the transformations for each link (composed of multiple shapes...)
  }

  manager->setCollisionObjectsTransforms(active_, poses);
}

void CollisionEnvFCLNew::addLinkAsCollisionObject(const urdf::LinkSharedPtr& link)
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
          collision_object_types.push_back(CollisionObjectType::USE_SHAPE_TYPE);
        }
      }
    }

    if (manager_->hasCollisionObject(link->name))
    {
      manager_->removeCollisionObject(link->name);
      active_.erase(std::find(active_.begin(), active_.end(), link->name));
    }

    try
    {
      auto cow = collision_detection_fcl::createCollisionObject(link->name, collision_detection::BodyType::ROBOT_LINK,
                                                                shapes, shape_poses, collision_object_types, true);
      cow->setContactDistanceThreshold(getContactDistanceThreshold());
      manager_->addCollisionObject(cow);
      active_.push_back(cow->getName());
    }
    catch (std::exception&)
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Not adding " << link->name << " due to bad arguments.");
    }
  }
}

void CollisionEnvFCLNew::addLinksAsCollisionObjects(const std::vector<urdf::LinkSharedPtr>& links)
{
  std::vector<collision_detection_fcl::CollisionObjectWrapperPtr> cows;

  for (auto& link : links)
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
            collision_object_types.push_back(CollisionObjectType::USE_SHAPE_TYPE);
          }
        }
      }

      if (manager_->hasCollisionObject(link->name))
      {
        manager_->removeCollisionObject(link->name);
        active_.erase(std::find(active_.begin(), active_.end(), link->name));
      }

      try
      {
        auto cow = collision_detection_fcl::createCollisionObject(link->name, collision_detection::BodyType::ROBOT_LINK,
                                                                  shapes, shape_poses, collision_object_types, true);
        cow->setContactDistanceThreshold(getContactDistanceThreshold());
        cows.push_back(cow);
        active_.push_back(cow->getName());
      }
      catch (std::exception&)
      {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Not adding " << link->name << " due to bad arguments.");
      }
    }
  }

  manager_->addCollisionObjects(cows);
}

const std::string CollisionEnvFCLNew::getCollisionName() const
{
  return NAME;
}

const BVHManagerConstPtr CollisionEnvFCLNew::getCollisionBVHManager() const
{
  return manager_;
}

const std::string& CollisionDetectorAllocatorFCLNew::getName() const
{
  return NAME;
}
}  // namespace collision_detection
