/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <utility>

namespace ompl_interface
{
constexpr char LOGNAME[] = "model_based_state_space";
}  // namespace ompl_interface

template <typename StateSpace>
ompl_interface::GroupBasedStateSpace<StateSpace>::GroupBasedStateSpace(GroupBasedStateSpaceSpecification spec)
  : StateSpace(), spec_(std::move(spec))
{
  // set the state space name
  StateSpace::setName(StateSpace::getName() + spec_.joint_model_group_->getName());

  std::size_t dim = 0;
  if (StateSpace::getType() == ompl::base::StateSpaceType::STATE_SPACE_REAL_VECTOR)
  {
    dim = spec_.joint_model_group_->getVariableCount();
    StateSpace::resize(dim);
  }
  else if (StateSpace::getType() == ompl::base::StateSpaceType::STATE_SPACE_SE2)
    dim = 2;
  else if (StateSpace::getType() == ompl::base::StateSpaceType::STATE_SPACE_SE3)
    dim = 3;
  else
  {
    ROS_ERROR_NAMED(LOGNAME, "Joint group '%s' state space type is unkonwn.",
                    spec_.joint_model_group_->getName().c_str());
    dim = spec_.joint_model_group_->getVariableCount();
  }

  ompl::base::RealVectorBounds rbounds(dim);

  std::size_t index = 0;
  for (const moveit::core::JointModel::Bounds* bounds : spec_.joint_bounds_)
  {
    for (const moveit::core::VariableBounds& bound : *bounds)
    {
      rbounds.setLow(index, bound.min_position_);
      rbounds.setHigh(index, bound.max_position_);
      index++;
      if (index == dim)
        break;
    }
    if (index == dim)
      break;
  }

  StateSpace::setBounds(rbounds);
}

template <typename StateSpace>
ompl_interface::GroupBasedStateSpace<StateSpace>::~GroupBasedStateSpace() = default;

template <typename StateSpace>
void ompl_interface::GroupBasedStateSpace<StateSpace>::copyToRobotState(moveit::core::RobotState& rstate,
                                                                        const ompl::base::State* state) const
{
  copyToRobotStateWithoutUpdate(rstate, state);
  rstate.update();
}

template <typename StateSpace>
void ompl_interface::GroupBasedStateSpace<StateSpace>::copyToRobotStateWithoutUpdate(
    moveit::core::RobotState& rstate, const ompl::base::State* state) const
{
  if (StateSpace::getType() == ompl::base::StateSpaceType::STATE_SPACE_REAL_VECTOR)
    rstate.setJointGroupPositions(spec_.joint_model_group_,
                                  state->as<ompl::base::RealVectorStateSpace::StateType>()->values);
  else
  {
    std::vector<double> reals;
    StateSpace::copyToReals(reals, state);
    rstate.setJointGroupPositions(spec_.joint_model_group_, reals);
  }
}

template <typename StateSpace>
void ompl_interface::GroupBasedStateSpace<StateSpace>::copyToOMPLState(ompl::base::State* state,
                                                                       const moveit::core::RobotState& rstate) const
{
  if (StateSpace::getType() == ompl::base::StateSpaceType::STATE_SPACE_REAL_VECTOR)
    rstate.copyJointGroupPositions(spec_.joint_model_group_,
                                   state->as<ompl::base::RealVectorStateSpace::StateType>()->values);
  else
  {
    std::vector<double> reals;
    rstate.copyJointGroupPositions(spec_.joint_model_group_, reals);
    StateSpace::copyFromReals(state, reals);
  }
}

// template<typename StateSpace>
// void ompl_interface::GroupBasedStateSpace<StateSpace>::copyJointToOMPLState(ompl::base::State* state,
//        const moveit::core::RobotState& robot_state,
//        const moveit::core::JointModel* joint_model,
//        int ompl_state_joint_index) const
//{
//    // Copy one joint (multiple variables possibly)
//    if (StateSpace::getType() == ompl::base::StateSpaceType::STATE_SPACE_REAL_VECTOR)
//        memcpy(getValueAddressAtIndex(state, ompl_state_joint_index),
//                robot_state.getVariablePositions() + joint_model->getFirstVariableIndex(),
//                joint_model->getVariableCount() * sizeof(double));
//    else if (StateSpace::getType() == ompl::base::StateSpaceType::STATE_SPACE_SE2)
//    {
//        memcpy(getValueAddressAtIndex(state, ompl_state_joint_index),
//                robot_state.getVariablePositions() + joint_model->getFirstVariableIndex(),
//                2 * sizeof(double));
//        memcpy(getValueAddressAtIndex(state, ompl_state_joint_index + 2),
//                robot_state.getVariablePositions() + joint_model->getFirstVariableIndex() + 2,
//                sizeof(double));
//    }
//    else if (StateSpace::getType() == ompl::base::StateSpaceType::STATE_SPACE_SE3)
//    {
//        memcpy(getValueAddressAtIndex(state, ompl_state_joint_index),
//                robot_state.getVariablePositions() + joint_model->getFirstVariableIndex(),
//                3 * sizeof(double));
//        memcpy(getValueAddressAtIndex(state, ompl_state_joint_index + 3),
//                robot_state.getVariablePositions() + joint_model->getFirstVariableIndex() + 3,
//                4 * sizeof(double));
//    }
//}

template <typename StateSpace>
void ompl_interface::GroupBasedStateSpace<StateSpace>::setPlanningVolume(double minX, double maxX, double minY,
                                                                         double maxY, double minZ, double maxZ)
{
  if (StateSpace::getType() == ompl::base::StateSpaceType::STATE_SPACE_SE2)
  {
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0, minX);
    bounds.setHigh(0, maxX);
    bounds.setLow(1, minY);
    bounds.setHigh(1, maxY);
    StateSpace::setBounds(bounds);
  }
  else if (StateSpace::getType() == ompl::base::StateSpaceType::STATE_SPACE_SE3)
  {
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, minX);
    bounds.setHigh(0, maxX);
    bounds.setLow(1, minY);
    bounds.setHigh(1, maxY);
    bounds.setLow(2, minZ);
    bounds.setHigh(2, maxZ);
    StateSpace::setBounds(bounds);
  }
}

template <typename StateSpace>
void ompl_interface::GroupBasedStateSpace<StateSpace>::printState(const ompl::base::State* state,
                                                                  std::ostream& out) const
{
  std::vector<const moveit::core::JointModel*> joint_model_vector = spec_.joint_model_group_->getActiveJointModels();

  if (StateSpace::getType() == ompl::base::StateSpaceType::STATE_SPACE_REAL_VECTOR)
  {
    for (const moveit::core::JointModel* j : joint_model_vector)
    {
      out << j->getName() << " = ";
      const int idx = spec_.joint_model_group_->getVariableGroupIndex(j->getName());
      const int vc = j->getVariableCount();
      for (int i = 0; i < vc; ++i)
        out << state->as<ompl::base::RealVectorStateSpace::StateType>()->values[idx + i] << " ";
      out << std::endl;
    }
  }
  else
  {
    assert(joint_model_vector.size() == 1u);
    out << joint_model_vector[0]->getName() << " = " << state << std::endl;
  }
}

ompl_interface::ModelBasedStateSpace::ModelBasedStateSpace(ModelBasedStateSpaceSpecification spec)
  : ompl::base::CompoundStateSpace(), spec_(std::move(spec))
{
  // set the state space name
  setName(getName() + spec_.joint_model_group_->getName());
  state_values_size_ = spec_.joint_model_group_->getVariableCount() * sizeof(double);
  joint_model_vector_ = spec_.joint_model_group_->getActiveJointModels();

  // make sure we have bounds for every joint stored within the spec (use default bounds if not specified)
  if (!spec_.joint_bounds_.empty() && spec_.joint_bounds_.size() != joint_model_vector_.size())
  {
    ROS_ERROR_NAMED(LOGNAME, "Joint group '%s' has incorrect bounds specified. Using the default bounds instead.",
                    spec_.joint_model_group_->getName().c_str());
    spec_.joint_bounds_.clear();
  }

  // copy the default joint bounds if needed
  if (spec_.joint_bounds_.empty())
    spec_.joint_bounds_ = spec_.joint_model_group_->getActiveJointModelsBounds();

  // new perform a deep copy of the bounds, in case we need to modify them
  joint_bounds_storage_.resize(spec_.joint_bounds_.size());
  for (std::size_t i = 0; i < joint_bounds_storage_.size(); ++i)
  {
    joint_bounds_storage_[i] = *spec_.joint_bounds_[i];
    spec_.joint_bounds_[i] = &joint_bounds_storage_[i];
  }

  std::vector<const moveit::core::JointModelGroup*> sub_groups;
  spec_.joint_model_group_->getSubgroups(sub_groups);

  if (sub_groups.empty())
  {
    GroupBasedStateSpaceSpecification sub_spec(spec_.joint_model_group_);
    sub_spec.joint_bounds_ = spec_.joint_bounds_;
    addSubspace(std::make_shared<GroupBasedStateSpace<ompl::base::RealVectorStateSpace>>(sub_spec), 1.0);
  }
  else
  {
    std::size_t sub_joint_count = 0;
    const std::vector<std::string>& active_joint_names = spec_.joint_model_group_->getActiveJointModelNames();
    for (const moveit::core::JointModelGroup* group : sub_groups)
      sub_joint_count += group->getActiveJointModelNames().size();
    if (sub_joint_count != active_joint_names.size())
    {
      ROS_ERROR_NAMED(LOGNAME, "Joint group '%s' contains incorrect sub groups.",
                      spec_.joint_model_group_->getName().c_str());
      throw moveit::Exception("Unable to intialize the model based state space");
    }

    for (const moveit::core::JointModelGroup* group : sub_groups)
    {
      GroupBasedStateSpaceSpecification sub_spec(group);
      const std::vector<std::string>& sub_joint_names = group->getActiveJointModelNames();
      for (const std::string& name : sub_joint_names)
      {
        auto it = std::find(active_joint_names.begin(), active_joint_names.end(), name);
        if (it == active_joint_names.end())
        {
          ROS_ERROR_NAMED(LOGNAME, "Joint group '%s' contains incorrect joint model '%s'.", group->getName().c_str(),
                          name.c_str());
          throw moveit::Exception("Unable to intialize the model based state space");
        }
        sub_spec.joint_bounds_.push_back(spec_.joint_bounds_[it - active_joint_names.begin()]);
      }

      if (sub_joint_names.size() > 1)
        addSubspace(std::make_shared<GroupBasedStateSpace<ompl::base::RealVectorStateSpace>>(sub_spec), 1.0);
      else
      {
        moveit::core::JointModel::JointType type = group->getJointModel(sub_joint_names[0])->getType();
        if (type == moveit::core::JointModel::PLANAR)
          addSubspace(std::make_shared<GroupBasedStateSpace<ompl::base::SE2StateSpace>>(sub_spec), 1.0);
        else if (type == moveit::core::JointModel::FLOATING)
          addSubspace(std::make_shared<GroupBasedStateSpace<ompl::base::SE3StateSpace>>(sub_spec), 1.0);
        else
          addSubspace(std::make_shared<GroupBasedStateSpace<ompl::base::RealVectorStateSpace>>(sub_spec), 1.0);
      }
    }
  }

  // default settings
  setTagSnapToSegment(0.95);

  /// expose parameters
  params_.declareParam<double>("tag_snap_to_segment",
                               std::bind(&ModelBasedStateSpace::setTagSnapToSegment, this, std::placeholders::_1),
                               std::bind(&ModelBasedStateSpace::getTagSnapToSegment, this));
}

ompl_interface::ModelBasedStateSpace::~ModelBasedStateSpace() = default;

double ompl_interface::ModelBasedStateSpace::getTagSnapToSegment() const
{
  return tag_snap_to_segment_;
}

void ompl_interface::ModelBasedStateSpace::setTagSnapToSegment(double snap)
{
  if (snap < 0.0 || snap > 1.0)
    ROS_WARN_NAMED(LOGNAME,
                   "Snap to segment for tags is a ratio. It's value must be between 0.0 and 1.0. "
                   "Value remains as previously set (%lf)",
                   tag_snap_to_segment_);
  else
  {
    tag_snap_to_segment_ = snap;
    tag_snap_to_segment_complement_ = 1.0 - tag_snap_to_segment_;
  }
}

void ompl_interface::ModelBasedStateSpace::copyState(ompl::base::State* destination,
                                                     const ompl::base::State* source) const
{
  ompl::base::CompoundStateSpace::copyState(destination, source);
  destination->as<StateType>()->tag = source->as<StateType>()->tag;
  destination->as<StateType>()->flags = source->as<StateType>()->flags;
  destination->as<StateType>()->distance = source->as<StateType>()->distance;
}

unsigned int ompl_interface::ModelBasedStateSpace::getSerializationLength() const
{
  return state_values_size_ + sizeof(int);
}

void ompl_interface::ModelBasedStateSpace::serialize(void* serialization, const ompl::base::State* state) const
{
  *reinterpret_cast<int*>(serialization) = state->as<StateType>()->tag;
  ompl::base::CompoundStateSpace::serialize(reinterpret_cast<char*>(serialization) + sizeof(int), state);
}

void ompl_interface::ModelBasedStateSpace::deserialize(ompl::base::State* state, const void* serialization) const
{
  state->as<StateType>()->tag = *reinterpret_cast<const int*>(serialization);
  ompl::base::CompoundStateSpace::deserialize(state, reinterpret_cast<const char*>(serialization) + sizeof(int));
}

double ompl_interface::ModelBasedStateSpace::distance(const ompl::base::State* state1,
                                                      const ompl::base::State* state2) const
{
  if (distance_function_)
    return distance_function_(state1, state2);
  else
    return ompl::base::CompoundStateSpace::distance(state1, state2);
}

void ompl_interface::ModelBasedStateSpace::interpolate(const ompl::base::State* from, const ompl::base::State* to,
                                                       const double t, ompl::base::State* state) const
{
  // clear any cached info (such as validity known or not)
  state->as<StateType>()->clearKnownInformation();

  if (!interpolation_function_ || !interpolation_function_(from, to, t, state))
  {
    // perform the actual interpolation
    ompl::base::CompoundStateSpace::interpolate(from, to, t, state);

    // compute tag
    if (from->as<StateType>()->tag >= 0 && t < 1.0 - tag_snap_to_segment_)
      state->as<StateType>()->tag = from->as<StateType>()->tag;
    else if (to->as<StateType>()->tag >= 0 && t > tag_snap_to_segment_)
      state->as<StateType>()->tag = to->as<StateType>()->tag;
    else
      state->as<StateType>()->tag = -1;
  }
}

void ompl_interface::ModelBasedStateSpace::setPlanningVolume(double minX, double maxX, double minY, double maxY,
                                                             double minZ, double maxZ)
{
  for (std::size_t i = 0; i < joint_model_vector_.size(); ++i)
  {
    if (joint_model_vector_[i]->getType() == moveit::core::JointModel::PLANAR)
    {
      joint_bounds_storage_[i][0].min_position_ = minX;
      joint_bounds_storage_[i][0].max_position_ = maxX;
      joint_bounds_storage_[i][1].min_position_ = minY;
      joint_bounds_storage_[i][1].max_position_ = maxY;
    }
    else if (joint_model_vector_[i]->getType() == moveit::core::JointModel::FLOATING)
    {
      joint_bounds_storage_[i][0].min_position_ = minX;
      joint_bounds_storage_[i][0].max_position_ = maxX;
      joint_bounds_storage_[i][1].min_position_ = minY;
      joint_bounds_storage_[i][1].max_position_ = maxY;
      joint_bounds_storage_[i][2].min_position_ = minZ;
      joint_bounds_storage_[i][2].max_position_ = maxZ;
    }
  }

  const std::vector<ompl::base::StateSpacePtr>& subspaces = getSubspaces();
  for (const ompl::base::StateSpacePtr& space : subspaces)
  {
    if (space->getType() == ompl::base::StateSpaceType::STATE_SPACE_SE2)
    {
      ompl::base::RealVectorBounds bounds(2);
      bounds.setLow(0, minX);
      bounds.setHigh(0, maxX);
      bounds.setLow(1, minY);
      bounds.setHigh(1, maxY);
      space->as<ompl::base::SE2StateSpace>()->setBounds(bounds);
    }
    else if (space->getType() == ompl::base::StateSpaceType::STATE_SPACE_SE3)
    {
      ompl::base::RealVectorBounds bounds(3);
      bounds.setLow(0, minX);
      bounds.setHigh(0, maxX);
      bounds.setLow(1, minY);
      bounds.setHigh(1, maxY);
      bounds.setLow(2, minZ);
      bounds.setHigh(2, maxZ);
      space->as<ompl::base::SE3StateSpace>()->setBounds(bounds);
    }
  }
}

ompl::base::StateSamplerPtr ompl_interface::ModelBasedStateSpace::allocDefaultStateSampler() const
{
  class DefaultStateSampler : public ompl::base::CompoundStateSampler
  {
  public:
    DefaultStateSampler(const ompl::base::StateSpace* space) : ompl::base::CompoundStateSampler(space)
    {
    }

    void sampleUniform(ompl::base::State* state) override
    {
      ompl::base::CompoundStateSampler::sampleUniform(state);
      state->as<StateType>()->clearKnownInformation();
    }

    void sampleUniformNear(ompl::base::State* state, const ompl::base::State* near, const double distance) override
    {
      ompl::base::CompoundStateSampler::sampleUniformNear(state, near, distance);
      state->as<StateType>()->clearKnownInformation();
    }

    void sampleGaussian(ompl::base::State* state, const ompl::base::State* mean, const double stdDev) override
    {
      ompl::base::CompoundStateSampler::sampleGaussian(state, mean, stdDev);
      state->as<StateType>()->clearKnownInformation();
    }
  };

  auto ss(std::make_shared<DefaultStateSampler>(this));
  if (weightSum_ < std::numeric_limits<double>::epsilon())
    for (unsigned int i = 0; i < componentCount_; ++i)
      ss->addSampler(components_[i]->allocStateSampler(), 1.0);
  else
    for (unsigned int i = 0; i < componentCount_; ++i)
      ss->addSampler(components_[i]->allocStateSampler(), weights_[i] / weightSum_);
  return ss;
}

void ompl_interface::ModelBasedStateSpace::printSettings(std::ostream& out) const
{
  out << "ModelBasedStateSpace '" << getName() << "' of dimension " << getDimension() << (isLocked() ? " (locked)" : "")
      << " [" << std::endl;
  for (unsigned int i = 0; i < componentCount_; ++i)
  {
    components_[i]->printSettings(out);
    out << " of weight " << weights_[i] << std::endl;
  }
  out << "]" << std::endl;
}

void ompl_interface::ModelBasedStateSpace::printState(const ompl::base::State* state, std::ostream& out) const
{
  ompl::base::CompoundStateSpace::printState(state, out);

  if (state->as<StateType>()->isStartState())
    out << "* start state" << std::endl;
  if (state->as<StateType>()->isGoalState())
    out << "* goal state" << std::endl;
  if (state->as<StateType>()->isValidityKnown())
  {
    if (state->as<StateType>()->isMarkedValid())
      out << "* valid state" << std::endl;
    else
      out << "* invalid state" << std::endl;
  }
  out << "Tag: " << state->as<StateType>()->tag << std::endl;
}

void ompl_interface::ModelBasedStateSpace::copyToRobotState(moveit::core::RobotState& rstate,
                                                            const ompl::base::State* state) const
{
  copyToRobotStateWithoutUpdate(rstate, state);
  rstate.update();
}

void ompl_interface::ModelBasedStateSpace::copyToRobotStateWithoutUpdate(moveit::core::RobotState& rstate,
                                                                         const ompl::base::State* state) const
{
  const ompl::base::CompoundState* cstate = static_cast<const ompl::base::CompoundState*>(state);
  for (unsigned int i = 0; i < componentCount_; ++i)
  {
    if (components_[i]->getType() == ompl::base::StateSpaceType::STATE_SPACE_REAL_VECTOR)
      components_[i]->as<GroupBasedStateSpace<ompl::base::RealVectorStateSpace>>()->copyToRobotStateWithoutUpdate(
          rstate, cstate->components[i]);
    else if (components_[i]->getType() == ompl::base::StateSpaceType::STATE_SPACE_SE2)
      components_[i]->as<GroupBasedStateSpace<ompl::base::SE2StateSpace>>()->copyToRobotStateWithoutUpdate(
          rstate, cstate->components[i]);
    else if (components_[i]->getType() == ompl::base::StateSpaceType::STATE_SPACE_SE3)
      components_[i]->as<GroupBasedStateSpace<ompl::base::SE3StateSpace>>()->copyToRobotStateWithoutUpdate(
          rstate, cstate->components[i]);
  }
}

void ompl_interface::ModelBasedStateSpace::copyToOMPLState(ompl::base::State* state,
                                                           const moveit::core::RobotState& rstate) const
{
  ompl::base::CompoundState* cstate = static_cast<ompl::base::CompoundState*>(state);
  for (unsigned int i = 0; i < componentCount_; ++i)
  {
    if (components_[i]->getType() == ompl::base::StateSpaceType::STATE_SPACE_REAL_VECTOR)
      components_[i]->as<GroupBasedStateSpace<ompl::base::RealVectorStateSpace>>()->copyToOMPLState(
          cstate->components[i], rstate);
    else if (components_[i]->getType() == ompl::base::StateSpaceType::STATE_SPACE_SE2)
      components_[i]->as<GroupBasedStateSpace<ompl::base::SE2StateSpace>>()->copyToOMPLState(cstate->components[i],
                                                                                             rstate);
    else if (components_[i]->getType() == ompl::base::StateSpaceType::STATE_SPACE_SE3)
      components_[i]->as<GroupBasedStateSpace<ompl::base::SE3StateSpace>>()->copyToOMPLState(cstate->components[i],
                                                                                             rstate);
  }

  state->as<StateType>()->clearKnownInformation();
}

// void ompl_interface::ModelBasedStateSpace::copyJointToOMPLState(ompl::base::State* state,
//        const moveit::core::RobotState& robot_state,
//        const moveit::core::JointModel* joint_model,
//        int ompl_state_joint_index) const
//{
//    // Copy one joint (multiple variables possibly)
//    memcpy(getValueAddressAtIndex(state, ompl_state_joint_index),
//            robot_state.getVariablePositions() + joint_model->getFirstVariableIndex(),
//            joint_model->getVariableCount() * sizeof(double));
//
//    // clear any cached info (such as validity known or not)
//    state->as<StateType>()->clearKnownInformation();
//}
