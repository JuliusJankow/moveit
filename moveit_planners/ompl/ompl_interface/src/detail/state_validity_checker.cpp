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

/* Author: Ioan Sucan */

#include <moveit/collision_detection/collision_env.h>
#include <moveit/ompl_interface/detail/state_validity_checker.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/profiler/profiler.h>
#include <ros/ros.h>

ompl_interface::StateValidityChecker::StateValidityChecker(const ModelBasedPlanningContext* pc)
  : ompl::base::StateValidityChecker(pc->getOMPLSimpleSetup()->getSpaceInformation())
  , planning_context_(pc)
  , group_name_(pc->getGroupName())
  , tss_(pc->getCompleteInitialRobotState())
  , verbose_(false)
{
  specs_.clearanceComputationType = ompl::base::StateValidityCheckerSpecs::APPROXIMATE;
  specs_.hasValidDirectionComputation = false;

  collision_request_with_distance_.distance = true;
  collision_request_with_cost_.cost = true;

  collision_request_simple_.group_name = planning_context_->getGroupName();
  collision_request_with_distance_.group_name = planning_context_->getGroupName();
  collision_request_with_cost_.group_name = planning_context_->getGroupName();

  collision_request_simple_verbose_ = collision_request_simple_;
  collision_request_simple_verbose_.verbose = true;

  collision_request_with_distance_verbose_ = collision_request_with_distance_;
  collision_request_with_distance_verbose_.verbose = true;
}

void ompl_interface::StateValidityChecker::setVerbose(bool flag)
{
  verbose_ = flag;
}

bool ompl_interface::StateValidityChecker::isValid(const ompl::base::State* state, bool verbose) const
{
  //  moveit::Profiler::ScopedBlock sblock("isValid");
  return planning_context_->useStateValidityCache() ? isValidWithCache(state, verbose) :
                                                      isValidWithoutCache(state, verbose);
}

bool ompl_interface::StateValidityChecker::isValid(const ompl::base::State* state, double& dist, bool verbose) const
{
  //  moveit::Profiler::ScopedBlock sblock("isValid");
  return planning_context_->useStateValidityCache() ? isValidWithCache(state, dist, verbose) :
                                                      isValidWithoutCache(state, dist, verbose);
}

double ompl_interface::StateValidityChecker::cost(const ompl::base::State* state) const
{
  double cost = 0.0;

  robot_state::RobotState* robot_state = tss_.getStateStorage();
  planning_context_->getOMPLStateSpace()->copyToRobotState(*robot_state, state);

  // Calculates cost from a summation of distance to obstacles times the size of the obstacle
  collision_detection::CollisionResult res;
  planning_context_->getPlanningScene()->checkCollision(collision_request_with_cost_, res, *robot_state);

  for (const collision_detection::CostSource& cost_source : res.cost_sources)
    cost += cost_source.cost * cost_source.getVolume();

  return cost;
}

double ompl_interface::StateValidityChecker::clearance(const ompl::base::State* state) const
{
  robot_state::RobotState* robot_state = tss_.getStateStorage();
  planning_context_->getOMPLStateSpace()->copyToRobotState(*robot_state, state);
  robot_state->updateCollisionBodyTransforms();

  // get the collision environment handle which is a wrapper for fcl
  const collision_detection::CollisionEnvConstPtr collision_env = 
    planning_context_->getPlanningScene()->getCollisionEnvUnpadded();

  collision_detection::DistanceRequest req;
  req.group_name = planning_context_->getGroupName();
  req.acm = &(planning_context_->getPlanningScene()->getAllowedCollisionMatrix());
  req.enableGroup(collision_env->getRobotModel());
  req.enable_nearest_points = true;
  req.compute_gradient = true;
  req.type = collision_detection::DistanceRequestType::SINGLE; // Find the global minimum distance for each pair

  collision_detection::DistanceResult res;
  collision_env->distanceRobot(req, res, *robot_state);

  if (res.collision)
    return 0.0;

  if (res.minimum_distance.distance > 0.2)
    return std::numeric_limits<double>::infinity();

  double clearance = std::numeric_limits<double>::infinity();
  const robot_model::JointModelGroup* group = robot_state->getJointModelGroup(planning_context_->getGroupName());

  /*for (auto const& distance_map : res.distances) {
    if (distance_map.second.size() == 0)
      continue;*/

    const collision_detection::DistanceResultsData& distance_data = res.minimum_distance; // distance_map.second[0];
    double d = distance_data.distance;
    Eigen::Vector3d p1; // point on robot
    Eigen::Vector3d p2; // point on obstacle
    const robot_model::LinkModel* link; // link model of robot link that is colliding
    if (distance_data.body_types[0] == collision_detection::BodyType::WORLD_OBJECT) {
      p1 = distance_data.nearest_points[1]; 
      p2 = distance_data.nearest_points[0];
      link = robot_state->getLinkModel(distance_data.link_names[1]);
    } else if (distance_data.body_types[1] == collision_detection::BodyType::WORLD_OBJECT) {
      p1 = distance_data.nearest_points[0];
      p2 = distance_data.nearest_points[1];
      link = robot_state->getLinkModel(distance_data.link_names[0]);
    } else {
      ROS_WARN_NAMED("state_validity_checker", "Both distance objects are not a WORLD_OBJECT");
    }

    Eigen::Vector3d normal = distance_data.normal;

    // vector from link 1 origin to p1
    Eigen::Vector3d p1_rel = p1 - robot_state->getGlobalLinkTransform(link).translation();

    Eigen::MatrixXd J_p1;
    robot_state->getJacobian(group, link, p1_rel, J_p1);

    Eigen::MatrixXd J_p1_trans = J_p1.topRows(3); // extract only translational part

    double norm_dd_dq = (normal.transpose() * J_p1_trans).norm();

    if (norm_dd_dq < 0.001)
      return std::numeric_limits<double>::infinity();

    double radius_q_free = d / norm_dd_dq;

    clearance = radius_q_free < clearance ? radius_q_free : clearance;
  //}

  return clearance;
}

/*double ompl_interface::StateValidityChecker::clearance(const ompl::base::State* state) const
{
  robot_state::RobotState* robot_state = tss_.getStateStorage();
  planning_context_->getOMPLStateSpace()->copyToRobotState(*robot_state, state);

  collision_detection::CollisionResult res;
  planning_context_->getPlanningScene()->checkCollision(collision_request_with_distance_, res, *robot_state);
  return res.collision ? 0.0 : (res.distance < 0.0 ? std::numeric_limits<double>::infinity() : res.distance);
}*/

bool ompl_interface::StateValidityChecker::isValidWithoutCache(const ompl::base::State* state, bool verbose) const
{
  // check bounds
  if (!si_->satisfiesBounds(state))
  {
    if (verbose)
      ROS_INFO_NAMED("state_validity_checker", "State outside bounds");
    return false;
  }

  // convert ompl state to MoveIt robot state
  robot_state::RobotState* robot_state = tss_.getStateStorage();
  planning_context_->getOMPLStateSpace()->copyToRobotState(*robot_state, state);

  // check path constraints
  const kinematic_constraints::KinematicConstraintSetPtr& kset = planning_context_->getPathConstraints();
  if (kset && !kset->decide(*robot_state, verbose).satisfied)
    return false;

  // check feasibility
  if (!planning_context_->getPlanningScene()->isStateFeasible(*robot_state, verbose))
    return false;

  // check collision avoidance
  collision_detection::CollisionResult res;
  planning_context_->getPlanningScene()->checkCollision(
      verbose ? collision_request_simple_verbose_ : collision_request_simple_, res, *robot_state);
  return !res.collision;
}

bool ompl_interface::StateValidityChecker::isValidWithoutCache(const ompl::base::State* state, double& dist,
                                                               bool verbose) const
{
  if (!si_->satisfiesBounds(state))
  {
    if (verbose)
      ROS_INFO_NAMED("state_validity_checker", "State outside bounds");
    return false;
  }

  robot_state::RobotState* robot_state = tss_.getStateStorage();
  planning_context_->getOMPLStateSpace()->copyToRobotState(*robot_state, state);

  // check path constraints
  const kinematic_constraints::KinematicConstraintSetPtr& kset = planning_context_->getPathConstraints();
  if (kset)
  {
    kinematic_constraints::ConstraintEvaluationResult cer = kset->decide(*robot_state, verbose);
    if (!cer.satisfied)
    {
      dist = cer.distance;
      return false;
    }
  }

  // check feasibility
  if (!planning_context_->getPlanningScene()->isStateFeasible(*robot_state, verbose))
  {
    dist = 0.0;
    return false;
  }

  // check collision avoidance
  collision_detection::CollisionResult res;
  planning_context_->getPlanningScene()->checkCollision(
      verbose ? collision_request_with_distance_verbose_ : collision_request_with_distance_, res, *robot_state);
  dist = res.distance;
  return !res.collision;
}

bool ompl_interface::StateValidityChecker::isValidWithCache(const ompl::base::State* state, bool verbose) const
{
  if (state->as<ModelBasedStateSpace::StateType>()->isValidityKnown())
    return state->as<ModelBasedStateSpace::StateType>()->isMarkedValid();

  if (!si_->satisfiesBounds(state))
  {
    if (verbose)
      ROS_INFO_NAMED("state_validity_checker", "State outside bounds");
    const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
    return false;
  }

  robot_state::RobotState* robot_state = tss_.getStateStorage();
  planning_context_->getOMPLStateSpace()->copyToRobotState(*robot_state, state);

  // check path constraints
  const kinematic_constraints::KinematicConstraintSetPtr& kset = planning_context_->getPathConstraints();
  if (kset && !kset->decide(*robot_state, verbose).satisfied)
  {
    const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
    return false;
  }

  // check feasibility
  if (!planning_context_->getPlanningScene()->isStateFeasible(*robot_state, verbose))
  {
    const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
    return false;
  }

  // check collision avoidance
  collision_detection::CollisionResult res;
  planning_context_->getPlanningScene()->checkCollision(
      verbose ? collision_request_simple_verbose_ : collision_request_simple_, res, *robot_state);
  if (!res.collision)
  {
    const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markValid();
  }
  else
  {
    const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
  }
  return !res.collision;
}

bool ompl_interface::StateValidityChecker::isValidWithCache(const ompl::base::State* state, double& dist,
                                                            bool verbose) const
{
  if (state->as<ModelBasedStateSpace::StateType>()->isValidityKnown() &&
      state->as<ModelBasedStateSpace::StateType>()->isGoalDistanceKnown())
  {
    dist = state->as<ModelBasedStateSpace::StateType>()->distance;
    return state->as<ModelBasedStateSpace::StateType>()->isMarkedValid();
  }

  if (!si_->satisfiesBounds(state))
  {
    if (verbose)
      ROS_INFO_NAMED("state_validity_checker", "State outside bounds");
    const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid(0.0);
    return false;
  }

  robot_state::RobotState* robot_state = tss_.getStateStorage();
  planning_context_->getOMPLStateSpace()->copyToRobotState(*robot_state, state);

  // check path constraints
  const kinematic_constraints::KinematicConstraintSetPtr& kset = planning_context_->getPathConstraints();
  if (kset)
  {
    kinematic_constraints::ConstraintEvaluationResult cer = kset->decide(*robot_state, verbose);
    if (!cer.satisfied)
    {
      dist = cer.distance;
      const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid(dist);
      return false;
    }
  }

  // check feasibility
  if (!planning_context_->getPlanningScene()->isStateFeasible(*robot_state, verbose))
  {
    dist = 0.0;
    return false;
  }

  // check collision avoidance
  collision_detection::CollisionResult res;
  planning_context_->getPlanningScene()->checkCollision(
      verbose ? collision_request_with_distance_verbose_ : collision_request_with_distance_, res, *robot_state);
  dist = res.distance;
  return !res.collision;
}
