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

/* Author: Ioan Sucan */

#include <moveit/ompl_interface/planning_context_manager.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/profiler/profiler.h>
#include <algorithm>
#include <set>
#include <utility>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/PRMsep.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/est/ProjEST.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>

#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space_factory.h>
#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h>
#include <moveit/ompl_interface/parameterization/work_space/pose_model_state_space_factory.h>

using namespace std::placeholders;

namespace ompl_interface
{
struct PlanningContextManager::CachedContexts
{
  std::map<std::pair<std::string, std::string>, std::vector<ModelBasedPlanningContextPtr> > contexts_;
  std::mutex lock_;
};

}  // namespace ompl_interface

ompl_interface::PlanningContextManager::PlanningContextManager(robot_model::RobotModelConstPtr robot_model,
                                                               constraint_samplers::ConstraintSamplerManagerPtr csm)
  : robot_model_(std::move(robot_model))
  , constraint_sampler_manager_(std::move(csm))
  , max_goal_samples_(10)
  , max_state_sampling_attempts_(4)
  , max_goal_sampling_attempts_(1000)
  , max_planning_threads_(4)
  , max_solution_segment_length_(0.0)
  , minimum_waypoint_count_(2)
{
  cached_contexts_.reset(new CachedContexts());
  registerDefaultPlanners();
  registerDefaultStateSpaces();
}

ompl_interface::PlanningContextManager::~PlanningContextManager() = default;

namespace
{
using namespace ompl_interface;

template <typename T>
static ompl::base::PlannerPtr allocatePlanner(const ob::SpaceInformationPtr& si, const std::string& new_name,
                                              const ModelBasedPlanningContextSpecification& spec)
{
  ompl::base::PlannerPtr planner(new T(si));
  if (!new_name.empty())
    planner->setName(new_name);
  planner->params().setParams(spec.config_, true);
  planner->setup();
  return planner;
}
}  // namespace

ompl_interface::ConfiguredPlannerAllocator
ompl_interface::PlanningContextManager::plannerSelector(const std::string& planner) const
{
  auto it = known_planners_.find(planner);
  if (it != known_planners_.end())
    return it->second;
  else
  {
    ROS_ERROR_NAMED("planning_context_manager", "Unknown planner: '%s'", planner.c_str());
    return ConfiguredPlannerAllocator();
  }
}

void ompl_interface::PlanningContextManager::registerDefaultPlanners()
{
  registerPlannerAllocator(  //
      "geometric::RRT",      //
      std::bind(&allocatePlanner<og::RRT>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  registerPlannerAllocator(     //
      "geometric::RRTConnect",  //
      std::bind(&allocatePlanner<og::RRTConnect>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  registerPlannerAllocator(  //
      "geometric::LazyRRT",  //
      std::bind(&allocatePlanner<og::LazyRRT>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  registerPlannerAllocator(  //
      "geometric::TRRT",     //
      std::bind(&allocatePlanner<og::TRRT>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  registerPlannerAllocator(  //
      "geometric::EST",      //
      std::bind(&allocatePlanner<og::EST>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  registerPlannerAllocator(  //
      "geometric::SBL",      //
      std::bind(&allocatePlanner<og::SBL>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  registerPlannerAllocator(  //
      "geometric::KPIECE",   //
      std::bind(&allocatePlanner<og::KPIECE1>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  registerPlannerAllocator(  //
      "geometric::BKPIECE",  //
      std::bind(&allocatePlanner<og::BKPIECE1>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  registerPlannerAllocator(   //
      "geometric::LBKPIECE",  //
      std::bind(&allocatePlanner<og::LBKPIECE1>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  registerPlannerAllocator(  //
      "geometric::RRTstar",  //
      std::bind(&allocatePlanner<og::RRTstar>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  registerPlannerAllocator(  //
      "geometric::PRM",      //
      std::bind(&allocatePlanner<og::PRM>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  registerPlannerAllocator(  //
      "geometric::PRMstar",  //
      std::bind(&allocatePlanner<og::PRMstar>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  registerPlannerAllocator(  //
      "geometric::PRMsep",  //
      std::bind(&allocatePlanner<og::PRMsep>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  registerPlannerAllocator(  //
      "geometric::FMT",      //
      std::bind(&allocatePlanner<og::FMT>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  registerPlannerAllocator(  //
      "geometric::BFMT",     //
      std::bind(&allocatePlanner<og::BFMT>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  registerPlannerAllocator(  //
      "geometric::PDST",     //
      std::bind(&allocatePlanner<og::PDST>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  registerPlannerAllocator(  //
      "geometric::STRIDE",   //
      std::bind(&allocatePlanner<og::STRIDE>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  registerPlannerAllocator(  //
      "geometric::BiTRRT",   //
      std::bind(&allocatePlanner<og::BiTRRT>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  registerPlannerAllocator(  //
      "geometric::LBTRRT",   //
      std::bind(&allocatePlanner<og::LBTRRT>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  registerPlannerAllocator(  //
      "geometric::BiEST",    //
      std::bind(&allocatePlanner<og::BiEST>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  registerPlannerAllocator(  //
      "geometric::ProjEST",  //
      std::bind(&allocatePlanner<og::ProjEST>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  registerPlannerAllocator(  //
      "geometric::LazyPRM",  //
      std::bind(&allocatePlanner<og::LazyPRM>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  registerPlannerAllocator(      //
      "geometric::LazyPRMstar",  //
      std::bind(&allocatePlanner<og::LazyPRMstar>, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3));
  registerPlannerAllocator(  //
      "geometric::SPARS",    //
      std::bind(&allocatePlanner<og::SPARS>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  registerPlannerAllocator(   //
      "geometric::SPARStwo",  //
      std::bind(&allocatePlanner<og::SPARStwo>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void ompl_interface::PlanningContextManager::registerDefaultStateSpaces()
{
  registerStateSpaceFactory(ModelBasedStateSpaceFactoryPtr(new JointModelStateSpaceFactory()));
  registerStateSpaceFactory(ModelBasedStateSpaceFactoryPtr(new PoseModelStateSpaceFactory()));
}

ompl_interface::ConfiguredPlannerSelector ompl_interface::PlanningContextManager::getPlannerSelector() const
{
  return std::bind(&PlanningContextManager::plannerSelector, this, std::placeholders::_1);
}

void ompl_interface::PlanningContextManager::setPlannerConfigurations(
    const planning_interface::PlannerConfigurationMap& pconfig)
{
  planner_configs_ = pconfig;
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::PlanningContextManager::getPlanningContext(
    const std::string& config, const std::string& factory_type) const
{
  auto pc = planner_configs_.find(config);

  if (pc != planner_configs_.end())
  {
    moveit_msgs::MotionPlanRequest req;  // dummy request with default values
    return getPlanningContext(
        pc->second,
        std::bind(&PlanningContextManager::getStateSpaceFactory1, this, std::placeholders::_1, factory_type), req);
  }
  else
  {
    ROS_ERROR_NAMED("planning_context_manager", "Planning configuration '%s' was not found", config.c_str());
    return ModelBasedPlanningContextPtr();
  }
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::PlanningContextManager::getPlanningContext(
    const planning_interface::PlannerConfigurationSettings& config,
    const StateSpaceFactoryTypeSelector& factory_selector, const moveit_msgs::MotionPlanRequest& req) const
{
  const ompl_interface::ModelBasedStateSpaceFactoryPtr& factory = factory_selector(config.group);

  // Check for a cached planning context
  ModelBasedPlanningContextPtr context;

  {
    std::unique_lock<std::mutex> slock(cached_contexts_->lock_);
    auto cached_contexts = cached_contexts_->contexts_.find(std::make_pair(config.name, factory->getType()));
    if (cached_contexts != cached_contexts_->contexts_.end())
    {
      for (const ModelBasedPlanningContextPtr& cached_context : cached_contexts->second)
        if (cached_context.unique())
        {
          ROS_DEBUG_NAMED("planning_context_manager", "Reusing cached planning context");
          context = cached_context;
          break;
        }
    }
  }

  // Create a new planning context
  if (!context)
  {
    ModelBasedStateSpaceSpecification space_spec(robot_model_, config.group);
    ModelBasedPlanningContextSpecification context_spec;
    context_spec.config_ = config.config;
    context_spec.planner_selector_ = getPlannerSelector();
    context_spec.constraint_sampler_manager_ = constraint_sampler_manager_;
    context_spec.state_space_ = factory->getNewStateSpace(space_spec);

    // Choose the correct simple setup type to load
    context_spec.ompl_simple_setup_.reset(new ompl::geometric::SimpleSetup(context_spec.state_space_));

    bool state_validity_cache = true;
    if (config.config.find("subspaces") != config.config.end())
    {
      context_spec.config_.erase("subspaces");
      // if the planner operates at subspace level the cache may be unsafe
      state_validity_cache = false;
      boost::char_separator<char> sep(" ");
      boost::tokenizer<boost::char_separator<char> > tok(config.config.at("subspaces"), sep);
      for (boost::tokenizer<boost::char_separator<char> >::iterator beg = tok.begin(); beg != tok.end(); ++beg)
      {
        const ompl_interface::ModelBasedStateSpaceFactoryPtr& sub_fact = factory_selector(*beg);
        if (sub_fact)
        {
          ModelBasedStateSpaceSpecification sub_space_spec(robot_model_, *beg);
          context_spec.subspaces_.push_back(sub_fact->getNewStateSpace(sub_space_spec));
        }
      }
    }

    ROS_DEBUG_NAMED("planning_context_manager", "Creating new planning context");
    context.reset(new ModelBasedPlanningContext(config.name, context_spec));
    context->useStateValidityCache(state_validity_cache);
    {
      std::unique_lock<std::mutex> slock(cached_contexts_->lock_);
      cached_contexts_->contexts_[std::make_pair(config.name, factory->getType())].push_back(context);
    }
  }

  context->setMaximumPlanningThreads(max_planning_threads_);
  context->setMaximumGoalSamples(max_goal_samples_);
  context->setMaximumStateSamplingAttempts(max_state_sampling_attempts_);
  context->setMaximumGoalSamplingAttempts(max_goal_sampling_attempts_);
  if (max_solution_segment_length_ > std::numeric_limits<double>::epsilon())
    context->setMaximumSolutionSegmentLength(max_solution_segment_length_);
  context->setMinimumWaypointCount(minimum_waypoint_count_);

  context->setSpecificationConfig(config.config);

  return context;
}

const ompl_interface::ModelBasedStateSpaceFactoryPtr& ompl_interface::PlanningContextManager::getStateSpaceFactory1(
    const std::string& /* dummy */, const std::string& factory_type) const
{
  auto f = factory_type.empty() ? state_space_factories_.begin() : state_space_factories_.find(factory_type);
  if (f != state_space_factories_.end())
    return f->second;
  else
  {
    ROS_ERROR_NAMED("planning_context_manager", "Factory of type '%s' was not found", factory_type.c_str());
    static const ModelBasedStateSpaceFactoryPtr EMPTY;
    return EMPTY;
  }
}

const ompl_interface::ModelBasedStateSpaceFactoryPtr& ompl_interface::PlanningContextManager::getStateSpaceFactory2(
    const std::string& group, const moveit_msgs::MotionPlanRequest& req) const
{
  // find the problem representation to use
  auto best = state_space_factories_.end();
  int prev_priority = -1;
  for (auto it = state_space_factories_.begin(); it != state_space_factories_.end(); ++it)
  {
    int priority = it->second->canRepresentProblem(group, req, robot_model_);
    if (priority > 0)
      if (best == state_space_factories_.end() || priority > prev_priority)
      {
        best = it;
        prev_priority = priority;
      }
  }

  if (best == state_space_factories_.end())
  {
    ROS_ERROR_NAMED("planning_context_manager", "There are no known state spaces that can represent the given planning "
                                                "problem");
    static const ModelBasedStateSpaceFactoryPtr EMPTY;
    return EMPTY;
  }
  else
  {
    ROS_DEBUG_NAMED("planning_context_manager", "Using '%s' parameterization for solving problem", best->first.c_str());
    return best->second;
  }
}

ompl_interface::ModelBasedPlanningContextPtr
ompl_interface::PlanningContextManager::getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                           const moveit_msgs::MotionPlanRequest& req,
                                                           moveit_msgs::MoveItErrorCodes& error_code) const
{
  if (req.group_name.empty())
  {
    ROS_ERROR_NAMED("planning_context_manager", "No group specified to plan for");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return ModelBasedPlanningContextPtr();
  }

  error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;

  if (!planning_scene)
  {
    ROS_ERROR_NAMED("planning_context_manager", "No planning scene supplied as input");
    return ModelBasedPlanningContextPtr();
  }

  // identify the correct planning configuration
  auto pc = planner_configs_.end();
  if (!req.planner_id.empty())
  {
    pc = planner_configs_.find(req.planner_id.find(req.group_name) == std::string::npos ?
                                   req.group_name + "[" + req.planner_id + "]" :
                                   req.planner_id);
    if (pc == planner_configs_.end())
      ROS_WARN_NAMED("planning_context_manager",
                     "Cannot find planning configuration for group '%s' using planner '%s'. Will use defaults instead.",
                     req.group_name.c_str(), req.planner_id.c_str());
  }

  if (pc == planner_configs_.end())
  {
    pc = planner_configs_.find(req.group_name);
    if (pc == planner_configs_.end())
    {
      ROS_ERROR_NAMED("planning_context_manager", "Cannot find planning configuration for group '%s'",
                      req.group_name.c_str());
      return ModelBasedPlanningContextPtr();
    }
  }

  // Check if sampling in JointModelStateSpace is enforced for this group by user.
  // This is done by setting 'enforce_joint_model_state_space' to 'true' for the desired group in ompl_planning.yaml.
  //
  // Some planning problems like orientation path constraints are represented in PoseModelStateSpace and sampled via IK.
  // However consecutive IK solutions are not checked for proximity at the moment and sometimes happen to be flipped,
  // leading to invalid trajectories. This workaround lets the user prevent this problem by forcing rejection sampling
  // in JointModelStateSpace.
  StateSpaceFactoryTypeSelector factory_selector;
  auto it = pc->second.config.find("enforce_joint_model_state_space");

  if (it != pc->second.config.end() && boost::lexical_cast<bool>(it->second))
    factory_selector = std::bind(&PlanningContextManager::getStateSpaceFactory1, this, std::placeholders::_1,
                                 JointModelStateSpace::PARAMETERIZATION_TYPE);
  else
    factory_selector = std::bind(&PlanningContextManager::getStateSpaceFactory2, this, std::placeholders::_1, req);

  ModelBasedPlanningContextPtr context = getPlanningContext(pc->second, factory_selector, req);

  if (context)
  {
    context->clear();

    robot_state::RobotStatePtr start_state = planning_scene->getCurrentStateUpdated(req.start_state);

    // Setup the context
    context->setPlanningScene(planning_scene);
    context->setMotionPlanRequest(req);
    context->setCompleteInitialState(*start_state);

    context->setPlanningVolume(req.workspace_parameters);
    if (!context->setPathConstraints(req.path_constraints, &error_code))
      return ModelBasedPlanningContextPtr();

    if (!context->setGoalConstraints(req.goal_constraints, req.path_constraints, &error_code))
      return ModelBasedPlanningContextPtr();

    try
    {
      context->configure();
      ROS_DEBUG_NAMED("planning_context_manager", "%s: New planning context is set.", context->getName().c_str());
      error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
    catch (ompl::Exception& ex)
    {
      ROS_ERROR_NAMED("planning_context_manager", "OMPL encountered an error: %s", ex.what());
      context.reset();
    }
  }

  return context;
}
