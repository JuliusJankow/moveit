#pragma once

#include "ompl/base/objectives/StateCostIntegralObjective.h"

namespace ompl_interface
{
class ModelBasedPlanningContext;
        
/** \brief Defines optimization objectives where path cost can
    be represented as a path integral over a cost function
    defined over the state space. This cost function is
    specified by implementing the stateCost() method. */
class StateCostIntegralObjectiveCustom : public ompl::base::StateCostIntegralObjective
{
public:
    /** \brief If enableMotionCostInterpolation is set to
        true, then calls to motionCost() will divide the
        motion segment into smaller parts (the number of parts
        being defined by StateSpace::validSegmentCount()) for
        more accurate cost integral computation (but this
        takes more computation time). If
        enableMotionCostInterpolation is false (the default),
        only the two endpoint states are used for motion cost
        computation.
    */
    StateCostIntegralObjectiveCustom(const ModelBasedPlanningContext* pc, const ompl::base::SpaceInformationPtr &si, bool enableMotionCostInterpolation = true) :
        StateCostIntegralObjective(si,enableMotionCostInterpolation),
        planning_context_(pc),
        work_state_(pc->getCompleteInitialRobotState()) {}

    /** \brief Returns a cost with a value of 1. */
    ompl::base::Cost stateCost(const ompl::base::State *s) const override;

    /** \brief Compute the cost of a path segment from \e s1 to \e s2 (including endpoints)
        \param s1 start state of the motion to be evaluated
        \param s2 final state of the motion to be evaluated
        \param cost the cost of the motion segment

        By default, this function computes
        \f{eqnarray*}{
        \mbox{cost} &=& \frac{cost(s_1) + cost(s_2)}{2}\vert s_1 - s_2 \vert
        \f}

        If enableMotionCostInterpolation was specified as true
        in constructing this object, the cost will be computed
        by separating the motion into
        StateSpace::validSegmentCount() segments, using the
        above formula to compute the cost of each of those
        segments, and adding them up.
    */
    ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const override;

    ompl::base::Cost motionCostHeuristic(const ompl::base::State *s1, const ompl::base::State *s2) const override;

    double getWorkSpaceDistance(const ompl::base::State *s1, const ompl::base::State *s2) const;
private:
    double segmentCostDefault(const ompl::base::State *s1, const ompl::base::State *s2) const;
    double segmentCostGravity(const ompl::base::State *s1, const ompl::base::State *s2) const;
    double segmentCostSafetyDirection(const ompl::base::State *s1, const ompl::base::State *s2) const;
    double segmentCostQGravity(const ompl::base::State *s1, const ompl::base::State *s2) const;
    double segmentCostXSafetyDirection(const ompl::base::State *s1, const ompl::base::State *s2) const;
    double segmentCostQDefault(const ompl::base::State *s1, const ompl::base::State *s2) const;
    double segmentCostXDefault(const ompl::base::State *s1, const ompl::base::State *s2) const;

    Eigen::VectorXd getGravity(const Eigen::VectorXd& q) const;

    robot_state::RobotState work_state_;
    const ModelBasedPlanningContext* planning_context_;
};
}
