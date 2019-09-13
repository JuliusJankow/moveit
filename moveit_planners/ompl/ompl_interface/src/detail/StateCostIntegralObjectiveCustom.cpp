#include <moveit/robot_state/robot_state.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/ompl_interface/detail/StateCostIntegralObjectiveCustom.h>

ompl::base::Cost ompl_interface::StateCostIntegralObjectiveCustom::stateCost(const ompl::base::State *) const
{
    return ompl::base::Cost(1.0);
}

ompl::base::Cost ompl_interface::StateCostIntegralObjectiveCustom::motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    if (interpolateMotionCost_)
    {
        ompl::base::Cost totalCost = this->identityCost();

        int nd = si_->getStateSpace()->validSegmentCount(s1, s2);

        ompl::base::State *test1 = si_->cloneState(s1);
        ompl::base::Cost prevStateCost = this->stateCost(test1);
        if (nd > 1)
        {
            ompl::base::State *test2 = si_->allocState();
            for (int j = 1; j < nd; ++j)
            {
                si_->getStateSpace()->interpolate(s1, s2, (double)j / (double)nd, test2);
                ompl::base::Cost nextStateCost = this->stateCost(test2);
                double distance = si_->distance(test1, test2) + getWorkSpaceDistance(test1, test2);
                totalCost = ompl::base::Cost(totalCost.value() +
                                 this->trapezoid(prevStateCost, nextStateCost, distance).value());
                std::swap(test1, test2);
                prevStateCost = nextStateCost;
            }
            si_->freeState(test2);
        }

        // Lastly, add s2
        totalCost = ompl::base::Cost(totalCost.value() +
                         this->trapezoid(prevStateCost, this->stateCost(s2), si_->distance(test1, s2)).value());

        si_->freeState(test1);

        return totalCost;
    }

    return this->trapezoid(this->stateCost(s1), this->stateCost(s2), si_->distance(s1, s2));
}

double ompl_interface::StateCostIntegralObjectiveCustom::getWorkSpaceDistance(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    std::string link_name = "panda_link8";
    robot_state::RobotState robot_state(work_state_);

    planning_context_->getOMPLStateSpace()->copyToRobotState(robot_state, s1);
    robot_state.updateLinkTransforms();
    Eigen::Vector3d x_EE_1 = robot_state.getGlobalLinkTransform(link_name).translation();

    planning_context_->getOMPLStateSpace()->copyToRobotState(robot_state, s2);
    robot_state.updateLinkTransforms();
    Eigen::Vector3d x_EE_2 = robot_state.getGlobalLinkTransform(link_name).translation();

    return (x_EE_1 - x_EE_2).norm();
}
