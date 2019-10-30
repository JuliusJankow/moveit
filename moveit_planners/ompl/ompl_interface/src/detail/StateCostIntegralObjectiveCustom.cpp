#include <moveit/robot_state/robot_state.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/ompl_interface/detail/StateCostIntegralObjectiveCustom.h>

ompl::base::Cost ompl_interface::StateCostIntegralObjectiveCustom::stateCost(const ompl::base::State *) const
{
    return ompl::base::Cost(1.0);
}

double ompl_interface::StateCostIntegralObjectiveCustom::segmentCostQDefault(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    return 1.0;
}

double ompl_interface::StateCostIntegralObjectiveCustom::segmentCostQGravity(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    robot_state::RobotState robot_state(work_state_);
    planning_context_->getOMPLStateSpace()->copyToRobotState(robot_state, s1);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(7);
    for (size_t j=0; j<7; j++) {
        q(j) = robot_state.getVariablePosition(j);
    }
    Eigen::VectorXd tau_g_1 = getGravity(q);
    planning_context_->getOMPLStateSpace()->copyToRobotState(robot_state, s2);
    for (size_t j=0; j<7; j++) {
        q(j) = robot_state.getVariablePosition(j);
    }
    Eigen::VectorXd tau_g_2 = getGravity(q);

    double cost = 0.0;
    for (size_t j=0; j<7; j++) {
        cost += std::fabs(tau_g_1(j));
        cost += std::fabs(tau_g_2(j));
    }

    return (cost + 1.0);
}

double ompl_interface::StateCostIntegralObjectiveCustom::segmentCostXDefault(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    return 1.0;
}

double ompl_interface::StateCostIntegralObjectiveCustom::segmentCostXSafetyDirection(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    Eigen::Vector3d e_tool(0.0, 0.0, 1.0);
    Eigen::Vector3d zero_v(0.0,0.0,0.0);

    std::string link_name = "panda_link8";
    robot_state::RobotState robot_state(work_state_);
    const robot_model::JointModelGroup* group = robot_state.getJointModelGroup("panda_arm");
    const robot_model::LinkModel* link = robot_state.getLinkModel(link_name);

    planning_context_->getOMPLStateSpace()->copyToRobotState(robot_state, s1);
    robot_state.updateLinkTransforms();
    Eigen::Vector3d x_EE_1 = robot_state.getGlobalLinkTransform(link_name).translation();
    Eigen::Matrix3d R_EE_1 = robot_state.getGlobalLinkTransform(link_name).rotation();
    Eigen::MatrixXd J_EE_1;
    robot_state.getJacobian(group, link, zero_v, J_EE_1);
    Eigen::VectorXd q1 = Eigen::VectorXd::Zero(7);
    for (size_t j=0; j<7; j++) {
        q1(j) = robot_state.getVariablePosition(j);
    }

    planning_context_->getOMPLStateSpace()->copyToRobotState(robot_state, s2);
    robot_state.updateLinkTransforms();
    Eigen::Vector3d x_EE_2 = robot_state.getGlobalLinkTransform(link_name).translation();
    Eigen::Matrix3d R_EE_2 = robot_state.getGlobalLinkTransform(link_name).rotation();
    Eigen::MatrixXd J_EE_2;
    robot_state.getJacobian(group, link, zero_v, J_EE_2);
    Eigen::VectorXd q2 = Eigen::VectorXd::Zero(7);
    for (size_t j=0; j<7; j++) {
        q2(j) = robot_state.getVariablePosition(j);
    }
    
    /*Eigen::Vector3d e_x_motion_1 = (J_EE_1 * (q2 - q1)).normalized();
    Eigen::Vector3d e_x_motion_2 = (J_EE_2 * (q2 - q1)).normalized();*/

    /*Eigen::Vector3d e_x_danger_1 = R_EE_1 * e_danger;
    Eigen::Vector3d e_x_danger_2 = R_EE_2 * e_danger;*/

    Eigen::Vector3d e_tool_actual_1 = R_EE_1 * e_tool;
    Eigen::Vector3d e_tool_actual_2 = R_EE_2 * e_tool;

    Eigen::Vector3d e_tool_danger_1 = x_EE_1.normalized();
    Eigen::Vector3d e_tool_danger_2 = x_EE_2.normalized();

    //return 15*(3.0 + e_x_motion_1.dot(e_x_danger_1) + e_x_motion_2.dot(e_x_danger_2));
    return 15*(3.0 + e_tool_actual_1.dot(e_tool_danger_1) + e_tool_actual_2.dot(e_tool_danger_2)); // result is in between [1, 75]
}

double ompl_interface::StateCostIntegralObjectiveCustom::segmentCostDefault(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    double cq = std::max(1.0, segmentCostQDefault(s1, s2));
    double cx = std::max(1.0, segmentCostXDefault(s1, s2));
    return si_->distance(s1, s2) * cq + getWorkSpaceDistance(s1, s2) * cx;
}

double ompl_interface::StateCostIntegralObjectiveCustom::segmentCostSafetyDirection(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    double cq = std::max(1.0, segmentCostQDefault(s1, s2));
    double cx = std::max(1.0, segmentCostXSafetyDirection(s1, s2));
    return si_->distance(s1, s2) * cq + getWorkSpaceDistance(s1, s2) * cx;
}

double ompl_interface::StateCostIntegralObjectiveCustom::segmentCostGravity(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    double cq = std::max(1.0, segmentCostQGravity(s1, s2));
    double cx = std::max(1.0, segmentCostXDefault(s1, s2));
    return si_->distance(s1, s2) * cq + getWorkSpaceDistance(s1, s2) * cx;
}

ompl::base::Cost ompl_interface::StateCostIntegralObjectiveCustom::motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    if (interpolateMotionCost_)
    {
        ompl::base::Cost totalCost = ompl::base::Cost(0.0);

        double resolution = 0.1;
        double distance = si_->distance(s1, s2);
        int nd = (distance / resolution);

        ompl::base::State *test1 = si_->cloneState(s1);
        if (nd > 1)
        {
            ompl::base::State *test2 = si_->allocState();
            for (int j = 1; j <= nd; ++j)
            {
                si_->getStateSpace()->interpolate(s1, s2, (double)j / (double)nd, test2);
                totalCost = ompl::base::Cost(totalCost.value() + segmentCostDefault(test1, test2));
                std::swap(test1, test2);
            }
            si_->freeState(test2);
            si_->freeState(test1);

            return totalCost;
        }
        si_->freeState(test1);
    }

    return ompl::base::Cost(segmentCostDefault(s1, s2));
}

ompl::base::Cost ompl_interface::StateCostIntegralObjectiveCustom::motionCostHeuristic(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    double cost_heuristic = si_->distance(s1, s2) + getWorkSpaceDistance(s1, s2);
    return ompl::base::Cost(cost_heuristic);
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

Eigen::VectorXd ompl_interface::StateCostIntegralObjectiveCustom::getGravity(const Eigen::VectorXd& q) const
{
    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);
    double q4 = q(3);
    double q5 = q(4);
    double q6 = q(5);
    double q7 = q(6);

    // real kin pars
    double g0 = 9.80665;
    double a4 = 0.0825;
    double a5 = -0.0825;
    double a7 = 0.088;
    double d1 = 0.333;
    double d3 = 0.316;
    double d5 = 0.384;

    double t2 = cos(q2);
    double t3 = sin(q2);
    double t4 = cos(q3);
    double t5 = cos(q4);
    double t6 = sin(q4);
    double t7 = sin(q3);
    double t8 = cos(q5);
    double t9 = sin(q5);
    double t10 = cos(q6);
    double t11 = sin(q6);
    double t12 = cos(q7);
    double t13 = sin(q7);

    Eigen::VectorXd g_hat = Eigen::VectorXd::Zero(7);
    g_hat << 0.0,
             g0*t2*5.368784261474552e-3-g0*t3*3.10257567176716-g0*t2*t4*6.873533102775262e-1-g0*t3*t5*1.718488169611198+g0*t2*t7*2.337649457371796e-2+g0*t3*t6*4.888360246332429e-1+g0*t2*t4*t5*4.888360246332429e-1+g0*t2*t4*t6*1.718488169611198+g0*t2*t7*t8*7.705324195893029e-2+g0*t3*t6*t8*1.00039522535591e-2-g0*t2*t7*t9*1.00039522535591e-2+g0*t3*t5*t10*6.771174144247946e-2+g0*t3*t6*t9*7.705324195893029e-2-g0*t3*t5*t11*1.656809687593008e-1+g0*t2*t4*t5*t8*1.00039522535591e-2+g0*t2*t4*t5*t9*7.705324195893029e-2-g0*t2*t4*t6*t10*6.771174144247946e-2+g0*t2*t4*t6*t11*1.656809687593008e-1-g0*t3*t6*t8*t10*1.656809687593008e-1+g0*t2*t7*t9*t10*1.656809687593008e-1-g0*t3*t6*t8*t11*6.771174144247946e-2+g0*t2*t7*t8*t12*4.883521085127657e-4+g0*t2*t7*t9*t11*6.771174144247946e-2-g0*t2*t7*t8*t13*6.230758965393622e-3+g0*t3*t6*t9*t12*4.883521085127657e-4-g0*t3*t5*t11*t12*6.230758965393622e-3-g0*t3*t6*t9*t13*6.230758965393622e-3-g0*t3*t5*t11*t13*4.883521085127657e-4-g0*t2*t4*t5*t8*t10*1.656809687593008e-1-g0*t2*t4*t5*t8*t11*6.771174144247946e-2+g0*t2*t4*t5*t9*t12*4.883521085127657e-4-g0*t2*t4*t5*t9*t13*6.230758965393622e-3+g0*t2*t4*t6*t11*t12*6.230758965393622e-3+g0*t2*t4*t6*t11*t13*4.883521085127657e-4-g0*t3*t6*t8*t10*t12*6.230758965393622e-3+g0*t2*t7*t9*t10*t12*6.230758965393622e-3-g0*t3*t6*t8*t10*t13*4.883521085127657e-4+g0*t2*t7*t9*t10*t13*4.883521085127657e-4-g0*t2*t4*t5*t8*t10*t12*6.230758965393622e-3-g0*t2*t4*t5*t8*t10*t13*4.883521085127657e-4,
             g0*t3*t4*2.337649457371796e-2+g0*t3*t7*6.873533102775262e-1+g0*t3*t4*t8*7.705324195893029e-2-g0*t3*t5*t7*4.888360246332429e-1-g0*t3*t4*t9*1.00039522535591e-2-g0*t3*t6*t7*1.718488169611198-g0*t3*t5*t7*t8*1.00039522535591e-2-g0*t3*t5*t7*t9*7.705324195893029e-2+g0*t3*t4*t9*t10*1.656809687593008e-1+g0*t3*t6*t7*t10*6.771174144247946e-2+g0*t3*t4*t8*t12*4.883521085127657e-4+g0*t3*t4*t9*t11*6.771174144247946e-2-g0*t3*t6*t7*t11*1.656809687593008e-1-g0*t3*t4*t8*t13*6.230758965393622e-3+g0*t3*t5*t7*t8*t10*1.656809687593008e-1+g0*t3*t5*t7*t8*t11*6.771174144247946e-2-g0*t3*t5*t7*t9*t12*4.883521085127657e-4+g0*t3*t5*t7*t9*t13*6.230758965393622e-3+g0*t3*t4*t9*t10*t12*6.230758965393622e-3+g0*t3*t4*t9*t10*t13*4.883521085127657e-4-g0*t3*t6*t7*t11*t12*6.230758965393622e-3-g0*t3*t6*t7*t11*t13*4.883521085127657e-4+g0*t3*t5*t7*t8*t10*t12*6.230758965393622e-3+g0*t3*t5*t7*t8*t10*t13*4.883521085127657e-4,
             g0*t2*t5*(-4.888360246332429e-1)-g0*t2*t6*1.718488169611198+g0*t3*t4*t5*1.718488169611198-g0*t3*t4*t6*4.888360246332429e-1-g0*t2*t5*t8*1.00039522535591e-2-g0*t2*t5*t9*7.705324195893029e-2+g0*t2*t6*t10*6.771174144247946e-2-g0*t2*t6*t11*1.656809687593008e-1-g0*t3*t4*t6*t8*1.00039522535591e-2-g0*t3*t4*t5*t10*6.771174144247946e-2-g0*t3*t4*t6*t9*7.705324195893029e-2+g0*t3*t4*t5*t11*1.656809687593008e-1+g0*t2*t5*t8*t10*1.656809687593008e-1+g0*t2*t5*t8*t11*6.771174144247946e-2-g0*t2*t5*t9*t12*4.883521085127657e-4+g0*t2*t5*t9*t13*6.230758965393622e-3-g0*t2*t6*t11*t12*6.230758965393622e-3-g0*t2*t6*t11*t13*4.883521085127657e-4+g0*t3*t4*t6*t8*t10*1.656809687593008e-1+g0*t3*t4*t6*t8*t11*6.771174144247946e-2-g0*t3*t4*t6*t9*t12*4.883521085127657e-4+g0*t3*t4*t5*t11*t12*6.230758965393622e-3+g0*t3*t4*t6*t9*t13*6.230758965393622e-3+g0*t3*t4*t5*t11*t13*4.883521085127657e-4+g0*t2*t5*t8*t10*t12*6.230758965393622e-3+g0*t2*t5*t8*t10*t13*4.883521085127657e-4+g0*t3*t4*t6*t8*t10*t12*6.230758965393622e-3+g0*t3*t4*t6*t8*t10*t13*4.883521085127657e-4,
             g0*t2*t6*t8*(-7.705324195893029e-2)+g0*t2*t6*t9*1.00039522535591e-2-g0*t3*t7*t8*1.00039522535591e-2-g0*t3*t7*t9*7.705324195893029e-2+g0*t3*t4*t5*t8*7.705324195893029e-2-g0*t3*t4*t5*t9*1.00039522535591e-2-g0*t2*t6*t9*t10*1.656809687593008e-1-g0*t2*t6*t8*t12*4.883521085127657e-4-g0*t2*t6*t9*t11*6.771174144247946e-2+g0*t3*t7*t8*t10*1.656809687593008e-1+g0*t2*t6*t8*t13*6.230758965393622e-3+g0*t3*t7*t8*t11*6.771174144247946e-2-g0*t3*t7*t9*t12*4.883521085127657e-4+g0*t3*t7*t9*t13*6.230758965393622e-3+g0*t3*t4*t5*t9*t10*1.656809687593008e-1+g0*t3*t4*t5*t8*t12*4.883521085127657e-4+g0*t3*t4*t5*t9*t11*6.771174144247946e-2-g0*t3*t4*t5*t8*t13*6.230758965393622e-3-g0*t2*t6*t9*t10*t12*6.230758965393622e-3-g0*t2*t6*t9*t10*t13*4.883521085127657e-4+g0*t3*t7*t8*t10*t12*6.230758965393622e-3+g0*t3*t7*t8*t10*t13*4.883521085127657e-4+g0*t3*t4*t5*t9*t10*t12*6.230758965393622e-3+g0*t3*t4*t5*t9*t10*t13*4.883521085127657e-4,
             g0*t2*t5*t10*1.656809687593008e-1+g0*t2*t5*t11*6.771174144247946e-2+g0*t3*t4*t6*t10*1.656809687593008e-1+g0*t3*t4*t6*t11*6.771174144247946e-2+g0*t2*t6*t8*t10*6.771174144247946e-2-g0*t2*t6*t8*t11*1.656809687593008e-1+g0*t2*t5*t10*t12*6.230758965393622e-3+g0*t3*t7*t9*t10*6.771174144247946e-2+g0*t2*t5*t10*t13*4.883521085127657e-4-g0*t3*t7*t9*t11*1.656809687593008e-1-g0*t3*t4*t5*t8*t10*6.771174144247946e-2+g0*t3*t4*t5*t8*t11*1.656809687593008e-1+g0*t3*t4*t6*t10*t12*6.230758965393622e-3+g0*t3*t4*t6*t10*t13*4.883521085127657e-4-g0*t2*t6*t8*t11*t12*6.230758965393622e-3-g0*t2*t6*t8*t11*t13*4.883521085127657e-4-g0*t3*t7*t9*t11*t12*6.230758965393622e-3-g0*t3*t7*t9*t11*t13*4.883521085127657e-4+g0*t3*t4*t5*t8*t11*t12*6.230758965393622e-3+g0*t3*t4*t5*t8*t11*t13*4.883521085127657e-4,
             g0*t2*t6*t9*t12*6.230758965393622e-3+g0*t2*t5*t11*t12*4.883521085127657e-4+g0*t2*t6*t9*t13*4.883521085127657e-4-g0*t3*t7*t8*t12*6.230758965393622e-3-g0*t2*t5*t11*t13*6.230758965393622e-3-g0*t3*t7*t8*t13*4.883521085127657e-4-g0*t3*t4*t5*t9*t12*6.230758965393622e-3-g0*t3*t4*t5*t9*t13*4.883521085127657e-4+g0*t3*t4*t6*t11*t12*4.883521085127657e-4-g0*t3*t4*t6*t11*t13*6.230758965393622e-3+g0*t2*t6*t8*t10*t12*4.883521085127657e-4-g0*t2*t6*t8*t10*t13*6.230758965393622e-3+g0*t3*t7*t9*t10*t12*4.883521085127657e-4-g0*t3*t7*t9*t10*t13*6.230758965393622e-3-g0*t3*t4*t5*t8*t10*t12*4.883521085127657e-4+g0*t3*t4*t5*t8*t10*t13*6.230758965393622e-3;

    return g_hat;
}
