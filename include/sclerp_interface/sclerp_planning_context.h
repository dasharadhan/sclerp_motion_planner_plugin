/* Author: Dasharadhan Mahalingam */

#pragma once

#include <moveit/planning_interface/planning_interface.h>

#include "sclerp_interface/sclerp_interface.h"

namespace sclerp_interface
{

MOVEIT_CLASS_FORWARD(ScLERPPlanningContext);

class ScLERPPlanningContext : public planning_interface::PlanningContext
{
  public:
    ScLERPPlanningContext(const std::string &context_name,
                          const std::string &group_name,
                          const moveit::core::RobotModelConstPtr &model);

    ~ScLERPPlanningContext() override
    {

    }

    bool solve(planning_interface::MotionPlanResponse &res) override;
    bool solve(planning_interface::MotionPlanDetailedResponse &res) override;

    bool terminate() override;
    void clear() override;

  private:
    moveit::core::RobotModelConstPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    ScLERPInterfacePtr sclerp_interface_;
};

} // namespace sclerp_interface
