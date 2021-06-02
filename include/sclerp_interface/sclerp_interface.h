/* Author: Dasharadhan Mahalingam */

#pragma once

#include <ros/ros.h>
#include <moveit/planning_interface/planning_interface.h>

#include "kinlib/kinlib_kinematics.h"

namespace sclerp_interface
{

MOVEIT_CLASS_FORWARD(ScLERPInterface);

class ScLERPInterface
{
  public:
    ScLERPInterface(const ros::NodeHandle &nh = ros::NodeHandle("~"));

    bool solve( const planning_scene::PlanningSceneConstPtr &planning_scene,
                const planning_interface::MotionPlanRequest &req, 
                moveit_msgs::MotionPlanDetailedResponse &res);

  protected:
    ros::NodeHandle nh_;
    std::string name_;

    kinlib::KinematicsSolver kinlib_solver_;

  private:
    /*
    void interpolate( const std::vector<std::string> &joint_names, 
                      moveit::core::RobotStatePtr &robot_state,
                      const moveit::core::JointModelGroup *joint_model_group, 
                      const std::vector<double> &start_joint_vals,
                      const std::vector<double> &goal_joint_vals, 
                      trajectory_msgs::JointTrajectory &joint_trajectory);
    */
};

} // namespace sclerp_interface
