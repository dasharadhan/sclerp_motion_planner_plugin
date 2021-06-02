/* Author: Dasharadhan Mahalingam */

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_scene/planning_scene.h>
#include <class_loader/class_loader.hpp>
#include "sclerp_interface/sclerp_planning_context.h"

namespace sclerp_interface
{

class ScLERPPlannerManager : public planning_interface::PlannerManager
{
  public:
    ScLERPPlannerManager() : planning_interface::PlannerManager()
    {

    }

    bool initialize(const moveit::core::RobotModelConstPtr& model,
                    const std::string & /*ns*/) override
    {
      for(const std::string &gpName : model->getJointModelGroupNames())
      {
        std::cout << "Group name  : " << gpName << std::endl
                  << "Robot model : " << model->getName() << std::endl;

        planning_contexts_[gpName] =
            ScLERPPlanningContextPtr(
                new ScLERPPlanningContext("sclerp_planning_context",
                                          gpName,
                                          model));
      }

      return true;
    }

    bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const
      override
    {
      return req.trajectory_constraints.constraints.empty();
    }

    std::string getDescription() const override
    {
      return "ScLERP";
    }

    void getPlanningAlgorithms(std::vector<std::string> &algs) const override
    {
      algs.clear();
      algs.push_back("ScLERP");
    }

    planning_interface::PlanningContextPtr getPlanningContext(
        const planning_scene::PlanningSceneConstPtr &planning_scene,
        const planning_interface::MotionPlanRequest &req,
        moveit_msgs::MoveItErrorCodes &error_code) const override
    {
      error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

      if(req.group_name.empty())
      {
        ROS_ERROR("No group specified to plan for");
        error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
        return planning_interface::PlanningContextPtr();
      }

      if(!planning_scene)
      {
        ROS_ERROR("No planning scene supplied as input");
        error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return planning_interface::PlanningContextPtr();
      }

      planning_scene::PlanningScenePtr ps = planning_scene->diff();

      const ScLERPPlanningContextPtr &context = 
          planning_contexts_.at(req.group_name);

      ROS_INFO_STREAM_NAMED("sclerp_planner_manager", "===>>> context is made");

      context->setPlanningScene(ps);
      context->setMotionPlanRequest(req);

      error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

      return context;
    }

  protected:
    std::map<std::string, ScLERPPlanningContextPtr> planning_contexts_;
};

} // namespace sclerp_interface

// Register the ScLERPPlannerManager class as a plugin
CLASS_LOADER_REGISTER_CLASS(sclerp_interface::ScLERPPlannerManager,
                            planning_interface::PlannerManager);
