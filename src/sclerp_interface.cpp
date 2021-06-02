/* Author: Dasharadhan Mahalingam */

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/MotionPlanRequest.h>

#include <ros/ros.h>

#include <moveit/rdf_loader/rdf_loader.h>
#include <urdf/model.h>
#include <srdfdom/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include <limits>
#include <vector>
#include <Eigen/Geometry>
#include <unordered_map>

#include "sclerp_interface/sclerp_interface.h"

#if DEBUG

#include "eigen_matrix_formatting.h"
#include <fstream>
#include <signal.h>
#include <ctime>
#include <sys/stat.h>
#include <errno.h>

#include <rosbag/bag.h>

namespace sclerp_interface
{

Eigen::IOFormat CSVFormat(Eigen::FullPrecision,0,",","\n","","","","");

std::string log_folder_name = "sclerp_interface";
std::string home_dir(getenv("HOME"));
std::string log_dir = home_dir + "/logs/" + log_folder_name;

std::string timestamp_str = "Y%YM%mD%d_T%H%M%S";

bool log_folder_error = false;
bool log_file_error = false;

rosbag::Bag bag;

}

#endif

namespace sclerp_interface
{

ScLERPInterface::ScLERPInterface(const ros::NodeHandle &nh)
  : nh_(nh), name_("ScLERPInterface")
{

}

bool ScLERPInterface::solve(
        const planning_scene::PlanningSceneConstPtr &planning_scene,
        const planning_interface::MotionPlanRequest &req,
        moveit_msgs::MotionPlanDetailedResponse &res)
{
  ros::WallTime start_time = ros::WallTime::now();

  moveit::core::RobotModelConstPtr 
    robot_model = planning_scene->getRobotModel();

  moveit::core::RobotStatePtr 
    start_state(new moveit::core::RobotState(robot_model));
  *start_state = planning_scene->getCurrentState();

  sensor_msgs::JointState start_joint_state = req.start_state.joint_state;

  const moveit::core::JointModelGroup 
    *joint_model_group = start_state->getJointModelGroup(req.group_name);

  std::vector<double> start_joint_values;
  start_state->copyJointGroupPositions(joint_model_group, start_joint_values);

  Eigen::VectorXd init_jnt_values(start_joint_values.size());
  Eigen::VectorXd goal_jnt_values(start_joint_values.size());
  
  Eigen::Matrix4d g_goal(Eigen::Matrix4d::Identity());
  
  bool plan_using_joint_constraints = false;

  moveit_msgs::Constraints goal_constraints = req.goal_constraints[0];

  if( (goal_constraints.orientation_constraints.size() != 0) && 
      (goal_constraints.position_constraints.size() != 0) )
  {
    ROS_INFO("Using end-effector goal pose constraint for planning");
    
    moveit_msgs::OrientationConstraint goal_orientation_constraint =
      goal_constraints.orientation_constraints[0];

    Eigen::Quaterniond goal_orientation(
        goal_orientation_constraint.orientation.w,
        goal_orientation_constraint.orientation.x,
        goal_orientation_constraint.orientation.y,
        goal_orientation_constraint.orientation.z);
    
    goal_orientation.normalize();

    g_goal.block<3,3>(0,0) = goal_orientation.toRotationMatrix();
    g_goal(0,3) = goal_constraints.position_constraints[0].target_point_offset.x;
    g_goal(1,3) = goal_constraints.position_constraints[0].target_point_offset.y;
    g_goal(2,3) = goal_constraints.position_constraints[0].target_point_offset.z;
  }
  else if(goal_constraints.joint_constraints.size() != 0)
  {
    ROS_INFO("Using goal joint constraints for planning");
    
    plan_using_joint_constraints = true;
    
    for(int jnt_itr = 0; jnt_itr < start_joint_values.size(); jnt_itr++)
    {
      goal_jnt_values(jnt_itr) = goal_constraints.joint_constraints[jnt_itr].position;
    }
  }
  else
  {
    ROS_ERROR("No goal state provided!");
    return false;
  }

  const std::vector<const moveit::core::LinkModel *> link_model =
    joint_model_group->getLinkModels();

  const std::vector<std::string> joint_model_group_variable_names = 
    joint_model_group->getVariableNames();

  // Set initial state from request
  for(int i = 0; i < start_joint_values.size(); i++)
  {
    int j = 0;

    for(j = 0; j < start_joint_state.name.size(); j++)
    {
      if(start_joint_state.name[j] == joint_model_group_variable_names[i])
      {
        start_joint_values[i] = start_joint_state.position[j];
        break;
      }
    }
  }

  for(int i = 0; i < start_joint_values.size(); i++)
  {
    init_jnt_values(i) = start_joint_values[i];
  }

  std::string base_joint_name = joint_model_group_variable_names[0];

  std::string base_link_name = link_model.front()->getName();
  std::string tip_link_name = link_model.back()->getName();

  // Determine root link for the given joint group
  const moveit::core::LinkModel *base_link_parent = 
    link_model.front()->getParentLinkModel();

  std::string root_link_name = base_link_parent->getName();

  std::vector<const moveit::core::JointModel * > child_joint_models =
    base_link_parent->getChildJointModels();

  bool root_link_found = false;

  for(int i = 0; i < child_joint_models.size(); i++)
  {
    if(base_joint_name == child_joint_models[i]->getName())
    {
      root_link_found = true;
      break;
    }
  }

  if(!root_link_found)
  {
    int link_itr = 0;

    while(link_itr < link_model.size())
    {
      child_joint_models = link_model[link_itr]->getChildJointModels();

      for(int i = 0; i < child_joint_models.size(); i++)
      {
        if(base_joint_name == child_joint_models[i]->getName())
        {
          root_link_name = link_model[link_itr]->getName();
          root_link_found = true;
          break;
        }
      }

      if(root_link_found)
      {
        break;
      }
    }
  }

  const Eigen::Affine3d w_root = start_state->getFrameTransform(root_link_name);

  std::vector<std::string> joint_names = joint_model_group->getVariableNames();

  ROS_DEBUG("Reading robot model URDF from Parameter Server");
  // Read URDF model from parameter server
  urdf::Model urdf_model;
  if(!urdf_model.initParam("/robot_description"))
  {
    ROS_ERROR("Failed to read URDF file from Parameter Server");
    return false;
  }

  // Get joint list from URDF
  std::map<std::string, urdf::JointSharedPtr> jnt_list = urdf_model.joints_;

  // Construct KDL tree for robot
  KDL::Tree robot_tree;
  std::string robot_desc_string;
  nh_.param("/robot_description", robot_desc_string, std::string());
  if(!kdl_parser::treeFromString(robot_desc_string, robot_tree))
  {
    ROS_ERROR("Failed to construct KDL tree");
    return false;
  }

  ROS_DEBUG_STREAM("JointModel first link name : " << base_link_name.c_str());
  ROS_DEBUG_STREAM("JointModel last link name  : " << tip_link_name.c_str());
  ROS_DEBUG_STREAM("JointModel root link name  : " << root_link_name.c_str());

  // Construct KDL chain for joint model group
  KDL::Chain manip_kdl_chain;
  //if(!robot_tree.getChain(base_link_name, tip_link_name, manip_kdl_chain))
  if(!robot_tree.getChain(root_link_name, tip_link_name, manip_kdl_chain))
  {
    ROS_ERROR("Failed to construct KDL chain from tree");
    return false;
  }

  // Transform between root frame and current link reference frame
  Eigen::Matrix4d t_ref(Eigen::Matrix4d::Identity());
  // Transform from current link reference frame to joint in current link
  Eigen::Matrix4d t_jnt(Eigen::Matrix4d::Identity());
  // Transform from current link reference frame to tip of current link
  Eigen::Matrix4d t_tip(Eigen::Matrix4d::Identity());

  // Assign value based on root frame (Also considering virtual joints)
  t_ref = w_root.matrix();

  // Position Vector of origin of joint wrt current link reference frame
  Eigen::Vector4d jnt_pos_vec;
  // Direction of joint axis wrt current link reference frame
  Eigen::Vector4d jnt_axis_vec;

  jnt_pos_vec << 0, 0, 0, 1;
  jnt_axis_vec << 0, 0, 0, 0;

  KDL::Frame frame_to_tip;

  kinlib::Manipulator manip;
  kinlib::JointType jnt_type;
  kinlib::JointLimits jnt_lim;

  ROS_DEBUG_STREAM("Number of Segments in KDL Chain : " << manip_kdl_chain.getNrOfSegments());

  for(int itr = 0; itr < manip_kdl_chain.getNrOfSegments(); itr++)
  {
    KDL::Segment chain_seg = manip_kdl_chain.getSegment(itr);
    std::string seg_name = chain_seg.getName();
    KDL::Joint jnt = chain_seg.getJoint();

    frame_to_tip = chain_seg.getFrameToTip();

    urdf::JointSharedPtr jnt_ptr(jnt_list[jnt.getName()]);

    for(int row_itr = 0; row_itr < 3; row_itr++)
    {
      for(int col_itr = 0; col_itr < 3; col_itr++)
      {
        t_tip(row_itr, col_itr) = frame_to_tip.M.data[(row_itr*3)+col_itr];
      }
      
      t_tip(row_itr, 3) = frame_to_tip.p.data[row_itr];

      jnt_pos_vec(row_itr) = jnt.JointOrigin().data[row_itr];
      jnt_axis_vec(row_itr) = jnt.JointAxis().data[row_itr];
    }

    // Transforming to root frame
    Eigen::Vector4d w_jnt_pos_vec = t_ref * jnt_pos_vec;
    Eigen::Vector4d w_jnt_axis_vec = t_ref * jnt_axis_vec;

    t_ref = t_ref * t_tip;

    ROS_DEBUG_STREAM("Joint type : " << jnt.getTypeName().c_str());
    ROS_DEBUG_STREAM("Joint name : " << jnt.getName().c_str());

    if(jnt.getType() == KDL::Joint::JointType::RotAxis)
    {
      jnt_lim.upper_limit_ = jnt_ptr->limits->upper;
      jnt_lim.lower_limit_ = jnt_ptr->limits->lower;

      jnt_type = kinlib::JointType::Revolute;

      manip.addJoint( jnt_type, jnt.getName(), w_jnt_axis_vec,
                      w_jnt_pos_vec, jnt_lim, t_ref);

    }
    else
    {
      manip.modifyEndJointTipPose(t_ref);
    }
  }

  //kinlib_solver_.setManipulator(manip);
  kinlib_solver_ = kinlib::KinematicsSolver(manip);

  Eigen::Matrix4d g_init;

  kinlib_solver_.getFK(init_jnt_values, g_init);
  
  if(plan_using_joint_constraints)
  {
    kinlib_solver_.getFK(goal_jnt_values, g_goal);
  }

  trajectory_msgs::JointTrajectory joint_trajectory;

#if DEBUG

  std::time_t now = std::time(0);
  std::tm* timestamp = std::localtime(&now);

  char timestamp_char[100];
  strftime(timestamp_char, 100, timestamp_str.c_str(), timestamp);

  std::string current_log_folder(timestamp_char);
  std::string log_dir_path = log_dir + "/" + current_log_folder;

  std::string timestamp_str_val(timestamp_char);
  std::string log_file_path = log_dir_path + "/" + timestamp_str_val
                            + "_request.csv";

  std::ofstream log_file;

  //raise(SIGTRAP);

  if(mkdir(log_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
  {
    if(errno != EEXIST)
    {
      std::cout << "Error : Cannot create log folders\n";
      std::cout << "Error " << errno << ": " << strerror(errno);
      std::cout.flush();
      log_folder_error = true;
    }
  }
  
  if(!log_folder_error)
  {
    if(mkdir(log_dir_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
    {
      if(errno != EEXIST)
      {
        std::cout << "Error : Cannot create log folders\n";
        std::cout << "Error " << errno << ": " << strerror(errno);
        std::cout.flush();
        log_folder_error = true;
      }
    }
    
    if(!log_folder_error)
    {
      bag.open(log_dir_path + "/bagfile.bag", rosbag::bagmode::Write);

      //bag.write("planning_scene", ros::Time::now(), planning_scene);
      bag.write("request", ros::Time::now(), req);

      log_file.open(log_file_path, std::ofstream::out | std::ofstream::app);

      if(!log_file.is_open())
      {
        log_file_error = true;

        std::cout << "Error : Cannot open log file\n";
        std::cout.flush();
      }
      else
      {
        log_file << init_jnt_values.transpose().format(CSVFormat) << '\n'
                 << goal_jnt_values.transpose().format(CSVFormat) << '\n';
        log_file.flush();
      }
    }
  }

#endif

  kinlib::ErrorCodes plan_result = kinlib_solver_.getMotionPlan(
                                      init_jnt_values,
                                      g_init,
                                      g_goal,
                                      joint_trajectory);

  res.processing_time.push_back((ros::WallTime::now() - start_time).toSec());

  res.group_name = req.group_name;
  //res.trajectory_start.joint_state.name = joint_names;
  //res.trajectory_start.joint_state.position = start_joint_values;
  
  res.trajectory_start = req.start_state;

  if(plan_result == kinlib::ErrorCodes::JOINT_LIMIT_ERROR)
  {
    ROS_INFO("One or more joints have reached their joint limits!");
  }

  if( plan_result == kinlib::ErrorCodes::OPERATION_SUCCESS ||
      plan_result == kinlib::ErrorCodes::JOINT_LIMIT_ERROR)
  {
    res.trajectory.resize(1);

    //res.trajectory[0].joint_trajectory = joint_trajectory;

    res.trajectory[0].joint_trajectory.joint_names = joint_names;
    res.trajectory[0].joint_trajectory.header = 
      req.start_state.joint_state.header;

    res.trajectory[0].joint_trajectory.points = joint_trajectory.points;

    ROS_INFO_STREAM("Trajectory Length : " << joint_trajectory.points.size());


    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

#if DEBUG
    if(!log_folder_error)
    {
      bag.write("response", ros::Time::now(), res);
      bag.close();
    }
#endif

    return true;
  }
  else
  {
    ROS_ERROR_STREAM("ScLERP Motion Planner Failed!");

#if DEBUG
    if(!log_folder_error)
    {
      bag.write("reponse", ros::Time::now(), res);
      bag.close();
    }
#endif

    res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;

    return false;
  }

}

} // namespace sclerp_interface
