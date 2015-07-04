// Core ros functionality like ros::init and spin
#include <ros/ros.h>
// ROS Trajectory Action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>

// Includes the descartes robot model we will be using
#include <descartes_moveit/moveit_state_adapter.h>
// Includes the descartes trajectory type we will be using
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
// Includes the planner we will be using
#include <descartes_planner/dense_planner.h>
// #include <boost/function.hpp>
// #include <boost/assign/list_of.hpp>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
// #include <moveit_msgs/PlanningScene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
// #include <geometry_msgs/PoseArray.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <eigen_conversions/eigen_msg.h>


typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;
typedef TrajectoryVec::const_iterator TrajectoryIter;

/**
 * Generates an completely defined (zero-tolerance) cartesian point from a pose
 */
descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d& pose);
/**
 * Generates a cartesian point with free rotation about the Z axis of the EFF frame
 */
descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose);

/**
 * Translates a descartes trajectory to a ROS joint trajectory
 */
trajectory_msgs::JointTrajectory
toROSJointTrajectory(const TrajectoryVec& trajectory, const descartes_core::RobotModel& model,
                     const std::vector<std::string>& joint_names, double time_delay);

/**
 * Sends a ROS trajectory to the robot controller
 */
bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory);

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "descartes_tutorial");
  ros::NodeHandle nh;

  // Required for communication with moveit components
  ros::AsyncSpinner spinner (1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("manipulator");
  // ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  // moveit_msgs::DisplayTrajectory display_trajectory;
  moveit::planning_interface::MoveGroup::Plan plan;
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose target_pose;
  group.setNamedTarget("home");
  group.move();
  
  target_pose.position.x = 0.25;
  target_pose.position.y = 0.25;
  target_pose.position.z = 1.0;
  group.setPoseTarget(target_pose);
  group.move();
  for(int i = 0; i < 8; ++i)
  {
    // target_pose.orientation.w = 1.0; //try full quaternion
    target_pose.position.x = 0.25;
    target_pose.position.y = 0.25;
    target_pose.position.z = 1.0 + 0.05*i;
    waypoints.push_back(target_pose); 
  }

  //   for(int i = 0; i < 10; ++i)
  // {
  //   // target_pose.orientation.w = 1.0; //try full quaternion
  //   // target_pose.position.x = 0;
  //   // target_pose.position.y = 0.4;
  //   target_pose.position.z = 1.0 + 0.05*i;
  //   waypoints.push_back(target_pose); 
  // }

  // 1. Define sequence of points
  TrajectoryVec points;
  for (unsigned int i = 0; i < 10; ++i)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(0.25, 0.25, 1.4 - 0.05 * i);
    descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pose);
    points.push_back(pt);
  }

  // for (unsigned int i = 0; i < 5; ++i)
  // {
  //   Eigen::Affine3d pose;
  //   pose = Eigen::Translation3d(0.0, 0.04 * i, 1.3);
  //   descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pose);
  //   points.push_back(pt);
  // }

  // 2. Create a robot model and initialize it
  descartes_core::RobotModelPtr model (new descartes_moveit::MoveitStateAdapter);

  // Name of description on parameter server. Typically just "robot_description".
  const std::string robot_description = "robot_description";

  // name of the kinematic group you defined when running MoveitSetupAssistant
  const std::string group_name = "manipulator";

  // Name of frame in which you are expressing poses. Typically "world_frame" or "base_link".
  const std::string world_frame = "base_link";

  // tool center point frame (name of link associated with tool)
  const std::string tcp_frame = "tool0";

  if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
  {
    ROS_INFO("Could not initialize robot model");
    return -1;
  }

  moveit_msgs::RobotTrajectory moveit_robot_traj;
  group.setPlanningTime(10.0); 

  double fraction = group.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0,   // jump_threshold
                                               moveit_robot_traj); 

  robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "manipulator");
  rt.setRobotTrajectoryMsg(*group.getCurrentState(), moveit_robot_traj);
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  bool success = iptp.computeTimeStamps(rt);
  ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
  rt.getRobotTrajectoryMsg(moveit_robot_traj);
  ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",fraction * 100.0); 

  plan.trajectory_ = moveit_robot_traj;
  // group.execute(plan);

  // display_trajectory.trajectory_start = plan.start_state_;
  // display_trajectory.trajectory.push_back(plan.trajectory_);
  // display_publisher.publish(display_trajectory);


  // 3. Create a planner and initialize it with our robot model
  descartes_planner::DensePlanner planner;
  planner.initialize(model);

  // 4. Feed the trajectory to the planner
  if (!planner.planPath(points))
  {
    ROS_ERROR("Could not solve for a valid path");
    return -2;
  }

  TrajectoryVec result;
  if (!planner.getPath(result))
  {
    ROS_ERROR("Could not retrieve path");
    return -3;
  }

  // 5. Translate the result into a type that ROS understands
  // Get Joint Names
  std::vector<std::string> names;
  nh.getParam("controller_joint_names", names);
  // Generate a ROS joint trajectory with the result path, robot model, given joint names,
  // a certain time delta between each trajectory point
  trajectory_msgs::JointTrajectory descartes_joint_solution = toROSJointTrajectory(result, *model, names, 1.0);

  trajectory_msgs::JointTrajectory moveit_joint_traj = moveit_robot_traj.joint_trajectory;
  robot_trajectory::RobotTrajectory overall_robot_traj(group.getCurrentState()->getRobotModel(), "manipulator");
  robot_trajectory::RobotTrajectory descartes_robot_traj(group.getCurrentState()->getRobotModel(), "manipulator");
  overall_robot_traj.setRobotTrajectoryMsg(*group.getCurrentState(), moveit_robot_traj);
  descartes_robot_traj.setRobotTrajectoryMsg(*group.getCurrentState(), descartes_joint_solution);
  overall_robot_traj.append(descartes_robot_traj, descartes_robot_traj.getWaypointDurationFromStart(descartes_robot_traj.getWayPointCount()));
  moveit_msgs::RobotTrajectory combined;
  overall_robot_traj.getRobotTrajectoryMsg(combined);
  //TOMIGHTDO http://docs.ros.org/hydro/api/moveit_core/html/classrobot__trajectory_1_1RobotTrajectory.html
  //This API could be improved by allowing for trajectory_msgs::JointTrajectory &trajectory argument in void getRobotTrajectoryMsg 
  //Coz for setRobotTrajectory msg, you could pass trajectory_msgs::JointTrajectory or moveit_msgs::RobotTrajectory as done for Descartes above
  trajectory_msgs::JointTrajectory joints_combined = combined.joint_trajectory;

  // 6. Send the ROS trajectory to the robot for execution
  // if (!executeTrajectory(moveit_joint_traj))
  // {
  //   ROS_ERROR("Could not execute trajectory!");
  //   return -4;
  // }
  // if (!executeTrajectory(descartes_joint_solution))
  // {
  //   ROS_ERROR("Could not execute trajectory!");
  //   return -4;
  // }
  if (!executeTrajectory(joints_combined))
  {
    ROS_ERROR("Could not execute trajectory!");
    return -4;
  }
  // Wait till user kills the process (Control-C)
  ROS_INFO("Done!");
  return 0;
}

descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d& pose)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  return TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose)) );
}

descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;
  return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI/2.0-0.0001, AxialSymmetricPt::Z_AXIS) );
}

trajectory_msgs::JointTrajectory
toROSJointTrajectory(const TrajectoryVec& trajectory,
                     const descartes_core::RobotModel& model,
                     const std::vector<std::string>& joint_names,
                     double time_delay)
{
  // Fill out information about our trajectory
  trajectory_msgs::JointTrajectory result;
  result.header.stamp = ros::Time::now();
  result.header.frame_id = "world_frame";
  result.joint_names = joint_names;

  // For keeping track of time-so-far in the trajectory
  double time_offset = 0.0;
  // Loop through the trajectory
  for (TrajectoryIter it = trajectory.begin(); it != trajectory.end(); ++it)
  {
    // Find nominal joint solution at this point
    std::vector<double> joints;
    it->get()->getNominalJointPose(std::vector<double>(), model, joints);

    // Fill out a ROS trajectory point
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions = joints;
    // velocity, acceleration, and effort are given dummy values
    // we'll let the controller figure them out
    pt.velocities.resize(joints.size(), 0.0);
    pt.accelerations.resize(joints.size(), 0.0);
    pt.effort.resize(joints.size(), 0.0);
    // set the time into the trajectory
    pt.time_from_start = ros::Duration(time_offset);
    // increment time
    time_offset += time_delay;

    result.points.push_back(pt);
  }

  return result;
}

bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
  // Create a Follow Joint Trajectory Action Client
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("joint_trajectory_action", true);
  if (!ac.waitForServer(ros::Duration(2.0)))
  {
    ROS_ERROR("Could not connect to action server");
    return false;
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  goal.goal_time_tolerance = ros::Duration(1.0);
  
  ac.sendGoal(goal);

  if (ac.waitForResult(goal.trajectory.points[goal.trajectory.points.size()-1].time_from_start + ros::Duration(5)))
  {
    ROS_INFO("Action server reported successful execution");
    return true;
  } else {
    ROS_WARN("Action server could not execute trajectory");
    return false;
  }
}