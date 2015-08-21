#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <descartes_moveit/moveit_state_adapter.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/DisplayTrajectory.h>

typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;
typedef TrajectoryVec::const_iterator TrajectoryIter;

// Generates an completely defined (zero-tolerance) cartesian point from a pose
descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d& pose);

// Generates a cartesian point with free rotation about the Z axis of the EEF frame

descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose);

// Translates a descartes trajectory to a ROS joint trajectory
trajectory_msgs::JointTrajectory toROSJointTrajectory(const TrajectoryVec& trajectory, const descartes_core::RobotModel& model,                    const std::vector<std::string>& joint_names, double time_delay);

// Sends a ROS trajectory to the robot controller
bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory);

/*
 * In order to store the unplanned segments, and categorize them as free and process
 * TODO Add more data structures in TrajectorySegment - joint vals, descartes type itself, etc
 */
enum TrajectorySegmentType {FREE_MOTION, PROCESS_PATH};
struct TrajectorySegment
{
  TrajectorySegmentType type;      
  std::vector<geometry_msgs::Pose> waypoints_geom_msgs;
};       

// Adds for free space segment to a vector of unplanned TrajectorySegment struct
void appendFreeMotionTrajectorySegment(std::vector<geometry_msgs::Pose>& waypoints, std::vector<TrajectorySegment> unplanned_trajectory);

// Adds for process path points to a vector of unplanned TrajectorySegment struct
// TODO make argument for planner generic => Use pathPlannerBase
void appendProcessPathTrajectorySegment(std::vector<geometry_msgs::Pose>& waypoints, std::vector<TrajectorySegment> unplanned_trajectory);

// Plans the unplanned trajectory, stores the final joint space traj ready to be executed in overall_robot_traj
void PlanHybrid(std::vector<TrajectorySegment> unplanned_trajectory, moveit::planning_interface::MoveGroup& group, descartes_planner::DensePlanner& planner, descartes_core::RobotModelPtr model, robot_trajectory::RobotTrajectory& overall_robot_traj, ros::NodeHandle& nh);

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "descartes_tutorial");
  ros::NodeHandle nh;

  // Required for communication with moveit components
  ros::AsyncSpinner spinner (1);
  spinner.start();

  // Initialize a MoveIt! group
  moveit::planning_interface::MoveGroup group("manipulator");

  // Move to home
  group.setNamedTarget("home");
  group.move();
    
  // Move to target pose
  geometry_msgs::Pose target_pose;
  target_pose.position.x = 0.25;
  target_pose.position.y = 0.25;
  target_pose.position.z = 1.0;
  group.setPoseTarget(target_pose);
  group.move();

  // Create a Descartes robot model and initialize it
  descartes_core::RobotModelPtr model(new descartes_moveit::MoveitStateAdapter);

  // Name of description on parameter server. Typically just "robot_description".
  const std::string robot_description = "robot_description";
  // name of the kinematic group you defined when running MoveitSetupAssistant
  // TODO Fermi has an issue of hardcoded manipulator. Avoid in Hamilton
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

  // Initialize a Descartes planner with the Descartes Model 
  descartes_planner::DensePlanner planner;
  planner.initialize(model);

  // The overall_robot_traj is the final, hybrid trajectory that is meant to be executed adter  
  robot_trajectory::RobotTrajectory overall_robot_traj(group.getCurrentState()->getRobotModel(), "manipulator"); //TODO group.getRobotModel

  // Vector of TrajectorySegment to store unplanned segments(either process or free)
  std::vector<TrajectorySegment> unplanned_trajectory;

  // Generate some MoveIt | free space waypoints
  std::vector<geometry_msgs::Pose> waypoints;
  for(int i = 0; i < 8; ++i)
  {
    target_pose.position.z += 0.05;
    waypoints.push_back(target_pose); 
  }
  appendFreeMotionTrajectorySegment(waypoints, unplanned_trajectory);

  // Generate some Descartes | semi-constrained waypoints
  std::vector<geometry_msgs::Pose> waypoints_2;
  for (int i = 0; i <= 5; ++i)
  {
    target_pose.position.z = 1.4 - 0.05*i;
    waypoints_2.push_back(target_pose);
  }
  appendProcessPathTrajectorySegment(waypoints_2, unplanned_trajectory);

 /* // Generate some MoveIt | free space waypoints
  std::vector<geometry_msgs::Pose> waypoints_2;
  for(int i = 0; i < 20; ++i)
  {
    target_pose.position.z = 0.9;
    target_pose.position.y -= 0.05;
    waypoints_2.push_back(target_pose); 
  }
  addFreeSpaceSegment(waypoints_2, group, overall_robot_traj);

  TrajectoryVec points_2;
  for (unsigned int i = 0; i < 20; ++i)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(0.25, -0.75 + 0.02*i, 0.9 + 0.02*i);
    descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pose);
    points_2.push_back(pt);
  }
  addProcessSegment(points_2, planner, group, model, overall_robot_traj, nh);*/

  //TOMIGHTDO http://docs.ros.org/hydro/api/moveit_core/html/classrobot__trajectory_1_1RobotTrajectory.html
  //This API could be improved by allowing for trajectory_msgs::JointTrajectory &trajectory argument in void getRobotTrajectoryMsg 
  //Coz for setRobotTrajectory msg, you could pass trajectory_msgs::JointTrajectory or moveit_msgs::RobotTrajectory as done for Descartes above

  PlanHybrid(unplanned_trajectory, group, planner, model, overall_robot_traj, nh);

  // TODO Following 3 lines could be taken care of by PlanHybrid method 
  moveit_msgs::RobotTrajectory combined;
  overall_robot_traj.getRobotTrajectoryMsg(combined);
  trajectory_msgs::JointTrajectory joints_combined = combined.joint_trajectory;
  
  // Send the ROS trajectory to the robot for execution
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

void appendFreeMotionTrajectorySegment(std::vector<geometry_msgs::Pose>& waypoints, std::vector<TrajectorySegment> unplanned_trajectory)
{
  TrajectorySegment current_free_segment;
  current_free_segment.type = TrajectorySegmentType::FREE_MOTION;
  current_free_segment.waypoints_geom_msgs = waypoints;
  unplanned_trajectory.push_back(current_free_segment);              
}

void appendProcessPathTrajectorySegment(std::vector<geometry_msgs::Pose>& waypoints, std::vector<TrajectorySegment> unplanned_trajectory)
{
  TrajectorySegment current_process_segment;
  current_process_segment.type = TrajectorySegmentType::PROCESS_PATH;
  current_process_segment.waypoints_geom_msgs = waypoints;
  unplanned_trajectory.push_back(current_process_segment);              
}

void PlanHybrid(std::vector<TrajectorySegment> unplanned_trajectory, moveit::planning_interface::MoveGroup& group, descartes_planner::DensePlanner& planner, descartes_core::RobotModelPtr model, robot_trajectory::RobotTrajectory& overall_robot_traj, ros::NodeHandle& nh)
{
  for(auto it = unplanned_trajectory.begin(); it != unplanned_trajectory.end(); it++) 
  {
    // Free segment planning
    if(it->type == FREE_MOTION) 
    {       
      moveit_msgs::RobotTrajectory moveit_robot_traj;
      group.setPlanningTime(10.0); 
      std::vector<geometry_msgs::Pose> current_free_segment = it->waypoints_geom_msgs; //the current key of the std::multimap, which is geometry_msgs::Pose object
      double fraction = group.computeCartesianPath(current_free_segment,
                                                 0.01,  // eef_step
                                                 0,   // jump_threshold
                                                 moveit_robot_traj); 

      robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "manipulator");
      rt.setRobotTrajectoryMsg(*group.getCurrentState(), moveit_robot_traj);
      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      bool success = iptp.computeTimeStamps(rt);
      ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

      //Appending planned path
      overall_robot_traj.append(rt, rt.getWaypointDurationFromStart(rt.getWayPointCount()));
      ROS_INFO("Appended Free space segment");
    }   

    // Process segment planning
    if(it->type == PROCESS_PATH)
    {
      using namespace descartes_core;
      using namespace descartes_trajectory; 

      // Container for the current segment
      TrajectoryVec process_segment_descartes; 
     
      // Convert from geometry_msgs::Pose to Descartes point type
      for(auto segment_iterator = it->waypoints_geom_msgs.begin(); segment_iterator!= it->waypoints_geom_msgs.end(); segment_iterator++)
      {
        Eigen::Affine3d pose;
        pose = Eigen::Translation3d(segment_iterator->position.x, segment_iterator->position.y, segment_iterator->position.z); 
        descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pose);
        process_segment_descartes.push_back(pt);
      }

      // TODEBUG TODO For the segfault, the following block could be a cause.. But commenting it out still caused it, apparently with a diff error(no "empty traj sent for execution" if this block is commented out)

      // Append the first point of the next segment (if the current segment is not the last segment itself) 
      if(it != unplanned_trajectory.end())
      {
        std::advance(it, 1);
        auto first_point_of_next_segment = it->waypoints_geom_msgs.begin();
        Eigen::Affine3d first_point_of_next_segment_eigen;
        first_point_of_next_segment_eigen = Eigen::Translation3d(first_point_of_next_segment->position.x, first_point_of_next_segment->position.y, first_point_of_next_segment->position.z);
        descartes_core::TrajectoryPtPtr first_point_of_next_segment_descartes = makeTolerancedCartesianPoint(first_point_of_next_segment_eigen);
        process_segment_descartes.push_back(first_point_of_next_segment_descartes);
        std::advance(it, -1); //let the hybrid iterator come back to the current segment
      }

      // Prepend the first point of the next segment (if the current segment is not the first segment itself) 
      if(it != unplanned_trajectory.begin())
      { 
        std::advance(it, -1);
        auto last_point_of_previous_segment = it->waypoints_geom_msgs.begin();
        Eigen::Affine3d last_point_of_previous_segment_eigen;
        last_point_of_previous_segment_eigen = Eigen::Translation3d(last_point_of_previous_segment->position.x, last_point_of_previous_segment->position.y, last_point_of_previous_segment->position.z);
        descartes_core::TrajectoryPtPtr last_point_of_previous_segment_descartes = makeTolerancedCartesianPoint(last_point_of_previous_segment_eigen);
        process_segment_descartes.insert(process_segment_descartes.begin(), last_point_of_previous_segment_descartes);
        std::advance(it, 1); //let the hybrid iterator come back to the current segment
      }

      // Call the Descartes planner to do some planning
      if (!planner.planPath(process_segment_descartes))
      {
        ROS_ERROR("Could not solve for a valid path");
      }

      TrajectoryVec result;
      if (!planner.getPath(result))
      {
        ROS_ERROR("Could not retrieve path");
      }

      // Get Joint Names
      std::vector<std::string> names;
      nh.getParam("controller_joint_names", names);
      // Generate a ROS joint trajectory with the result path, robot model, given joint names,
      // a certain time delta between each trajectory point
      trajectory_msgs::JointTrajectory descartes_joint_solution = toROSJointTrajectory(result, *model, names, 1.0);

      //Append planned path
      robot_trajectory::RobotTrajectory descartes_robot_traj(group.getCurrentState()->getRobotModel(), "manipulator"); //TODO group.getRobotModel does the same thing. 
      descartes_robot_traj.setRobotTrajectoryMsg(*group.getCurrentState(), descartes_joint_solution);
      overall_robot_traj.append(descartes_robot_traj, descartes_robot_traj.getWaypointDurationFromStart(descartes_robot_traj.getWayPointCount())); 
      ROS_INFO("Appended process segment");
    }
  }
}

