#ifndef HYBRID_PLANNER_H
#define HYBRID_PLANNER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <descartes_moveit/moveit_state_adapter.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_planner/sparse_planner.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/DisplayTrajectory.h>

namespace Hamilton
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  /* ===== utilities ===== */
  typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;
  typedef TrajectoryVec::const_iterator TrajectoryIter; //TODO redundant use auto. 
  enum descartes_planner_type {DENSE, SPARSE};
  enum TrajectorySegmentType {FREE_MOTION, PROCESS_PATH, FREE_MOTION_START_END_ONLY};
  
  /*  =============================== Hybrid Configuration Struct ===============================
  * Is a member of HybridPlanner class and allows to define a few parameters required by MoveIt! and Descartes*/

  struct HybridConfiguration
  {
    std::string group_name;        // = "manipulator";               /* Name of the manipulation group containing the relevant links in the robot, which is what you define when running MoveitSetupAssistant */
    std::string robot_description; // = "robot_description";         /* Name of description on parameter server. Typically just "robot_description". */
    std::string tcp_frame;         // = "tool0";                     /* Tool center point frame (name of link associated with tool. Usually the last link in the kinematic chain of the robot */
    std::string base_link;         //                                /* The name of the base link of the robot */
    std::string world_frame;       // = "base_link" OR "world_frame";/* The name of the world link in the URDF file, the frame in which you are expressing poses. */
                                                                     /* Typically "world_frame" or "base_link". */
    std::vector<std::string> joint_names;                            /* A list with the names of the mobile joints in the robot */ 
                                                                     // 2nd param of toROSJointTrajectory()
    // MoveIt! params
    double plan_time;   // = 10.0;                      
    double step_size;  // = 0.01; 
    double jump_thresh; // =0;
    bool moveit_replan;
    bool avoid_collision_;
    std::string moveit_planner_type_;
    
    // Descartes params
    descartes_planner_type descartes_planner_type_;
    double time_delay; // = 1.0; Last param of toROSJointTrajectory() 
    double time_offset; // = 0.0; Used inside toROSJointTrajectory()
  };

  /*  =============================== TrajectorySegment Struct ===============================
  * Struct to store the unplanned segments, and categorize them as free and process path. 
  * Note: The third type "FREE_MOTION_START_END_ONLY" isn't used right now.
  * The idea is to call the MoveIt planner via a MotionPlanning Request in this case, via the PlanHybrid method.  
  * TODO Add more data structures in TrajectorySegment - joint vals, descartes type itself, etc */

  struct TrajectorySegment
  {
    TrajectorySegmentType type_;      
    std::vector<geometry_msgs::Pose> waypoints_geom_msgs_;
  }; 

  /*  =============================== Hybrid Planner Class ===============================
   * Provides a group of functions for planning a robot path using Moveit and  the Descartes Planning Library */

  class HybridPlanner
  {
    public:
      HybridPlanner();
      virtual ~HybridPlanner();
      // Initialize (descartes) robot model and planners
      void init();

    protected:
      // Append unplanned free space/process path segments defined by "waypoints' to unplanned_trajectory_ as a TrajectorySegment object. 
      // TODO extend to other types - joints, descartes types, etc.
    	void appendFreeMotionTrajectorySegment(std::vector<geometry_msgs::Pose>& waypoints);
    	void appendProcessPathTrajectorySegment(std::vector<geometry_msgs::Pose>& waypoints);

      // Appends unplanned free space segment, defined by start and end point only as a TrajectorySegment object. 
      void appendFreeMotionTrajectorySegment(geometry_msgs::Pose start_pose, geometry_msgs::Pose end_pose);

      // Iterates through the unplanned_trajectory_vector, and calls MoveIt! and Descartes planners as required.
      // Saves the planned, joint space trajectory in overall_robot_traj_, which can be send for execution to the controller.
      // TODO Not Implemented: Makes a motion planning request if the TrajetorySegemnt.type_ == FREE_MOTION_START_END_ONLY
    	void PlanHybrid();

      //====== Helper functions =====//
      /* Create an axial trajectory pt from a given tf transform */
      descartes_core::TrajectoryPtPtr tfToAxialTrajectoryPt(const tf::Transform& nominal, double discretization);  

      // Generates an completely defined (zero-tolerance) cartesian point from a pose
      descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d& pose);

      // Generates a cartesian point with free rotation about the Z axis of the EEF frame
      descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose);

      // Translates a descartes trajectory to a ROS joint trajectory
      trajectory_msgs::JointTrajectory toROSJointTrajectory(const TrajectoryVec& trajectory, const std::vector<std::string>& joint_names, double time_delay);
      //====== Helper functions =====//

      //====== Member Variables =====//
      HybridConfiguration config_;

      // ROS Constructs: Components needed to successfully run a ros-node and perform other important ros-related tasks 
      ros::NodeHandle node_handle_;                        /* Object used for creating and managing ros resources*/

      // Descartes Constructs: Components accessing the semi-constrained path planning capabilities in the Descartes library */
      descartes_core::RobotModelPtr descartes_model_;
      descartes_planner::SparsePlanner descartes_planner_sparse_;      
      descartes_planner::DensePlanner descartes_planner_dense_;

      // MoveIt! Constructs : Components accesing the free-space planning capabilities in the MoveIt! library.
      moveit::planning_interface::MoveGroup moveit_group_;

      // Vector of TrajectorySegment struct to store unplanned segments (either process or free)
      std::vector<TrajectorySegment> unplanned_trajectory_;

      // Final trajectory to be sent for execution 
      // TODO change name to planned_trajectory
      // TODO Make this variable accessible to other classes, so that it can be directly sent for execution.
      robot_trajectory::RobotTrajectory overall_robot_traj_; 

      // TODO not implemnted. Should be removed, as the HybridPlanner class doesn't deal with execution
      ros::ServiceClient moveit_run_path_client_; /* Sends a robot trajectory to moveit for execution */
  };
} /* namespace */
#endif /* HYBRID_PLANNER_H */