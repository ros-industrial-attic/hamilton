#include "hamilton/hybrid_planner.h"

namespace Hamilton
{   
  HybridPlanner::~HybridPlanner()
  {
    //)WHAT)TODO. Reset Descartes robot model ptr? What about MoveIt? 
  }

  void HybridPlanner::init()
  {    
    // Set the group name of HybridPlanner's MoveIt! group member in accordance to its config_ struct
    moveit_group_.Options.group_name_ = config_.GROUP_NAME;

    // Move to home
    moveit_group_.setNamedTarget("home");
    moveit_group_.move();
    
    // Initialize a Descartes robot model, MoveIt group is already initialized to config_.GROUP_NAME in HybridPlanner class // TOCHECK here v/s in class definition itself
    if (!descartes_model_->initialize(config_.ROBOT_DESCRIPTION, config_.GROUP_NAME, config_.WORLD_FRAME, config_.TCP_FRAME))
    {
      ROS_WARN("Could not initialize Descartes robot model");
    }

    // Initialize a Descartes planner with the Descartes Model 
    if(config_.descartes_planner_type_ == DENSE)
    {
      descartes_planner_dense_.initialize(descartes_model_);
      ROS_INFO("Initialized Descartes dense planner");
    }
    else if(config_.descartes_planner_type_ == SPARSE)
    {
      descartes_planner_sparse_.initialize(descartes_model_);
      ROS_INFO("Initialized Descartes sparse planner");
    }
    else
    {
      ROS_WARN("Descartes planner not initialized. Errors will occur if process path segments are being planned for");
    }

    // The overall_robot_traj_ is the final, hybrid trajectory that is meant to be executed.  
    overall_robot_traj_.setGroupName(config_.GROUP_NAME); 
    //TODO How to implement the OOP equivalent of following line?
    // overall_robot_traj_(moveit_group_.getCurrentState()->getRobotModel(), config_.GROUP_NAME); //TODO group.getRobotModel
  }

  void HybridPlanner::appendFreeMotionTrajectorySegment(std::vector<geometry_msgs::Pose>& waypoints)
  {
    TrajectorySegment current_free_segment;
    current_free_segment.type_ = FREE_MOTION;
    current_free_segment.waypoints_geom_msgs_ = waypoints;
    unplanned_trajectory_.push_back(current_free_segment);      
  }

  void HybridPlanner::appendProcessPathTrajectorySegment(std::vector<geometry_msgs::Pose>& waypoints)
  {
    TrajectorySegment current_process_segment;
    current_process_segment.type_ = PROCESS_PATH;
    current_process_segment.waypoints_geom_msgs_ = waypoints;
    unplanned_trajectory_.push_back(current_process_segment);              
  }

  void HybridPlanner::PlanHybrid()
  {
    for(auto it = unplanned_trajectory_.begin(); it != unplanned_trajectory_.end(); it++) 
    {
      // Free segment planning
      if(it->type_ == FREE_MOTION) 
      {       
        moveit_msgs::RobotTrajectory planned_moveit_robot_traj_segment; // container for the planned MoveIt! trajectory
        moveit_group_.setPlanningTime(config_.PLAN_TIME); 
        std::vector<geometry_msgs::Pose> current_free_segment = it->waypoints_geom_msgs_; 
        double fraction = moveit_group_.computeCartesianPath(current_free_segment, config_.STEP_SIZE, config_.JUMP_THRESH, planned_moveit_robot_traj_segment); //TODO fraction isn't used

        // convert from moveit_msgs::RobotTrajectory to robot_trajectory::RobotTrajectory
        robot_trajectory::RobotTrajectory planned_robot_traj_segment(moveit_group_.getCurrentState()->getRobotModel(), config_.GROUP_NAME);
        planned_robot_traj_segment.setRobotTrajectoryMsg(*moveit_group_.getCurrentState(), planned_moveit_robot_traj_segment);
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        bool success = iptp.computeTimeStamps(planned_robot_traj_segment);
        ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

        //Append planned trajectory for the current segment to the overall trajectory member variable 
        overall_robot_traj_.append(planned_robot_traj_segment, planned_robot_traj_segment.getWaypointDurationFromStart(planned_robot_traj_segment.getWayPointCount()));
        ROS_INFO("Appended Free space segment");
      }   

      // Process segment planning
      if(it->type_ == PROCESS_PATH)
      {
        using namespace descartes_core;
        using namespace descartes_trajectory; 

        // Container for the current segment
        TrajectoryVec process_segment_descartes; 
       
        // Convert from geometry_msgs::Pose to Descartes point type
        for(auto segment_iterator = it->waypoints_geom_msgs_.begin(); segment_iterator!= it->waypoints_geom_msgs_.end(); segment_iterator++)
        {
          Eigen::Affine3d pose;
          pose = Eigen::Translation3d(segment_iterator->position.x, segment_iterator->position.y, segment_iterator->position.z); 
          descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pose);
          process_segment_descartes.push_back(pt);
        }
  
        // TODEBUG TODO For the segfault, the following block could be a cause.. But commenting it out still caused it, apparently with a diff error(no "empty traj sent for execution" if this block is commented out)
        // Append the first point of the next segment (if the current segment is not the last segment itself) 
        if(it != unplanned_trajectory_.end())
        {
          std::advance(it, 1);
          auto first_point_of_next_segment = it->waypoints_geom_msgs_.begin();
          Eigen::Affine3d first_point_of_next_segment_eigen;
          first_point_of_next_segment_eigen = Eigen::Translation3d(first_point_of_next_segment->position.x, first_point_of_next_segment->position.y, first_point_of_next_segment->position.z);
          descartes_core::TrajectoryPtPtr first_point_of_next_segment_descartes = makeTolerancedCartesianPoint(first_point_of_next_segment_eigen);
          process_segment_descartes.push_back(first_point_of_next_segment_descartes);
          std::advance(it, -1); //let the hybrid iterator come back to the current segment
        }

        // Prepend the last point of the previous segment (if the current segment is not the first segment itself) 
        if(it != unplanned_trajectory_.begin())
        { 
          std::advance(it, -1);
          auto last_point_of_previous_segment = it->waypoints_geom_msgs_.begin();
          Eigen::Affine3d last_point_of_previous_segment_eigen;
          last_point_of_previous_segment_eigen = Eigen::Translation3d(last_point_of_previous_segment->position.x, last_point_of_previous_segment->position.y, last_point_of_previous_segment->position.z);
          descartes_core::TrajectoryPtPtr last_point_of_previous_segment_descartes = makeTolerancedCartesianPoint(last_point_of_previous_segment_eigen);
          process_segment_descartes.insert(process_segment_descartes.begin(), last_point_of_previous_segment_descartes);
          std::advance(it, 1); //let the hybrid iterator come back to the current segment
        }

        /* ===== The better solution ===== */ 

        // Prepend the last point of the previous _planned_ segment, i.e. joint vals of the last point of the previous segment is the seed for the current process segment   
        // std::vector<double> joint_values = moveit_group_.getCurrentJointValues(); //this won't work always (does it depend if the previous segment was free/descrtes?)
        // what I mean to ask is that both moveit_group_, and descartes_model_ get updated? 
        // Best is to just extract from the overall_robot_traj_
        if(it != unplanned_trajectory_.begin())
        {
          // Get Joint Names. 
          // Joint names could/should be member variables of HybridPlanner class
          std::vector<std::string> names;
          node_handle_.getParam("controller_joint_names", names); 
          // Get joint valur of the last waypoint in the overall_robot_traj_
          std::vector<double> joint_values = overall_robot_traj_->getLastWayPointPtr->getJointPositions(names);        
          descartes_core::TimingConstraint timing_constraint = TimingConstraint(0); //TODO is this correct?
          TrajectoryPtPtr first_point = TrajectoryPtPtr(new JointTrajectoryPt(joint_values, timing_constraint)); //TODO TOFIX? timing_constraint is mandatory in constructor. Shouldn't be. 
          process_segment_descartes.insert(process_segment_descartes.begin(), first_point);
        }

        // getClosestJointPose to the first pose of the next segment in unplanned_trajectory_
        // We'd like the process path to end close to the beginning of the following (free motion) segment, we'd call getClosestJointPose method with the start joint point
        // of the free motion as the seed argument to this method. Thus, this operation should return a valid joint pose for the under-constrained cartesian point.

        // TO DO:
        if(it != unplanned_trajectory_.end())
        {
          std::advance(it, 1);
          auto first_point_of_next_segment = it->waypoints_geom_msgs_.begin();
          Eigen::Affine3d first_point_of_next_segment_eigen;
          first_point_of_next_segment_eigen = Eigen::Translation3d(first_point_of_next_segment->position.x, first_point_of_next_segment->position.y, first_point_of_next_segment->position.z);
          descartes_core::TrajectoryPtPtr first_point_of_next_segment_descartes = makeTolerancedCartesianPoint(first_point_of_next_segment_eigen);
          getClosestJointPose
          process_segment_descartes.push_back(first_point_of_next_segment_descartes);
          std::advance(it, -1); //let the hybrid iterator come back to the current segment
        }

        /* ===== The better solution ===== */ 

        // Container for planned process-path(descartes type) trajectory segment 
        TrajectoryVec result;
        
        // Call the Descartes planner to do some planning
        if(config_.descartes_planner_type_ == DENSE)
        {
          if (!descartes_planner_dense_.planPath(process_segment_descartes))
          {
            ROS_ERROR("Could not solve for a valid path");
          }

          if (!descartes_planner_dense_.getPath(result))
          {
            ROS_ERROR("Could not retrieve path");
          }  
        }   

        if(config_.descartes_planner_type_ == SPARSE)
        {
          if (!descartes_planner_sparse_.planPath(process_segment_descartes))
          {
            ROS_ERROR("Could not solve for a valid path");
          }

          if (!descartes_planner_sparse_.getPath(result))
          {
            ROS_ERROR("Could not retrieve path");
          }  
        }

        // Get Joint Names. 
        // Joint names could/should be member variables of HybridPlanner class
        std::vector<std::string> names;
        node_handle_.getParam("controller_joint_names", names); 

        // Generate a ROS joint trajectory with the result path, robot model, given joint names with a certain time delta between each trajectory point
        trajectory_msgs::JointTrajectory descartes_joint_solution = toROSJointTrajectory(result, names, 1.0);

        //Append planned path
        robot_trajectory::RobotTrajectory descartes_robot_traj(moveit_group_.getCurrentState()->getRobotModel(), config_.GROUP_NAME); //TODO moveit_group_.getRobotModel does the same thing. 
        descartes_robot_traj.setRobotTrajectoryMsg(*moveit_group_.getCurrentState(), descartes_joint_solution);
        overall_robot_traj_.append(descartes_robot_traj, descartes_robot_traj.getWaypointDurationFromStart(descartes_robot_traj.getWayPointCount())); 
        ROS_INFO("Appended process segment");
      }
    }
  }

  // TODO joint_names and time_delay should be descartes config params 
  trajectory_msgs::JointTrajectory HybridPlanner::toROSJointTrajectory(const TrajectoryVec& trajectory, const std::vector<std::string>& joint_names, double time_delay)
  {
    // Fill out information about our trajectory
    trajectory_msgs::JointTrajectory result;
    result.header.stamp = ros::Time::now(); //TODO this should be got from node_handle_?
    result.header.frame_id = "world_frame"; //TODO config param?
    result.joint_names = joint_names;       //TODO config param?   

    // For keeping track of time-so-far in the trajectory
    double time_offset = 0.0;               //TODO config param?

    // Loop through the trajectory
    for (TrajectoryIter it = trajectory.begin(); it != trajectory.end(); it++)
    {
      // Find nominal joint solution at this point
      std::vector<double> joints;
      it->get()->getNominalJointPose(std::vector<double>(), *descartes_model_, joints);

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

  descartes_core::TrajectoryPtPtr HybridPlanner::makeCartesianPoint(const Eigen::Affine3d& pose)
  {
    return TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose)) );
  }

  descartes_core::TrajectoryPtPtr HybridPlanner::makeTolerancedCartesianPoint(const Eigen::Affine3d& pose)
  {
    return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI/2.0-0.0001, AxialSymmetricPt::Z_AXIS) );
  }

}
