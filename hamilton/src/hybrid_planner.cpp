#include "hamilton/hybrid_planner.h"

//TODO namespace
//TODO bools aren't returning anything!
namespace hamilton
{
	
	HybridPlanner::~HybridPlanner()
	{
		moveit_group_ptr_.reset();
		kinematic_state_.reset();
		//TODO Descartes
	}

	void HybridPlanner::init()
	{
		// Instantiating a robot model
		descartes_robot_model_ptr_.reset(new descartes_moveit::MoveitStateAdapter);
		// TOCHECK as compared to: 
		//descartes_core::RobotModelPtr model(new descartes_moveit::MoveitStateAdapter);

		if(descartes_robot_model_ptr_->initialize(ROBOT_DESCRIPTION_PARAM,
		                              config_.group_name,
		                              config_.world_frame,
		                              config_.tip_link))
		{
		ROS_INFO_STREAM("Descartes Robot Model initialized");
		}
		else
		{
		ROS_ERROR_STREAM("Failed to initialize Robot Model");
		exit(-1);
		}

		// TODO add planner type as config parameter?
		bool succeeded_dense = dense_planner_.initialize(descartes_robot_model_ptr_);
		if(succeeded_dense)
		{
		ROS_INFO_STREAM("Descartes dense planner initialized");
		}
		else
		{
		ROS_ERROR_STREAM("Failed to initialize Descartes dense Planner");
		exit(-1);
		}  

		bool succeeded_sparse = sparse_planner_.initialize(descartes_robot_model_ptr_);
		if(succeeded_sparse)
		{
		ROS_INFO_STREAM("Descartes sparse planner initialized");
		}
		else
		{
		ROS_ERROR_STREAM("Failed to initialize Descartes sparse Planner");
		exit(-1);
		}

		ROS_INFO_STREAM("Descartes members initialized");

		// creates publisher for trajectory visualization
		marker_publisher_  = nh_.advertise<visualization_msgs::MarkerArray>(VISUALIZE_TRAJECTORY_TOPIC,1,true);

		//Initializes MoveIt!
		moveit_group_ptr_ = boost::shared_ptr<move_group_interface::MoveGroup>(new moveit_group_interface::MoveGroup(config_.group_name));
		kinematic_state_ = moveit_group_ptr_->getCurrentState();
		kinematic_state_->setToDefaultValues(); //TOCHECK
		kinematic_model_ = kinematic_state_->getRobotModel();
		joint_model_group_ = kinematic_model_->getJointModelGroup(config_.group_name); 
		no_of_free_segments_ = 0;
		// ROS_INFO_STREAM("MoveIt! members initialized");

		// //creates a moveit_msgs::ExecuteKnownTrajectory client and assign it to the moveit_run_path_client_ member variable.  
		// moveit_run_path_client_ = nh_.serviceClient<moveit_msgs::ExecuteKnownTrajectory>(EXECUTE_TRAJECTORY_SERVICE,true);
		// // Establishing connection to server
		// if(moveit_run_path_client_.waitForExistence(ros::Duration(SERVICE_TIMEOUT)))
		// {
		// ROS_INFO_STREAM("Connected to '"<<moveit_run_path_client_.getService()<<"' service");
		// }
		// else
		// {
		// ROS_ERROR_STREAM("Failed to connect to '"<< moveit_run_path_client_.getService()<<"' service");
		// exit(-1);
		// }

		ROS_INFO_STREAM("Task '"<<__FUNCTION__<<"' completed");
		// ROS_INFO_STREAM("Descartes and MoveIt! are happy.");
	}

	void HybridPlanner::update()
	{
		moveit_group_ptr_->getCurrentState();
		kinematic_model_ = kinematic_state_->getRobotModel();
		joint_model_group_ = kinematic_model_->getJointModelGroup(config_.group_name); 
		ROS_INFO_STREAM("MoveIt! members updated");
		//TODO show the status and values of member variables ROS_INFO_STREAM
	}

	void HybridPlanner::loadParameters()
	{
		ros::NodeHandle ph("~");
		ros::NodeHandle nh;
		//TOCHECK if namespaces are needed in the node handle constructors 
		//TOCHECK required parameters
		//TODO MoveIt params are hardcoded
		if(ph.getParam("group_name",config_.group_name) &&
		  ph.getParam("tip_link",config_.tip_link) &&
		  ph.getParam("base_link",config_.base_link) &&
		  ph.getParam("world_frame",config_.world_frame) &&
		  ph.getParam("trajectory/time_delay",config_.time_delay) &&
		  ph.getParam("trajectory/seed_pose",config_.seed_pose) &&
		  ph.getParam("visualization/min_point_distance",config_.min_point_distance) &&
		  nh.getParam("controller_joint_names",config_.joint_names) )
		{
		ROS_INFO_STREAM("Loaded Descartes parameters");
		}
		else
		{
		ROS_ERROR_STREAM("Failed to load Descartes parameters");
		exit(-1);
		}

		//TOCHECK Set the following hereor should be loaded seperately for each free space segment?
		moveit_group_ptr_->setPlanningTime(config_.plan_time);
		moveit_group_ptr_->allowReplanning(config_.moveit_replan);
		ROS_INFO_STREAM("Loaded MoveIt! parameters");//TOCHECK add negative debug statement, if needed with if(){} block. 

		ROS_INFO_STREAM("Task '"<<__FUNCTION__<<"' completed");
	}


	bool HybridPlanner::appendDescartesCartPoint(Eigen::Affine3d& pose)
	{	
		//TOCHECK pass const parameter? 
		//TODO allocate size of descartes_traj_ or just suffice by push_back()?
	  	using namespace descartes_core;
  		using namespace descartes_trajectory;
	  	TrajectoryPtPtr pt = TrajectoryPtPtr(new CartTrajectoryPt(TolerancedFrame(pose)));
	  	
	  	if(!pt)
  		{	
  			descartes_traj_.push_back(pt);
	  	// publishing trajectory pose for visualization
	  	// publishPosesMarkers(pose);
  			return 1;	
  		}
  
	 	else
	 	{
	  		ROS_WARN("Failed for append point. Moving on anyway.");
	  		return 0;
	  	}	
	}

	bool HybridPlanner::appendTolerancedDescartesCartPoint(Eigen::Affine3d& pose)
	{
		using namespace descartes_core;
	  	using namespace descartes_trajectory;
	  	TrajectoryPtPtr pt = TrajectoryPtPtr(new AxialSymmetricPt(pose, ORIENTATION_INCREMENT, 
	  															AxialSymmetricPt::Z_AXIS));
	  	descartes_traj_.push_back(pt);
	}

	bool HybridPlanner::appendDescartesCartPointSegment(EigenSTL::vector_Affine3d poses)
	{
		using namespace descartes_core;
		using namespace descartes_trajectory;

		publishPosesMarkers(poses);

		for(int i=0; i<poses.size(); i++)
		{
			 const Eigen::Affine3d& pose = poses[i];

   
		   	// Create AxialSymetricPt objects in order to define a trajectory cartesian point with
		    //   rotational freedom about the tool's z axis.
		   	
		    descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(
        			new descartes_trajectory::AxialSymmetricPt(pose,ORIENTATION_INCREMENT,
                                                   descartes_trajectory::AxialSymmetricPt::FreeAxis::Z_AXIS) );

		    // saving points into trajectory
		    descartes_traj_.push_back(pt);
		}

	}

	void HybridPlanner::publishPosesMarkers(const EigenSTL::vector_Affine3d& poses)
	{
	  	// creating rviz markers
	  	visualization_msgs::Marker z_axes, y_axes, x_axes, line;
  		visualization_msgs::MarkerArray markers_msg;

	  	z_axes.type = y_axes.type = x_axes.type = visualization_msgs::Marker::LINE_LIST;
	  	z_axes.ns = y_axes.ns = x_axes.ns = "axes";
	  	z_axes.action = y_axes.action = x_axes.action = visualization_msgs::Marker::ADD;
	  	z_axes.lifetime = y_axes.lifetime = x_axes.lifetime = ros::Duration(0);
	  	z_axes.header.frame_id = y_axes.header.frame_id = x_axes.header.frame_id = config_.world_frame;
	  	z_axes.scale.x = y_axes.scale.x = x_axes.scale.x = AXIS_LINE_WIDTH;

	  	// z properties
	  	z_axes.id = 0;
	  	z_axes.color.r = 0;
	  	z_axes.color.g = 0;
	  	z_axes.color.b = 1;
	  	z_axes.color.a = 1;

	  	// y properties
	  	y_axes.id = 1;
	  	y_axes.color.r = 0;
	  	y_axes.color.g = 1;
	  	y_axes.color.b = 0;
	  	y_axes.color.a = 1;

		// x properties
		x_axes.id = 2;
		x_axes.color.r = 1;
		x_axes.color.g = 0;
		x_axes.color.b = 0;
		x_axes.color.a = 1;

		// line properties
		line.type = visualization_msgs::Marker::LINE_STRIP;
		line.ns = "line";
		line.action = visualization_msgs::Marker::ADD;
		line.lifetime = ros::Duration(0);
		line.header.frame_id = config_.world_frame;
		line.scale.x = AXIS_LINE_WIDTH;
		line.id = 0;
		line.color.r = 1;
		line.color.g = 1;
		line.color.b = 0;
		line.color.a = 1;

		// creating axes markers
		z_axes.points.reserve(2*poses.size());
		y_axes.points.reserve(2*poses.size());
		x_axes.points.reserve(2*poses.size());
		line.points.reserve(poses.size());
		geometry_msgs::Point p_start,p_end;
		double distance = 0;
		Eigen::Affine3d prev = poses[0];
		for(unsigned int i = 0; i < poses.size(); i++)
		{
		const Eigen::Affine3d& pose = poses[i];
		distance = (pose.translation() - prev.translation()).norm();

		tf::pointEigenToMsg(pose.translation(),p_start);

		if(distance > config_.min_point_distance)
		{
		  Eigen::Affine3d moved_along_x = pose * Eigen::Translation3d(AXIS_LINE_LENGHT,0,0);
		  tf::pointEigenToMsg(moved_along_x.translation(),p_end);
		  x_axes.points.push_back(p_start);
		  x_axes.points.push_back(p_end);

		  Eigen::Affine3d moved_along_y = pose * Eigen::Translation3d(0,AXIS_LINE_LENGHT,0);
		  tf::pointEigenToMsg(moved_along_y.translation(),p_end);
		  y_axes.points.push_back(p_start);
		  y_axes.points.push_back(p_end);

		  Eigen::Affine3d moved_along_z = pose * Eigen::Translation3d(0,0,AXIS_LINE_LENGHT);
		  tf::pointEigenToMsg(moved_along_z.translation(),p_end);
		  z_axes.points.push_back(p_start);
		  z_axes.points.push_back(p_end);

		  // saving previous
		  prev = pose;
		}

		line.points.push_back(p_start);
		}

		markers_msg.markers.push_back(x_axes);
		markers_msg.markers.push_back(y_axes);
		markers_msg.markers.push_back(z_axes);
		markers_msg.markers.push_back(line);

		marker_publisher_.publish(markers_msg);

	}
	// // HybridPlanner::appendJointSpacePoint(std::std::vector<double> joints)
	// // {
	// //   	group.setJointValueTarget(joints);
	// //   	// group.move()
	// // }

	bool HybridPlanner::appendFreeSpaceSegment(std::vector<geometry_msgs::Pose> waypoints)
	{
		
		// moveit_trajectory_.append(waypoint);
		// moveit_trajectory_.push_back(waypoints);
		// TODO TOCHECK maintain waypoints and call computecartesianpath in the plan / execute function
		// We want the cartesian path to be interpolated at a resolution of 1 cm which is why we will specify 0.01 as the max step in cartesian translation. 
		// We will specify the jump threshold as 0.0, effectively disabling it.

		double fraction = moveit_group_ptr_.computeCartesianPath(waypoints,
		                                             config_.step_size,  // eef_step
		                                             config_.jump_thresh,   // jump_threshold
		                                             moveit_robot_traj_);	

		robot_trajectory::RobotTrajectory robot_traj_segment(kinematic_model_, config_.group_name);
		robot_traj_segment.setRobotTrajectoryMsg(*kinematic_state_, moveit_robot_traj_);
		// robot_state::RobotState last_robot_state = robot_traj_segment.getLastWayPoint();

		overall_robot_traj_.setRobotTrajectoryMsg(*kinematic_state_, moveit_robot_traj_);

		overall_robot_traj_.append(robot_traj_segment,
    		robot_traj_segment.getWaypointDurationFromStart(robot_traj_segment.getWayPointCount()));

		//TODO. Convert this to moveit_robot_traj_. (in the plan function)
		//TOCHECK this might mess up with the stored "descartesToMoveIt" trajectories?
	    // overall_robot_traj_.getRobotTrajectoryMsg(moveit_robot_traj_);
	    no_of_free_segments_ += 1;

	}


	// //Helper functions from Godel. 
	// //TODO Need to set descretization as member variable? 
	// descartes_core::TrajectoryPtPtr HybridPlanner::tfToAxialTrajectoryPt(const tf::Transform& nominal, 
	// 																	double discretization)
 //  	{	
	//     using namespace descartes_core;
	//     using namespace descartes_trajectory;

	//     Eigen::Affine3d eigen_pose;
	//     tf::poseTFToEigen(nominal,eigen_pose);
	//     Eigen::Vector3d rpy =  eigen_pose.rotation().eulerAngles(0,1,2);

	//     double rx = rpy(0);
	//     double ry = rpy(1);
	//     double rz = rpy(2);
	//     double x = eigen_pose.translation()(0);
	//     double y = eigen_pose.translation()(1);
	//     double z = eigen_pose.translation()(2);


	//     return boost::shared_ptr<TrajectoryPt>(
	//       new CartTrajectoryPt(
	//             TolerancedFrame(utils::toFrame(x,y, z, rx, ry, rz, descartes_core::utils::EulerConventions::XYZ),
	//              ToleranceBase::zeroTolerance<PositionTolerance>(x, y, z),
	//              ToleranceBase::createSymmetric<OrientationTolerance>(rx, ry, 0, 0, 0, 2.0 * M_PI)),
	//             0.0, discretization));
 //  	}

	// // Translates a point relative to a reference pose to an absolute transformation
	// // Also flips the z axis of the orientation to point INTO the plane specified by the
	// // reference frame.
	// tf::Transform HybridPlanner::createNominalTransform(const geometry_msgs::Pose& ref_pose, const geometry_msgs::Point& point)
	// {
	// 	// To plane
	// 	tf::Transform marker_pose;
	// 	tf::poseMsgToTF(ref_pose, marker_pose);

	// 	// From plane to point
	// 	tf::Transform to_point = tf::Transform::getIdentity();
	// 	to_point.setOrigin(tf::Vector3(point.x, point.y, point.z));
	// 	// Reverse orientation of z axis
	// 	tf::Quaternion quat;
	// 	quat.setEuler(0.0, M_PI, 0.0); // yaw, pitch roll
	// 	tf::Transform flip_z (quat);
	// 	// Calculate transform
	// 	tf::Transform in_world = marker_pose * to_point * flip_z;

	// 	return in_world;
	// }

	// void HybridPlanner::moveit_computeCartesianPath(const std::vector<geometry_msgs::Pose>& waypoints)
	// {
	// 	double result = group.computeCartesianPath(const std::vector<geometry_msgs::Pose> &waypoints, double eef_step, double jump_threshold,
 //                  moveit_msgs::RobotTrajectory &trajectory,  bool avoid_collisions = true);
	// }


	// void HybridPlanner::fromDescartesToMoveitTrajectory(const DescartesTrajectory& in_traj)
	// {
	//   // Fill out information about our trajectory
	//   descartes_traj_.header.stamp = ros::Time::now();
	//   descartes_traj_.header.frame_id = config_.world_frame;
	//   descartes_traj_.joint_names = config_.joint_names;

	//   // For keeping track of time-so-far in the trajectory
	//   double time_offset = 0.0;

	//   // Loop through the trajectory
	//   for (unsigned int i = 0; i < in_traj.size(); i++)
	//   {
	//     // Find nominal joint solution at this point
	//     std::vector<double> joints;

	//     // getting joint position at current point
	//     const descartes_core::TrajectoryPtPtr& joint_point = in_traj[i];
	//     joint_point->getNominalJointPose(std::vector<double>(), *descartes_robot_model_ptr_, joints);

	//     // Fill out a ROS trajectory point
	//     trajectory_msgs::JointTrajectoryPoint pt;
	//     pt.positions = joints;
	//     // velocity, acceleration, and effort are given dummy values
	//     // we'll let the controller figure them out
	//     pt.velocities.resize(joints.size(), 0.0);
	//     pt.accelerations.resize(joints.size(), 0.0);
	//     pt.effort.resize(joints.size(), 0.0);
	//     // set the time into the trajectory
	//     pt.time_from_start = ros::Duration(time_offset);
	//     // increment time
	//     time_offset += config_.time_delay;

	//     descartes_traj_.points.push_back(pt);
	//   }

	// }
}

#include "hamilton/hybrid_planner.h"

int main(int argc, char** argv)
{
  ros::init(argc,argv,"HybridPlanner");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // creating hybrid_planner_
  // hamilton::HybridPlanner hybrid_planner_;

  // // loading parameters
  // hybrid_planner_.loadParameters();

  // // initializing ros components and Descartes
  // hybrid_planner_.init();

  // // // moving to home position
  // hybrid_planner_.moveHome();

  // // generating trajectory
  // HybridPlanner::DescartesTrajectory traj;
  // hybrid_planner_.generateTrajectory(traj);


  // // planning robot path
  // HybridPlanner::DescartesTrajectory output_path;
  // hybrid_planner_.planPath(traj,output_path);

  // // running robot path
  // hybrid_planner_.runPath(output_path);

  // exiting ros node
  spinner.stop();

  return 0;
}

