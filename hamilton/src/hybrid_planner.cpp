#include "hamilton/hybrid_planner.h"

//TODO namespace
namespace
{
	HybridPlanner::HybridPlanner()
	{

	}
	
	HybridPlanner::~HybridPlanner()
	{

	}

	HybridPlanner::appendDescartesCartPoint(EigenSTL::vector_Affine3d& pose)
	{	
		//TODO allocate size of descartes_traj_ or just suffice by push_back()?
	  	// publishing trajectory pose for visualization
	  	// publishPosesMarkers(pose);
	  	using namespace descartes_core;
  		using namespace descartes_trajectory;
	  	descartes_core::TrajectoryPtPtr pt = TrajectoryPtPtr(new CartTrajectoryPt(TolerancedFrame(pose))
	  	descartes_traj_.push_back(pt);
	}

	HybridPlanner::appendTolerancedDescartesCartPoint(EigenSTL::vector_Affine3d& pose)
	{
		using namespace descartes_core;
	  	using namespace descartes_trajectory;
	  	descartes_core::TrajectoryPtPtr pt = TrajectoryPtPtr(new AxialSymmetricPt(pose, ORIENTATION_INCREMENT, 
	  															AxialSymmetricPt::Z_AXIS);
	  	descartes_traj_.push_back(pt);
	}

	HybridPlanner::appendDescartesCartPointSegment(std::std::vector<char> v;)
	{
		using namespace descartes_core;
		using namespace descartes_trajectory;

	}

	HybridPlanner::appendFreeSpacePoint()
	{
		
	}
	//Helper functions from Godel. 
	//TODO Need to set descretization as member variable? 
	descartes_core::TrajectoryPtPtr HybridPlanner::tfToAxialTrajectoryPt(const tf::Transform& nominal, 
																		double discretization)
  	{	
	    using namespace descartes_core;
	    using namespace descartes_trajectory;

	    Eigen::Affine3d eigen_pose;
	    tf::poseTFToEigen(nominal,eigen_pose);
	    Eigen::Vector3d rpy =  eigen_pose.rotation().eulerAngles(0,1,2);

	    double rx = rpy(0);
	    double ry = rpy(1);
	    double rz = rpy(2);
	    double x = eigen_pose.translation()(0);
	    double y = eigen_pose.translation()(1);
	    double z = eigen_pose.translation()(2);


	    return boost::shared_ptr<TrajectoryPt>(
	      new CartTrajectoryPt(
	            TolerancedFrame(utils::toFrame(x,y, z, rx, ry, rz, descartes_core::utils::EulerConventions::XYZ),
	             ToleranceBase::zeroTolerance<PositionTolerance>(x, y, z),
	             ToleranceBase::createSymmetric<OrientationTolerance>(rx, ry, 0, 0, 0, 2.0 * M_PI)),
	            0.0, discretization));
  	}

	// Translates a point relative to a reference pose to an absolute transformation
	// Also flips the z axis of the orientation to point INTO the plane specified by the
	// reference frame.
	tf::Transform HybridPlanner::createNominalTransform(const geometry_msgs::Pose& ref_pose, const geometry_msgs::Point& point)
	{
		// To plane
		tf::Transform marker_pose;
		tf::poseMsgToTF(ref_pose, marker_pose);

		// From plane to point
		tf::Transform to_point = tf::Transform::getIdentity();
		to_point.setOrigin(tf::Vector3(point.x, point.y, point.z));
		// Reverse orientation of z axis
		tf::Quaternion quat;
		quat.setEuler(0.0, M_PI, 0.0); // yaw, pitch roll
		tf::Transform flip_z (quat);
		// Calculate transform
		tf::Transform in_world = marker_pose * to_point * flip_z;

		return in_world;
	}

}
