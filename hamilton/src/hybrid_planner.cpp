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

	HybridPlanner::appendDescartesJointPoint(EigenSTL::vector_Affine3d& pose)
	{	
		//TODO allocate size of descartes_traj_ or just suffice by push_back()?
	  	// publishing trajectory pose for visualization
	  	// publishPosesMarkers(pose);
	  	using namespace descartes_core;
  		using namespace descartes_trajectory;
	  	descartes_core::TrajectoryPtPtr pt = TrajectoryPtPtr(new CartTrajectoryPt(TolerancedFrame(pose))
	  	descartes_traj_.push_back(pt);
	}

	HybridPlanner::appendTolerancedDescartesJointPoint(EigenSTL::vector_Affine3d& pose)
	{
		using namespace descartes_core;
	  	using namespace descartes_trajectory;
	  	descartes_core::TrajectoryPtPtr pt = TrajectoryPtPtr(new AxialSymmetricPt(pose, ORIENTATION_INCREMENT, 
	  															AxialSymmetricPt::Z_AXIS);
	  	descartes_traj_.push_back(pt);
	}
}
