// //TOCHECK 
// #ifdef __i386__
//   #pragma message("i386 Architecture detected, disabling EIGEN VECTORIZATION")
//   #define EIGEN_DONT_VECTORIZE
//   #define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
// #else
//   #pragma message("64bit Architecture detected, enabling EIGEN VECTORIZATION")
// #endif

#include "hamilton/hybrid_planner.h"

int main(int argc,char** argv)
{
  // ros::init(argc,argv,"HybridPlanner");
  // ros::AsyncSpinner spinner(2);
  // spinner.start();

  // creating hybrid_planner_
  hamilton::HybridPlanner hybrid_planner_;

  // loading parameters
  hybrid_planner_.loadParameters();

  // initializing ros components and Descartes
  hybrid_planner_.init();

  // // moving to home position
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
  // spinner.stop();

  return 0;
}

