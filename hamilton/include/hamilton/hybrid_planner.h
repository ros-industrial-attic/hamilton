#ifndef HYBRID_PLANNER_H
#define HYBRID_PLANNER_H

#include <ros/ros.h>
#include <boost/function.hpp>
#include <boost/assign/list_of.hpp>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
#include <descartes_trajectory/joint_trajectory_pt.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_planner/sparse_planner.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>
// #include <ur5_demo_descartes/ur5_robot_model.h>
#include <pluginlib/class_loader.h>
#include <descartes_moveit/moveit_state_adapter.h>
#include <geometry_msgs/PoseArray.h>
//TODO instead of specifying traj, planner and their subtypes explicitly -> find a simpler way


//TODO namespace anon vs named
namespace hamilton
{
//TODO TOCHECK ros params vs protected variables (vs partially hard coded)
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const std::string EXECUTE_TRAJECTORY_SERVICE = "execute_kinematic_path";
const std::string VISUALIZE_TRAJECTORY_TOPIC = "visualize_trajectory_curve";
const double SERVICE_TIMEOUT = 5.0f; // seconds
const double ORIENTATION_INCREMENT = 0.5f;
const double EPSILON = 0.0001f;
const double AXIS_LINE_LENGHT = 0.01;
const double AXIS_LINE_WIDTH = 0.001;
const std::string PLANNER_ID = "RRTConnectkConfigDefault";
const std::string HOME_POSITION_NAME = "home";
//TOCHECK config_ members or hardcoded MoveIt! params

typedef std::vector<descartes_core::TrajectoryPtPtr> descartes_traj;
typedef descartes_trajectory::CartTrajectoryPt descartes_cart_pt;
// typedef descartes_trajectory::CartTrajectoryPt::AxialSymmetricPt descartes_cart_ax_sym_pt;
typedef descartes_trajectory::JointTrajectoryPt descartes_joint_traj_pt;
typedef moveit_msgs::RobotTrajectory moveit_robot_traj;
typedef trajectory_msgs::JointTrajectory moveit_joint_traj;
typedef boost::shared_ptr<move_group_interface::MoveGroup> MoveGroupPtr;
// typedef moveit_msgs::RobotTrajectory::joint_trajectory moveit_joint_traj;
// typedef trajectory_msgs::JointTrajectory moveit_joint_traj;
// typedef descartes_core::RobotModelPtr descartes_robot_model_ptr;



/*  =============================== Application Data Structure ===============================
 *
 * Holds the data used at various points in the application.  This structure is populated
 * from data found in the ros parameter server at runtime.
 *
 */

 //TODO TOCHECK HybridConfiguration should be broken into three structs:
 // - MoveIt! params
 // - Descartes params
 // - Common or global params

 //Params should be entered from GUI. Adapt Fermi for this?

struct HybridConfiguration
{
  std::string group_name;                 /* Name of the manipulation group containing the relevant links in the robot */
  std::string tip_link;                   /* Usually the last link in the kinematic chain of the robot */
  std::string base_link;                  /* The name of the base link of the robot */
  std::string world_frame;                /* The name of the world link in the URDF file */
  std::vector<std::string> joint_names;   /* A list with the names of the mobile joints in the robot */

  /* Trajectory Generation Members:
   *  Used to control the attributes (points, shape, size, etc) of the robot trajectory. */

  //TODO add support for straight lines, circle, B-splines. add to list. 

  double time_delay;              /* Time step between consecutive points in the robot path */
  std::vector<double> seed_pose;  /* Joint values close to the desired start of the robot path */

  /* Visualization Members
   * Used to control the attributes of the visualization artifacts
   */
  double min_point_distance;      /* Minimum distance between consecutive trajectory points. */

  //MoveIt! params
  const double plan_time; //planning time of MoveIt!
  const double step_size; 
  const double jump_thresh;
  const bool moveit_replan;
  const bool avoid_collision;

  const bool descartes_replan; //TOCHECK 
  std::string descartes_planner_type; //dense vs sparse
};


/*  =============================== Hybrid Planner Class ===============================
 *
 * Provides a group of functions for planning and executing a robot path using Moveit and
 * the Descartes Planning Library */

class HybridPlanner
{
  public:

    HybridPlanner();
    virtual ~HybridPlanner();

    void loadParameters();
    void init();
    void update();
    // void initRos();
    // void initDescartes();
    void moveHome();
    void generateTrajectory(descartes_traj& traj);
    void planPath(descartes_traj& input_traj,descartes_traj& output_path);
    void runPath(const descartes_traj& path);

  protected:
    // TOCHECK bool vs void. 
    bool appendDescartesPoint();
    bool appendDescartesCartPoint(Eigen::Affine3d& pose);
    bool appendTolerancedDescartesCartPoint(Eigen::Affine3d& pose);
    bool appendDescartesCartPointSegment(EigenSTL::vector_Affine3d poses);
    bool appendDescartesJointPoint();
    bool appendFreeSpaceSegment(std::vector<geometry_msgs::Pose> waypoints);
    bool appendFreeSpaceCartPoint();
    bool appendFreeSpaceJointPoint();
    void planProcessPath();

    // void HybridPlanner::appendDescartesCartPoint(EigenSTL::vector_Affine3d& pose);

    /* Create an axial trajectory pt from a given tf transform */
    descartes_core::TrajectoryPtPtr tfToAxialTrajectoryPt(const tf::Transform& nominal, double discretization);   

    /*Translates a point relative to a reference pose to an absolute transformation
    Also flips the z axis of the orientation to point INTO the plane specified by the
    reference frame.*/
    tf::Transform createNominalTransform(const geometry_msgs::Pose& ref_pose, const geometry_msgs::Point& point);

    void fromDescartesToMoveitTrajectory(const descartes_traj& in_traj);


    void publishPosesMarkers(const EigenSTL::vector_Affine3d& poses);


  protected:

    /* Data
     * Holds the data used by the various functions  
     */
    HybridConfiguration config_;

    /* ROS Constructs
     * Components needed to successfully run a ros-node and perform other important
     * ros-related tasks
     */
    ros::NodeHandle nh_;                        /* Object used for creating and managing ros resources*/
    ros::Publisher marker_publisher_;           /* Publishes visualization message to Rviz */
    ros::ServiceClient moveit_run_path_client_; /* Sends a robot trajectory to moveit for execution */

    /* Descartes Constructs
     * Components accessing the semi-constrained path planning capabilities in the Descartes library
     */
    descartes_traj& descartes_traj_input_;
    descartes_traj& descartes_traj_output_;  /* Maintains the semi-constrained points and segments*/
    trajectory_msgs::JointTrajectory& joint_traj_descartes_;//TODO change name
    descartes_core::RobotModelPtr descartes_robot_model_ptr_; /* Performs tasks specific to the Robot
                                                       such IK, FK and collision detection*/
    descartes_planner::SparsePlanner sparse_planner_;      /* Plans a smooth robot path given a trajectory of points */
    descartes_planner::DensePlanner dense_planner_;
    int no_of_descartes_segments_;

    /* MoveIt! Constructs
     * Components accesing the free-space planning capabilities in the MoveIt! library.
     */
    moveit::core::RobotStatePtr kinematic_state_;
    robot_model::RobotModelConstPtr& kinematic_model_;
    const moveit::core::JointModelGroup* joint_model_group_; //TOCHECK what all to keep const coz of return type of kinematic_model_->getJointModelGroup
    boost::shared_ptr<move_group_interface::MoveGroup> moveit_group_ptr_;
    moveit_robot_traj moveit_robot_traj_;
    robot_trajectory::RobotTrajectory overall_robot_traj_; //TODO better non-confusing names
    move_group_interface::MoveGroup::Plan overall_plan_; //TOCHECK Member variable vs instantiate within the plan or addFreeSpaceSegment function?
    int no_of_free_segments_; 
    //TODO just one naming scheme to avoid confusion 
    //Free vs process || descartes vs moveit 
};

} /* namespace */

#endif /* HYBRID_PLANNER_H */
