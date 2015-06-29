#include <ros/ros.h>
#include <moveit_msgs/GetCartesianPath.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <descartes_core/path_planner_base.h>
#include <descartes_core/robot_model.h>
#include <pluginlib/class_loader.h>

static const std::string ROBOT_MODEL_PLUGIN_PARAM = "robot_model_plugin";
static const std::string PLANNER_PLUGIN_PARAM = "planner_plugin";
static const std::string PLANNER_CONFIG_PARAM = "planner_config";
static const std::string GROUP_PARAM = "group_name";
static const std::string WORLD_FRAME_PARAM = "world_frame";
static const std::string TCP_FRAME_PARAM = "tcp_frame";
static const std::string TOOL_SPEED_PARAM = "tool_speed";
static const std::string INTERPOLATION_INTERVAL = "interpolation_interval";

static const std::string SPARSE_PLANNER_SAMPLING = "planner_sampling";
static const std::string GET_CARTESIAN_PATH_SERVICE = "get_cartesian_path";
static const std::string ROBOT_DESCRIPTION_NAME = "robot_description";


class CartesianPathService
{
public:

  CartesianPathService()
  {

  }

  ~CartesianPathService()
  {

  }

  bool run()
  {
    if(!init())
    {
      return false;
    }

    return true;
  }

protected:

  bool init()
  {
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    // loading parameters
    if(ph.getParam(ROBOT_MODEL_PLUGIN_PARAM,robot_model_plugin_) &&
        ph.getParam(PLANNER_PLUGIN_PARAM,planner_plugin_) &&
        ph.getParam(PLANNER_CONFIG_PARAM,planner_config_) &&
        ph.getParam(GROUP_PARAM,group_) &&
        ph.getParam(WORLD_FRAME_PARAM,world_frame_) &&
        ph.getParam(TCP_FRAME_PARAM,tcp_frame_) &&
        ph.getParam(TOOL_SPEED_PARAM,tool_speed_) &&
        ph.getParam(INTERPOLATION_INTERVAL,interp_interval_)
        )
    {
      ROS_INFO_STREAM("Parameters loaded successfully");
    }
    else
    {
      ROS_ERROR_STREAM("Parameters failed to load");
      return false;
    }

    // loading plugin
    pluginlib::ClassLoader<descartes_core::RobotModel> descartes_robot_model_class_loader(
        "descartes_core","descartes_core::RobotModel");
    pluginlib::ClassLoader<descartes_core::PathPlannerBase> planner_class_loader(
        "descartes_core","descartes_core::PathPlannerBase");
    try
    {
      robot_model_ = descartes_robot_model_class_loader.createInstance(robot_model_plugin_);
      planner_ = planner_class_loader.createInstance(planner_plugin_);
    }
    catch(pluginlib::PluginlibException& ex)
    {
      ROS_ERROR_STREAM("Failed to load plugin: "<<ex.what());
      return false;
    }

    // initializing plugins and interpolator
    if(!robot_model_->initialize(ROBOT_DESCRIPTION_NAME,group_,world_frame_,tcp_frame_))
    {
      ROS_ERROR_STREAM("Robot model pluging initialization failed");
      return false;
    }

    descartes_core::RobotModelConstPtr rm = robot_model_;
    if(!(planner_->initialize(rm)
        && planner_->setConfig(planner_config_)))
    {
      ROS_ERROR_STREAM("Planner plugin initialization failed");
      return false;
    }

    if(!interpolator_.initialize(robot_model_,tool_speed_,interp_interval_,zone_radius_))
    {
      ROS_ERROR_STREAM("Interpolator initialization failed");
      return false;
    }


    cartesian_path_server_ = nh.advertiseService(GET_CARTESIAN_PATH_SERVICE,
                                                 &CartesianPathService::cartesianPathCallback,
                                                 this);


    return true;
  }


  bool cartesianPathCallback(moveit_msgs::GetCartesianPath::Request& req,
                             moveit_msgs::GetCartesianPath::Response& res)
  {


    return true;
  }

protected:

  descartes_core::PathPlannerBasePtr planner_;
  descartes_core::RobotModelPtr robot_model_;
  descartes_trajectory::CartesianInterpolator interpolator_;

  ros::ServiceServer cartesian_path_server_;

  std::string robot_model_plugin_;
  std::string planner_plugin_;
  std::string group_;
  std::string world_frame_;
  std::string tcp_frame_;
  std::map<std::string,std::string> planner_config_;
  double tool_speed_;
  double interp_interval_;
  double zone_radius_;

};


int main(int argc,char** argv)
{
  ros::init(argc,argv,"cartesian_path_service");
  ros::NodeHandle nh;

  CartesianPathService cp;
  if(!cp.run())
  {
    return -1;
  }

  return 0;
}
