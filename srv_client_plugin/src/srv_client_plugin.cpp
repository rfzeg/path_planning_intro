#include <pluginlib/class_list_macros.h>
#include <srv_client_plugin.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(srv_client_plugin::SrvClientPlugin, nav_core::BaseGlobalPlanner)

using namespace std;

namespace srv_client_plugin
{

  SrvClientPlugin::SrvClientPlugin()
  {
  }

  SrvClientPlugin::SrvClientPlugin(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {
    initialize(name, costmap_ros);
  }

  void SrvClientPlugin::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {
  }

  // generate a dummy global plan containing only start and goal waypoints
  bool SrvClientPlugin::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
  {
    // add start location to global plan vector
    plan.push_back(start);
    // insert goal location
    plan.push_back(goal);
    return true;
  }

}; // namespace global_planner