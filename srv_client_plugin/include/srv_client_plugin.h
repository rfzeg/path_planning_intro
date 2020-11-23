#ifndef SRV_CLIENT_PLUGIN_H_
#define SRV_CLIENT_PLUGIN_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

namespace srv_client_plugin{
  /**
   * @class SrvClientPlugin
   * @brief A global planner that takes in a service request containing a nav_msg/path msg and forwards it to the move_base global planner module
   */
  class SrvClientPlugin : public nav_core::BaseGlobalPlanner {
    public:
      /**
       * @brief  Constructor for the SrvClientPlugin
       */
      SrvClientPlugin();
      /**
       * @brief  Constructor for the SrvClientPlugin
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      SrvClientPlugin(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Initialization function for the SrvClientPlugin
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

    private:
      costmap_2d::Costmap2DROS* costmap_ros_;
      costmap_2d::Costmap2D* costmap_;
      bool initialized_;
  };
};  
#endif