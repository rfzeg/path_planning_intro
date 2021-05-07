/*
 * costmap_to_gridmap.cpp
 *
 *  Created on: Mai 10, 2021
 *      Author: Roberto Zegers R.
 */

#include <ros/ros.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

namespace costmap_to_gridmap{

    using namespace grid_map;

    class Costmap2Gridmap{

    public:
        Costmap2Gridmap() : nh_("~")
        {
            // Input
            costmap_sub_ = nh_.subscribe("/input_topic/costmap", 1, &Costmap2Gridmap::costmap_cb, this);
            // Output
            grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
        }

    private:
        GridMap grid_map_;

        ros::Publisher grid_map_pub_;
        ros::Subscriber costmap_sub_;
        ros::NodeHandle nh_;

        void costmap_cb(const nav_msgs::OccupancyGrid incoming_grid)
        {
          grid_map::GridMapRosConverter::fromOccupancyGrid(incoming_grid, "elevation", grid_map_);

          // Scale layer to range [0-1]
          grid_map_["elevation"] = 0.01 * grid_map_["elevation"];
          // Publish as grid map
          grid_map_msgs::GridMap mapMessage;
          grid_map::GridMapRosConverter::toMessage(grid_map_, mapMessage);
          grid_map_pub_.publish(mapMessage);
        }
    };

}

int main(int argc, char** argv){

    ros::init(argc, argv, "costmap_to_gridmap");

    costmap_to_gridmap::Costmap2Gridmap costmap2gridmap;

    ros::spin();

}