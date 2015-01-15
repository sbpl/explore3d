#ifndef ExplorationExecutive_h
#define ExplorationExecutive_h

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <utacc_msgs/MissionStatus.h>
#include <utacc_msgs/MissionCommand.h>

#include "exploration_main.hpp"

namespace utacc
{

class ExplorationExecutive
{
public:

    ExplorationExecutive();
    ~ExplorationExecutive();

    int run();

private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    ros::Publisher goal_pub_;

    ros::Subscriber map_sub_;
    ros::Subscriber pose_sub_;

    ExplorationThread exploration_thread_;

    double executive_rate_hz_;
    nav_msgs::Path goals_;
    bool goal_pending_;

    bool initialize();
    void pose_callback(const nav_msgs::PathConstPtr& msg);
    void map_callback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& msg);

    void goals_callback(nav_msgs::Path& goals);
};

} // namespace utacc

#endif
