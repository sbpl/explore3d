/// Jonathan Butzke
///(c) 2014 Ros wrapper header
#include "exploration.hpp"
#include <ros/ros.h>
#include <nav_msgs/Path.h>


#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define PLANNERRATE_ 0.1


class EP_wrapper {
public:
  void plannerthread(void);
  void init(void);
  void PoseCallback(const nav_msgs::Path& msg);
  void MapCallback(const  pcl::PointCloud<pcl::PointXYZI>::ConstPtr& msg);
  ~EP_wrapper();

private:
  ExplorationPlanner EP;
  ExpParams_c params;
  std::vector<Locations_c> CurrentLocations_;
  std::vector<MapElement_c> MapPts_;

  double scale;

  std::mutex data_mutex_;
  std::thread *EP_thread_;
  ros::Publisher Goal_pub_;
  ros::Subscriber Map_sub_, Pose_sub_;
  std::string goal_topic_, map_topic_, pose_topic_;
  ros::NodeHandle nh;
};