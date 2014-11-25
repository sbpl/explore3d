/// Jonathan Butzke
///(c) 2014 Ros wrapper header
#include "exploration.hpp"
#include "ros.h"
#include "nav_msgs"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"

#define PLANNERRATE_ 0.1


class EP_wrapper {
public:
  void plannerthread(void);
  void init(void);

private:
  ExplorationPlanner_c EP;
  ExpParams_c params;
  Locations_c CurrentLocations_;
  std::vector<MapElement_c> MapPts_;

  double scale;

  std::mutex data_mutex_;
  ros::Publisher Goal_pub_;
  ros::Subscriber Map_sub_, Pose_sub_;
  ros::Nodehandle nh;
};