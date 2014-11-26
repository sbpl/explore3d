///Jonathan Michael Butzke
///(c) 2014
///ROS Wrapper for 3-D exploration module

#include "exploration_main.hpp"

void EP_wrapper::plannerthread(void) {
  ros::Rate looprate(PLANNERRATE_);
  std::vector<MapElement_c> pts;
  std::vector<Locations_c> robot_loc;

  while (ros::ok() ) {
	looprate.sleep();

	std::unique_lock<std::mutex> data_lock(data_mutex_);
	pts = MapPts_;
	robot_loc = CurrentLocations_;
	data_lock.unlock();

	EP.PartialUpdateMap(pts);
	std::vector<Locations_c> goals = EP.NewGoals(robot_loc);

	nav_msgs::Path goal_list;
	goal_list.poses.resize(2);

	goal_list.poses[0].position.x = goals[0].x;
	goal_list.poses[0].position.y = goals[0].y;
	goal_list.poses[0].position.z = goals[0].z;
	goal_list.poses[0].orientation.y = goals[0].theta;  //TODO fix this

	goal_list.poses[1].position.x = goals[1].x;
	goal_list.poses[1].position.y = goals[1].y;
	goal_list.poses[1].position.z = goals[1].z;
	goal_list.poses[1].orientation.y = goals[1].theta;  //TODO fix this

	Goal_pub_.publish(goal_list);
  }
}

void EP_wrapper::PoseCallback(const nav_msgs::Path& msg) {
  Locations_c SegLoc, HexaLoc;

  SegLoc.x = msg->poses[0].pose.position.x;
  SegLoc.y = msg->poses[0].pose.position.y;
  SegLoc.z = msg->poses[0].pose.position.z;
  SegLoc.theta = msg->poses[0].pose.position.theta;

  HexaLoc.x = msg->poses[1].pose.position.x;
  HexaLoc.y = msg->poses[1].pose.position.y;
  HexaLoc.z = msg->poses[1].pose.position.z;
  HexaLoc.theta = msg->poses[1].pose.position.theta;

  std::unique_lock<std::mutex> data_lock(data_mutex_);
  CurrentLocations_.clear();
  CurrentLocations_.push_back(SegLoc);
  CurrentLocations_.push_back(HexaLoc);
  data_lock.unlock();
}

void EP_wrapper::MapCallback(const  pcl::PointCloud<pcl::PointXYZI>::ConstPtr& msg) {
  std::unique_lock<std::mutex> data_lock(data_mutex_);
  MapElement_c pt;
  for (int pidx=0; pidx < msg->points.size(); pidx++) {
	pt.x = msg->points[pidx].x;
	pt.y = msg->points[pidx].y;
	pt.z = msg->points[pidx].z;
	pt.data = msg->points[pidx].intensity;

	MapPts_.push_back(pt);
  }
  data_lock.unlock();
}



void EP_wrapper::init(void) {
  Robot_c Segbot, Hexa;


  while(!ros::param::has("scale")) {ros::Duration(0.1).sleep();}
  nh.param<double>("scale", scale, 20);

  nh.param<double>("segbot/motionheight", Segbot.MotionHeight_, 0.0*scale);
  nh.param<double>("segbot/sensorheight", Segbot.SensorHeight_, 0.5*scale);
  nh.param<double>("segbot/horizontalfov", Segbot.HorizontalFOV_, M_PI/8);
  nh.param<double>("segbot/verticalfov", Segbot.VerticalFOV_, M_PI/10);
  nh.param<double>("segbot/detectionrange", Segbot.DetectionRange_, 5.0*scale);
  nh.param<double>("segbot/circularsize", Segbot.CircularSize_, 1.0*scale);
  nh.param<std::string>("segbot/name", Segbot.name, "Segbot");

  nh.param<double>("hexa/motionheight", Hexa.MotionHeight_, 1.2*scale);
  nh.param<double>("hexa/sensorheight", Hexa.SensorHeight_, 1.2*scale);
  nh.param<double>("hexa/horizontalfov", Hexa.HorizontalFOV_, M_PI/8);
  nh.param<double>("hexa/verticalfov", Hexa.VerticalFOV_, M_PI/10);
  nh.param<double>("hexa/detectionrange", Hexa.DetectionRange_, 5.0*scale);
  nh.param<double>("hexa/circularsize", Hexa.CircularSize_, 1.0*scale);
  nh.param<std::string>("hexa/name", Hexa.name, "Hexa");

  params.robots.push_back(Segbot);
  params.robots.push_back(Hexa);


  nh.param<int>("sizex", params.size_x, 500);
  nh.param<int>("sizey", params.size_y, 500);
  nh.param<int>("sizez", params.size_z, 50);
  nh.param<int>("objectmaxelev", params.ObjectMaxElev, 1.5*scale);
  nh.param<int>("obsvalue", params.obs, 100);
  nh.param<int>("freevalue", params.freespace, 50);
  nh.param<int>("unkvalue", params.unk, 0);
  nh.param<int>("numangles", params.NumAngles, 16);
  nh.param<int>("mindist", params.MinDist, 1.2*scale);

  EP.Init(params);

  nh.param<std::string>("goal_topic", nav_msgs::Path, "goal_list");
  nh.param<std::string>("map_topic", sensor_msgs::Pointcloud2, "combined_map");
  nh.param<std::string>("pose_topic", sensor_msgs::Pointcloud2, "combined_pose");

  EP_thread = new std::thread(&EP_wrapper::plannerthread, this);

  nh.subscribe(pose_topic, 1, &EP_wrapper::PoseCallback, this);
  nh.subscribe(map_topic, 5, &EP_wrapper::MapCallback, this);

}



int main(int argc, char* argv) {
  nh = ros::init(argc, argv, "Exploration");
  EP_wrapper EPW;
  EPW.init();

  while (ros::ok() ) {
	ros.spin();
  }
  EP_thread.join();
}

