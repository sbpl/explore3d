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
	//TODO: this is inefficient - change pointers or something
	pts = MapPts_;
	robot_loc = CurrentLocations_;
	data_lock.unlock();

	EP.PartialUpdateMap(pts);
	std::vector<Locations_c> goals = EP.NewGoals(robot_loc);

	nav_msgs::Path goal_list;
	goal_list.poses.resize(2);

	goal_list.poses[0].pose.position.x = goals[0].x;
	goal_list.poses[0].pose.position.y = goals[0].y;
	goal_list.poses[0].pose.position.z = goals[0].z;
	goal_list.poses[0].pose.orientation.y = goals[0].theta;  //TODO fix this - needs to be yaw

	goal_list.poses[1].pose.position.x = goals[1].x;
	goal_list.poses[1].pose.position.y = goals[1].y;
	goal_list.poses[1].pose.position.z = goals[1].z;
	goal_list.poses[1].pose.orientation.y = goals[1].theta;  //TODO fix this - needs to be yaw

	Goal_pub_.publish(goal_list);
  }
}

void EP_wrapper::PoseCallback(const nav_msgs::Path& msg) {
  Locations_c SegLoc, HexaLoc;

  SegLoc.x = msg.poses[0].pose.position.x;
  SegLoc.y = msg.poses[0].pose.position.y;
  SegLoc.z = msg.poses[0].pose.position.z;
  //SegLoc.theta = msg.poses[0].pose.position.theta;//TODO fix this - needs to be yaw

  HexaLoc.x = msg.poses[1].pose.position.x;
  HexaLoc.y = msg.poses[1].pose.position.y;
  HexaLoc.z = msg.poses[1].pose.position.z;
  //HexaLoc.theta = msg.poses[1].pose.position.theta;//TODO fix this - needs to be yaw

  std::unique_lock<std::mutex> data_lock(data_mutex_);
  CurrentLocations_.clear();
  CurrentLocations_.push_back(SegLoc);
  CurrentLocations_.push_back(HexaLoc);
  data_lock.unlock();
}

void EP_wrapper::MapCallback(const  pcl::PointCloud<pcl::PointXYZI>::ConstPtr& msg) {
  std::unique_lock<std::mutex> data_lock(data_mutex_);
  //TODO: this is inefficient
  MapElement_c pt;
  for (size_t pidx=0; pidx < msg->points.size(); pidx++) {
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

  nh.param<uint>("segbot/motionheight", Segbot.MotionHeight_, (uint)(0.0*scale)); // height to use for obstacles during motionheight (cells)
  nh.param<uint>("segbot/sensorheight", Segbot.SensorHeight_, (uint)0.5*scale);  // height sensor is at (cells)
  nh.param<double>("segbot/horizontalfov", Segbot.HorizontalFOV_, M_PI/8);
  nh.param<double>("segbot/verticalfov", Segbot.VerticalFOV_, M_PI/10);
  nh.param<uint>("segbot/detectionrange", Segbot.DetectionRange_, (uint)5.0*scale);		// how far the sensor works (cells)
  nh.param<uint>("segbot/circularsize", Segbot.CircularSize_, (uint)1.0*scale);			// how big the robot is (cells)
  nh.param<std::string>("segbot/name", Segbot.name, "Segbot");

  nh.param<uint>("hexa/motionheight", Hexa.MotionHeight_, (uint)1.2*scale);
  nh.param<uint>("hexa/sensorheight", Hexa.SensorHeight_, (uint)1.2*scale);
  nh.param<double>("hexa/horizontalfov", Hexa.HorizontalFOV_, M_PI/8);
  nh.param<double>("hexa/verticalfov", Hexa.VerticalFOV_, M_PI/10);
  nh.param<uint>("hexa/detectionrange", Hexa.DetectionRange_, (uint)5.0*scale);
  nh.param<uint>("hexa/circularsize", Hexa.CircularSize_, (uint)1.0*scale);
  nh.param<std::string>("hexa/name", Hexa.name, "Hexa");

  params.robots.push_back(Segbot);
  params.robots.push_back(Hexa);

  nh.param<uint>("sizex", params.size_x, 500);
  nh.param<uint>("sizey", params.size_y, 500);
  nh.param<uint>("sizez", params.size_z, 50);
  nh.param<uint>("objectmaxelev", params.ObjectMaxElev, (uint) 1.5*scale);				// max height to consider for the OOI (cells)
  nh.param<uint>("obsvalue", params.obs, 100);									// values for obstacles, freespace, unknown
  nh.param<uint>("freevalue", params.freespace, 50);
  nh.param<uint>("unkvalue", params.unk, 0);
  nh.param<uint>("numangles", params.NumAngles, 16);							// number of thetas
  nh.param<uint>("mindist", params.MinDist, (uint)1.2*scale);					// closest robots should operate without penalty (cells)

  nh.param<std::string>("goal_topic", goal_topic_, "goal_list");
  nh.param<std::string>("map_topic", map_topic_, "combined_map");
  nh.param<std::string>("pose_topic", pose_topic_, "combined_pose");

  EP.Init(params);
  EP_thread_ = new std::thread(&EP_wrapper::plannerthread, this);

  nh.subscribe(pose_topic_, 1, &EP_wrapper::PoseCallback, this);
  nh.subscribe(map_topic_, 5, &EP_wrapper::MapCallback, this);

}

EP_wrapper::~EP_wrapper() {
  EP_thread_->join();
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "Exploration");
  EP_wrapper EPW;
  EPW.init();

  while (ros::ok() ) {
	ros::spin();
  }
}

