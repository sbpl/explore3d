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

  int segbot_motionheight, segbot_sensorheight, segbot_detectionrange, segbot_circularsize, hexa_motionheight, hexa_sensorheight, hexa_detectionrange, hexa_circularsize;
  nh.param<int>("segbot/motionheight", segbot_motionheight, (int)(0.0*scale)); // height to use for obstacles during motionheight (cells)
  Segbot.MotionHeight_ = (uint) segbot_motionheight;
  nh.param<int>("segbot/sensorheight", segbot_sensorheight, (int)0.5*scale);  // height sensor is at (cells)
  Segbot.SensorHeight_ = (uint) segbot_sensorheight;
  nh.param<double>("segbot/horizontalfov", Segbot.HorizontalFOV_, M_PI/8);
  nh.param<double>("segbot/verticalfov", Segbot.VerticalFOV_, M_PI/10);
  nh.param<int>("segbot/detectionrange", segbot_detectionrange, (int)5.0*scale);		// how far the sensor works (cells)
  Segbot.DetectionRange_ = (uint) segbot_detectionrange;
  nh.param<int>("segbot/circularsize", segbot_circularsize, (uint)1.0*scale);			// how big the robot is (cells)
  Segbot.CircularSize_ = (uint) segbot_circularsize;
  nh.param<std::string>("segbot/name", Segbot.name, "Segbot");
  nh.param<int>("hexa/motionheight", hexa_motionheight, (int)1.2*scale);
  Hexa.MotionHeight_ = (uint) hexa_motionheight;
  nh.param<int>("hexa/sensorheight",hexa_sensorheight , (int)1.2*scale);
  Hexa.SensorHeight_ = hexa_sensorheight;
  nh.param<double>("hexa/horizontalfov", Hexa.HorizontalFOV_, M_PI/8);
  nh.param<double>("hexa/verticalfov", Hexa.VerticalFOV_, M_PI/10);
  nh.param<int>("hexa/detectionrange",hexa_detectionrange , (int)5.0*scale);
  Hexa.DetectionRange_ = hexa_detectionrange;
  nh.param<int>("hexa/circularsize", hexa_circularsize, (int)1.0*scale);
  Hexa.CircularSize_ = hexa_circularsize;
  nh.param<std::string>("hexa/name", Hexa.name, "Hexa");

  params.robots.push_back(Segbot);
  params.robots.push_back(Hexa);

  int sizex, sizey, sizez, objectmaxelev,obs, freespace, unk, numangles, mindist;
  nh.param<int>("sizex", sizex, 500);
  params.size_x = sizex;
  nh.param<int>("sizey", sizey, 500);
  params.size_y = sizey;
  nh.param<int>("sizez", sizez, 50);
  params.size_z = sizez;
  nh.param<int>("objectmaxelev", objectmaxelev, (uint) 1.5*scale);				// max height to consider for the OOI (cells)
  params.ObjectMaxElev = objectmaxelev;
  nh.param<int>("obsvalue", obs, 100);									// values for obstacles, freespace, unknown
  params.obs = obs;
  nh.param<int>("freevalue", freespace, 50);
  params.freespace = freespace;
  nh.param<int>("unkvalue", unk, 0);
  params.unk = unk;
  nh.param<int>("numangles", numangles, 16);							// number of thetas
  params.NumAngles = numangles;
  nh.param<int>("mindist", mindist, (uint)1.2*scale);					// closest robots should operate without penalty (cells)
  params.MinDist = mindist;

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

