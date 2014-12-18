///Jonathan Michael Butzke
///(c) 2014
///ROS Wrapper for 3-D exploration module

#include "exploration_main.hpp"

void EP_wrapper::plannerthread(void)
{
	ros::Rate looprate(planner_rate);
	std::vector<MapElement_c> pts;
	std::vector<Locations_c> robot_loc;

	while (ros::ok())
	{

		looprate.sleep();

		//do not execute planner until first map and pose updates are received
		if (!(got_first_map_update && got_first_pose_update))
		{
			ROS_INFO("planner_thread: waiting for first map and pose update\n");
			continue;
		}

		//update map and poses
		std::unique_lock < std::mutex > data_lock(data_mutex_);
		ROS_INFO("update map and poses, start\n");
		pts = MapPts_;
		robot_loc = CurrentLocations_;
		data_lock.unlock();
		EP.PartialUpdateMap(pts);

		ROS_INFO("update map and poses, done\n");

		//retrieve goals
		ROS_INFO("get goals, start\n");
		std::vector<Locations_c> goals = EP.NewGoals(robot_loc);
		ROS_INFO("get goals, done\n");

		publish_goal_list(goals);

		publish_planner_map();

		//clear the map points
		MapPts_.clear();

	}
}

void EP_wrapper::PoseCallback(const nav_msgs::PathConstPtr& msg)
{
	Locations_c SegLoc, HexaLoc;

	auto poses = msg->poses;

	if (poses.size() < 2)
	{
		ROS_ERROR("ERROR poses list provided is size %d when it should be size %d\n", (int ) poses.size(), 2);
	}

	//Convert poses from world continuous to map discrete
	double yaw;
	SegLoc.x = continuous_to_discrete(poses[0].pose.position.x - origin_x, resolution);
	SegLoc.y = continuous_to_discrete(poses[0].pose.position.y - origin_y, resolution);
	SegLoc.z = continuous_to_discrete(poses[0].pose.position.z - origin_z, resolution);
	quaternion_to_yaw(poses[0].pose.orientation, yaw);
	SegLoc.theta = continuous_angle_to_discrete(yaw, angle_resolution);

	HexaLoc.x = continuous_to_discrete(poses[1].pose.position.x - origin_x, resolution);
	HexaLoc.y = continuous_to_discrete(poses[1].pose.position.y - origin_y, resolution);
	HexaLoc.z = continuous_to_discrete(poses[1].pose.position.z - origin_z, resolution);
	quaternion_to_yaw(poses[1].pose.orientation, yaw);
	HexaLoc.theta = continuous_angle_to_discrete(yaw, angle_resolution);

	bool hexa_bound_check = true;
	bool segbot_bound_check = true;

	//bounds check both robot locationss
	if (!bounds_check(SegLoc))
	{
		segbot_bound_check = false;
		ROS_ERROR("ERROR: segbot location at discrete location %d %d %d outside of mapbounds %u %u %u", SegLoc.x, SegLoc.y, SegLoc.z, size_x, size_y,
				size_y);
	}
	if (!bounds_check(HexaLoc))
	{
		hexa_bound_check = false;
		ROS_ERROR("ERROR: hexa location at discrete location %d %d %d outside of mapbounds %u %u %u", HexaLoc.x, HexaLoc.y, HexaLoc.z, size_x, size_y,
				size_y);
	}

	// don't update locations if one is outside map bounds
	if (!(hexa_bound_check && segbot_bound_check))
	{
		return;
	}

	std::unique_lock < std::mutex > data_lock(data_mutex_);
	CurrentLocations_.clear();
	CurrentLocations_.push_back(SegLoc);
	CurrentLocations_.push_back(HexaLoc);
	data_lock.unlock();
	got_first_pose_update = true;
}

void EP_wrapper::MapCallback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& msg)
{
	ROS_INFO("map upate callback for seq %d\n", (int ) msg->header.seq);
	std::unique_lock < std::mutex > data_lock(data_mutex_);
	//TODO: this is inefficient
	MapElement_c pt;
	for (size_t pidx = 0; pidx < msg->points.size(); pidx++)
	{
		//convert coordinates to discrete in map frame
		pt.x = continuous_to_discrete(msg->points[pidx].x - origin_x, resolution);
		pt.y = continuous_to_discrete(msg->points[pidx].y - origin_y, resolution);
		pt.z = continuous_to_discrete(msg->points[pidx].z - origin_z, resolution);
		pt.data = msg->points[pidx].intensity;

		//dont add points outside of map bounds
		if (!bounds_check(pt))
		{
			continue;
		}

		MapPts_.push_back(pt);
	}
	data_lock.unlock();
	ROS_INFO("map upate callback finished\n");
	got_first_map_update = true;
}

void EP_wrapper::init(void)
{
	Robot_c Segbot, Hexa;

	got_first_map_update = false;
	got_first_pose_update = false;

//  while(!ros::param::has("scale")) {ros::Duration(0.1).sleep();}
	ph.param<double>("planner_rate", planner_rate, 0.5);
	ph.param<double>("scale", scale, 20);

	int segbot_motionheight, segbot_sensorheight, segbot_detectionrange, segbot_circularsize, hexa_motionheight, hexa_sensorheight,
			hexa_detectionrange, hexa_circularsize;
	ph.param<int>("segbot/motionheight", segbot_motionheight, (int) (0.0 * scale)); // height to use for obstacles during motionheight (cells)
	Segbot.MotionHeight_ = (uint) segbot_motionheight;
	ph.param<int>("segbot/sensorheight", segbot_sensorheight, (int) 0.5 * scale); // height sensor is at (cells)
	Segbot.SensorHeight_ = (uint) segbot_sensorheight;
	ph.param<double>("segbot/horizontalfov", Segbot.HorizontalFOV_, M_PI / 8);
	ph.param<double>("segbot/verticalfov", Segbot.VerticalFOV_, M_PI / 10);
	ph.param<int>("segbot/detectionrange", segbot_detectionrange, (int) 5.0 * scale); // how far the sensor works (cells)
	Segbot.DetectionRange_ = (uint) segbot_detectionrange;
	ph.param<int>("segbot/circularsize", segbot_circularsize, (uint) 1.0 * scale); // how big the robot is (cells)
	Segbot.CircularSize_ = (uint) segbot_circularsize;
	ph.param<std::string>("segbot/name", Segbot.name, "Segbot");
	ph.param<int>("hexa/motionheight", hexa_motionheight, (int) 1.2 * scale);
	Hexa.MotionHeight_ = (uint) hexa_motionheight;
	ph.param<int>("hexa/sensorheight", hexa_sensorheight, (int) 1.2 * scale);
	Hexa.SensorHeight_ = hexa_sensorheight;
	ph.param<double>("hexa/horizontalfov", Hexa.HorizontalFOV_, M_PI / 8);
	ph.param<double>("hexa/verticalfov", Hexa.VerticalFOV_, M_PI / 10);
	ph.param<int>("hexa/detectionrange", hexa_detectionrange, (int) 5.0 * scale);
	Hexa.DetectionRange_ = hexa_detectionrange;
	ph.param<int>("hexa/circularsize", hexa_circularsize, (int) 1.0 * scale);
	Hexa.CircularSize_ = hexa_circularsize;
	ph.param<std::string>("hexa/name", Hexa.name, "Hexa");

	params.robots.push_back(Segbot);
	params.robots.push_back(Hexa);

	int objectmaxelev, obs, freespace, unk, numangles, mindist;
	ph.param<std::string>("frame_id", frame_id, "/map");
	ph.param<int>("size_x", size_x, 500);
	params.size_x = size_x;
	ph.param<int>("size_y", size_y, 500);
	params.size_y = size_y;
	ph.param<int>("size_z", size_z, 50);
	params.size_z = size_z;
	ph.param<double>("origin_x", origin_x, 0);
	ph.param<double>("origin_y", origin_y, 0);
	ph.param<double>("origin_z", origin_z, 0);
	ph.param<double>("resolution", resolution, 0.5);

	ph.param<int>("objectmaxelev", objectmaxelev, (uint) 1.5 * scale); // max height to consider for the OOI (cells)
	params.ObjectMaxElev = objectmaxelev;
	ph.param<int>("obsvalue", obs, 100); // values for obstacles, freespace, unknown
	params.obs = obs;
	ph.param<int>("freevalue", freespace, 50);
	params.freespace = freespace;
	ph.param<int>("unkvalue", unk, 0);
	params.unk = unk;
	ph.param<int>("numangles", numangles, 16); // number of thetas
	params.NumAngles = numangles;
	ph.param<int>("mindist", mindist, (uint) 1.2 * scale); // closest robots should operate without penalty (cells)
	params.MinDist = mindist;

	angle_resolution = (2 * M_PI) / numangles;

	ph.param<std::string>("goal_topic", goal_topic_, "goal_list");
	ph.param<std::string>("map_topic", map_topic_, "combined_map");
	ph.param<std::string>("pose_topic", pose_topic_, "combined_pose");
	ph.param<std::string>("goal_point_cloud_topic", goal_point_cloud_topic, "goal_point_cloud");

	EP.Init(params);
	EP_thread_ = new std::thread(&EP_wrapper::plannerthread, this);

	ROS_ERROR("subscribed to %s and %s \n", pose_topic_.c_str(), map_topic_.c_str());
	ROS_ERROR("dims is %d %d %d\n", size_x, size_y, size_z);
	ROS_ERROR("resolution is %f\n", resolution);

	Pose_sub_ = nh.subscribe(pose_topic_, 1, &EP_wrapper::PoseCallback, this);
	Map_sub_ = nh.subscribe(map_topic_, 20, &EP_wrapper::MapCallback, this);
	Goal_pub_ = nh.advertise<nav_msgs::Path>(goal_topic_, 1);
	Goal_point_cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(goal_point_cloud_topic, 1);
	coverage_map_pub = ph.advertise<pcl::PointCloud<pcl::PointXYZI> >("coverage_map", 1);
	cost_map_pub = ph.advertise<pcl::PointCloud<pcl::PointXYZI> >("cost_map", 1);
	frontier_map_pub = ph.advertise<pcl::PointCloud<pcl::PointXYZI> >("frontier_map", 1);
	counts_map_pub = ph.advertise<pcl::PointCloud<pcl::PointXYZI> >("counts_map", 1);
}

EP_wrapper::EP_wrapper() :
		nh(), ph("~")
{
	init();
}

EP_wrapper::~EP_wrapper()
{
	EP_thread_->join();
}

template<typename T>
bool EP_wrapper::bounds_check(const T& point)
{
	if ((int) point.x < 0 || (int) point.x >= size_x || (int) point.y < 0 || (int) point.y >= size_y || (int) point.z < 0 || (int) point.z >= size_z)
	{
		return false;
	}
	return true;
}

int main(int argc, char** argv)
{
	ROS_ERROR("start\n");
	ros::init(argc, argv, "Exploration");
	ROS_ERROR("ros init done\n");
	EP_wrapper EPW;
	ROS_ERROR("made EP wrapper");

	while (ros::ok())
	{
		ros::spin();
	}
}

int EP_wrapper::continuous_to_discrete(double cont, double res)
{
	double v = cont / res;
	v >= 0 ? v = floor(v) : v = ceil(v - 1);
	int d = static_cast<int>(v);
	return d;
}

double EP_wrapper::discrete_to_continuous(int disc, double res)
{
	double s = (static_cast<double>(disc) * res) + (res / 2.0);
	return s;
}

void EP_wrapper::publish_goal_list(const std::vector<Locations_c>& goals)
{
	nav_msgs::Path goal_list;
	goal_list.header.frame_id = frame_id;
	goal_list.header.stamp = ros::Time::now();
	goal_list.poses.resize(2);

	//convert goals to world frame, continuous
	goal_list.poses[0].pose.position.x = discrete_to_continuous(goals[0].x, resolution) + origin_x;
	goal_list.poses[0].pose.position.y = discrete_to_continuous(goals[0].y, resolution) + origin_y;
	goal_list.poses[0].pose.position.z = discrete_to_continuous(goals[0].z, resolution) + origin_z;
	yaw_to_quaternion(discrete_anngle_to_continuous(goals[0].theta, angle_resolution), goal_list.poses[0].pose.orientation);

	goal_list.poses[1].pose.position.x = discrete_to_continuous(goals[1].x, resolution) + origin_x;
	goal_list.poses[1].pose.position.y = discrete_to_continuous(goals[1].y, resolution) + origin_y;
	goal_list.poses[1].pose.position.z = discrete_to_continuous(goals[1].z, resolution) + origin_z;
	yaw_to_quaternion(discrete_anngle_to_continuous(goals[1].theta, angle_resolution), goal_list.poses[1].pose.orientation);

	//publish goals as path
	Goal_pub_.publish(goal_list);

	//convert to list of points and publish point cloud
	std::vector<pcl::PointXYZI> points;
	for (auto & pose : goal_list.poses)
	{
		pcl::PointXYZI p;
		p.x = pose.pose.position.x;
		p.y = pose.pose.position.y;
		p.z = pose.pose.position.z;
		points.push_back(p);
	}
	publish_point_cloud(points, Goal_point_cloud_pub);
}

void EP_wrapper::quaternion_to_yaw(const geometry_msgs::Quaternion& quat, double& yaw)
{
	tf::Quaternion tf_quat;
	tf::quaternionMsgToTF(quat, tf_quat);
	yaw = tf::getYaw(tf_quat);
}

void EP_wrapper::yaw_to_quaternion(const double& yaw, geometry_msgs::Quaternion& quat)
{
	const geometry_msgs::Quaternion q;
	tf::Quaternion tf_quat = tf::createQuaternionFromRPY(0, 0, yaw);
	tf::quaternionTFToMsg(tf_quat, quat);
}

int EP_wrapper::continuous_angle_to_discrete(double cont, double res)
{
	double pi = M_PI;
	//set between 2pi and 0
	if (cont < pi)
	{
		cont = pi - cont;
	}
	double scaled = cont / res;
	int d = static_cast<int>(round(scaled));
	return d;
}

int EP_wrapper::discrete_anngle_to_continuous(int disc, double res)
{
	double scaled = static_cast<double>(disc);
	return scaled * res;
}

void EP_wrapper::publish_point_cloud(const std::vector<pcl::PointXYZI>& points, const ros::Publisher& publisher)
{
	//make point cloud for goals
	pcl::PointCloud<pcl::PointXYZI> cloud;
	cloud.header.stamp = ros::Time::now();
	cloud.header.frame_id = frame_id;
	cloud.height = 1;

	cloud.points.reserve(points.size());
	for (auto & p : points)
	{
		cloud.points.push_back(p);
	}
	cloud.width = cloud.points.size();
	publisher.publish(cloud);
}

void EP_wrapper::publish_planner_map()
{

	ROS_INFO("publishing maps....");
	std::vector<pcl::PointXYZI> coverage_points;
	get_point_cloud_from_map(EP.coverage_.map_, coverage_points);
	publish_point_cloud(coverage_points, coverage_map_pub);

	std::vector<pcl::PointXYZI> cost_points;
	get_point_cloud_from_map(EP.CostToPts_.at(0), cost_points);
	publish_point_cloud(cost_points, cost_map_pub);

	std::vector<pcl::PointXYZI> frontier_points;
	get_point_cloud_from_points(EP.Frontier3d_, frontier_points);
	publish_point_cloud(frontier_points, frontier_map_pub);

	std::vector<pcl::PointXYZI> count_points;
	get_point_cloud_from_map(EP.counts_.at(0), count_points);
	publish_point_cloud(count_points, counts_map_pub);
	ROS_INFO("done\n");
}

template<typename T>
void EP_wrapper::get_point_cloud_from_map(const std::vector<T> &map, std::vector<pcl::PointXYZI> & points)
{
	std::vector<int> indicies;
	get_point_cloud_from_inner_dim(map, points, indicies, 0);
}

template<typename T>
void EP_wrapper::get_point_cloud_from_inner_dim(const std::vector<T>& map, std::vector<pcl::PointXYZI>& points, std::vector<int> & indicies,
		int depth)
{
	size_t dim_size = map.size();
	for (size_t s = 0; s < dim_size; s++)
	{
		if (indicies.size() < (size_t) depth + 1)
		{
			indicies.push_back(0);
		}
		indicies[depth] = s;
		get_point_cloud_from_inner_dim(map.at(s), points, indicies, depth + 1);
	}
}

template<typename T>
void EP_wrapper::get_point_cloud_from_inner_dim(const T& val, std::vector<pcl::PointXYZI>& points, std::vector<int> & indicies, int depth)
{
	static int count = 0;
	count++;
	pcl::PointXYZI point;
	if (depth > 0)
	{
		point.x = discrete_to_continuous(indicies[0], resolution) + origin_x;
	}

	if (depth > 1)
	{
		point.y = discrete_to_continuous(indicies[1], resolution) + origin_y;
	}

	if (depth > 2)
	{
		point.z = discrete_to_continuous(indicies[2], resolution) + origin_z;
	}
	point.intensity = static_cast<double>(val);
	if (point.intensity > 0)
	{
		points.push_back(point);
	}
}

template<typename T>
inline void EP_wrapper::get_point_cloud_from_points(const std::vector<T>& point_list, std::vector<pcl::PointXYZI>& points)
{
	//convert to 3d points
	for (auto & lp : point_list)
	{
		pcl::PointXYZI p;
		p.x = discrete_to_continuous(lp.x, resolution) + origin_x;
		p.y = discrete_to_continuous(lp.y, resolution) + origin_y;
		p.z = discrete_to_continuous(lp.z, resolution) + origin_z;
		p.intensity = lp.cost;
		points.push_back(p);
	}

}
