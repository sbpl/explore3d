/// Jonathan Butzke
///(c) 2014 Ros wrapper header
#include "exploration.hpp"
#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/transform_listener.h>

class EP_wrapper
{
public:
	EP_wrapper();
	void plannerthread(void);
	void init(void);
	void PoseCallback(const nav_msgs::PathConstPtr& msg);
	void MapCallback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& msg);
	~EP_wrapper();

private:
	ExplorationPlanner EP;
	ExpParams_c params;
	std::vector<Locations_c> CurrentLocations_;
	std::vector<MapElement_c> MapPts_;

	template<typename T> bool bounds_check(const T & point);

	void quaternion_to_yaw(const geometry_msgs::Quaternion & quat, double & yaw);

	void yaw_to_quaternion(const double & yaw, geometry_msgs::Quaternion & quat);

	int continuous_angle_to_discrete(double cont, double res);

	int discrete_anngle_to_continuous(int disc, double res);

	int continuous_to_discrete(double cont, double res);

	double disrete_to_continuous(int disc, double res);

	void publish_goal_list(const std::vector<Locations_c> & goal_list);

	void publish_point_cloud(const std::vector<pcl::PointXYZI> & points, const ros::Publisher & publisher);

	double scale;
	double planner_rate;
	double origin_x;
	double origin_y;
	double origin_z;
	double resolution;
	double angle_resolution;
	int size_x;
	int size_y;
	int size_z;
	std::string frame_id;
	bool got_first_map_update;
	bool got_first_pose_update;

	std::mutex data_mutex_;
	std::thread *EP_thread_;
	ros::Publisher Goal_pub_;
	ros::Publisher Goal_point_cloud_pub;
	ros::Subscriber Map_sub_, Pose_sub_;
	std::string goal_topic_, map_topic_, pose_topic_, goal_point_cloud_topic;
	ros::NodeHandle nh;
	ros::NodeHandle ph;
};


