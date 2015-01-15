#include "exploration_executive.hpp"

namespace utacc
{

ExplorationExecutive::ExplorationExecutive() :
    nh_(),
    ph_("~"),
    goal_pub_(),
    map_sub_(),
    pose_sub_(),
    exploration_thread_(),
    goal_pending_(false)
{

}

ExplorationExecutive::~ExplorationExecutive()
{

}

int ExplorationExecutive::run()
{
    if (!initialize())
    {
        return 1;
    }

    if (exploration_thread_.run() != 0) {
        return 2;
    }

    ros::Rate executive_rate(executive_rate_hz_);
    bool last_goal_pending = false;
    while (ros::ok())
    {
        ros::spinOnce(); // process incoming messages

        if (!goal_pending_) {
            if (last_goal_pending) { // just finished a goal
                goal_pub_.publish(goals_);
                exploration_thread_.publish_maps();
            }
            exploration_thread_.compute_goals(std::bind(&ExplorationExecutive::goals_callback, this, std::placeholders::_1));
        }

        last_goal_pending = goal_pending_;
        executive_rate.sleep();
    }
    return 0;
}

bool ExplorationExecutive::initialize()
{
    if (!exploration_thread_.initialize())
    {
        ROS_ERROR("Failed to initialize Exploration Thread");
        return false;
    }

    goal_pub_ = nh_.advertise<nav_msgs::Path>("goal_topic", 1);
    pose_sub_ = nh_.subscribe("combined_pose", 1, &ExplorationExecutive::pose_callback, this);
    map_sub_ = nh_.subscribe("combined_map", 20, &ExplorationExecutive::map_callback, this);

    ph_.param("executive_rate", executive_rate_hz_, 0.5);

    return true;
}

void ExplorationExecutive::pose_callback(const nav_msgs::PathConstPtr& msg)
{
    exploration_thread_.update_poses(msg);
}

void ExplorationExecutive::map_callback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& msg)
{
    exploration_thread_.update_map(msg);
}

void ExplorationExecutive::goals_callback(nav_msgs::Path& goals)
{
    goals_ = goals;
    goal_pending_ = false;
}

} // namespace utacc

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Exploration");
    return utacc::ExplorationExecutive().run();
}

