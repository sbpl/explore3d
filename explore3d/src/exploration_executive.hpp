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

    enum ExplorationExecutiveStatus
    {
        INVALID = -1,
        WAIT,
        IDLE,
        MOVING_TO_START,
        EXPLORING,
        NUM_STATUSES
    };

    static std::string to_string(ExplorationExecutiveStatus status);

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    ros::Publisher goal_pub_;

    ros::Subscriber map_sub_;
    ros::Subscriber pose_sub_;

    ros::Publisher planner_status_pub_;
    int status_seqno_;

    ros::Subscriber e2_async_info_sub_;

    ExplorationThread exploration_thread_;

    double executive_rate_hz_;
    nav_msgs::Path goals_;

    ExplorationExecutiveStatus last_status_;
    ExplorationExecutiveStatus curr_status_;
    ExplorationExecutiveStatus next_status_;

    utacc_msgs::MissionCommand::ConstPtr last_mission_command_;
    nav_msgs::Path::ConstPtr last_poses_;

    std::vector<void (ExplorationExecutive::*)()> on_enter_state_methods_;
    std::vector<void (ExplorationExecutive::*)()> on_state_methods_;
    std::vector<void (ExplorationExecutive::*)()> on_exit_state_methods_;

    /// @name "Exploring" state variables
    /// @{
    int goalid_;
    bool goal_pending_;
    /// @}

    bool initialize();
    void pose_callback(const nav_msgs::PathConstPtr& msg);
    void map_callback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& msg);

    void goals_callback(nav_msgs::Path& goals);
    void e2_async_info_callback(const utacc_msgs::MissionCommand::ConstPtr& msg);

    void onWaitEnter();
    void onWait();
    void onWaitExit();
    void onIdleEnter();
    void onIdle();
    void onIdleExit();
    void onMovingToStartEnter();
    void onMovingToStart();
    void onMovingToStartExit();
    void onExploringEnter();
    void onExploring();
    void onExploringExit();
};

} // namespace utacc

#endif
