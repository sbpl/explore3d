#include "exploration_executive.hpp"

#include <utacc_msgs/MissionStatus.h>

#define CALL_MEMFN(obj, memfn, ...) ((obj)->*(memfn))(##__VA_ARGS__)

namespace utacc
{

ExplorationExecutive::ExplorationExecutive() :
    nh_(),
    ph_("~"),
    goal_pub_(),
    map_sub_(),
    pose_sub_(),
    planner_status_pub_(),
    status_seqno_(0),
    e2_async_info_sub_(),
    exploration_thread_(),
    last_status_(INVALID),
    curr_status_(INVALID),
    next_status_(INVALID),
    last_mission_command_(),
    last_poses_(),
    goalid_(0),
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
    while (ros::ok())
    {
        ros::spinOnce(); // process incoming messages

        auto entering_state = [this](){ return this->last_status_ != this->curr_status_; };
        auto exiting_state  = [this](){ return this->curr_status_ != this->next_status_; };

        ROS_INFO("Spinning: %s", to_string(curr_status_).c_str());

        // state pump
        if (entering_state()) {
            assert(on_state_enter_methods_[curr_status_]);
            CALL_MEMFN(this, on_enter_state_methods_[curr_status_]);
        }
            assert(on_state_methods_[curr_status_]);
        CALL_MEMFN(this, on_state_methods_[curr_status_]);
        if (exiting_state()) {
            assert(on_exit_state_methods_[curr_status_]);
            CALL_MEMFN(this, on_exit_state_methods_[curr_status_]);
        }
        // end state pump

        if (exiting_state()) {
            ROS_INFO("Exploration Planner Status: %s -> %s", to_string(curr_status_).c_str(), to_string(next_status_).c_str());
        }

        last_status_ = curr_status_;
        curr_status_ = next_status_;

        executive_rate.sleep();
    }
    return 0;
}

std::string ExplorationExecutive::to_string(ExplorationExecutiveStatus status)
{
    switch (status)
    {
    case INVALID:
        return "INVALID";
    case WAIT:
        return "WAIT";
    case IDLE:
        return "IDLE";
    case MOVING_TO_START:
        return "MOVING_TO_START";
    case EXPLORING:
        return "EXPLORING";
    default:
        return "UNRECOGNIZED";
    }
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

    planner_status_pub_ = nh_.advertise<utacc_msgs::MissionStatus>("planner_status", 5);
    e2_async_info_sub_ = nh_.subscribe("e2_async_info", 1, &ExplorationExecutive::e2_async_info_callback, this);

    ph_.param("executive_rate", executive_rate_hz_, 0.5);

    last_status_ = WAIT;
    curr_status_ = WAIT;
    next_status_ = WAIT;

    on_enter_state_methods_.resize(NUM_STATUSES, nullptr);
    on_state_methods_.resize(NUM_STATUSES, nullptr);
    on_exit_state_methods_.resize(NUM_STATUSES, nullptr);

    on_enter_state_methods_[WAIT] = &ExplorationExecutive::onWaitEnter;
    on_enter_state_methods_[IDLE] = &ExplorationExecutive::onIdleEnter;
    on_enter_state_methods_[MOVING_TO_START] = &ExplorationExecutive::onMovingToStartEnter;
    on_enter_state_methods_[EXPLORING] = &ExplorationExecutive::onExploringEnter;

    on_state_methods_[WAIT] = &ExplorationExecutive::onWait;
    on_state_methods_[IDLE] = &ExplorationExecutive::onIdle;
    on_state_methods_[MOVING_TO_START] = &ExplorationExecutive::onMovingToStart;
    on_state_methods_[EXPLORING] = &ExplorationExecutive::onExploring;

    on_exit_state_methods_[WAIT] = &ExplorationExecutive::onWaitExit;
    on_exit_state_methods_[IDLE] = &ExplorationExecutive::onIdleExit;
    on_exit_state_methods_[MOVING_TO_START] = &ExplorationExecutive::onMovingToStartExit;
    on_exit_state_methods_[EXPLORING] = &ExplorationExecutive::onExploringExit;

    return true;
}

void ExplorationExecutive::pose_callback(const nav_msgs::PathConstPtr& msg)
{
    last_poses_ = msg;
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

void ExplorationExecutive::e2_async_info_callback(const utacc_msgs::MissionCommand::ConstPtr& msg)
{
    last_mission_command_ = msg;
}

void ExplorationExecutive::onWaitEnter()
{
    last_mission_command_.reset();
}

void ExplorationExecutive::onWait()
{
    if (exploration_thread_.ready_to_plan()) {
        next_status_ = IDLE;
    }
}

void ExplorationExecutive::onWaitExit()
{

}

void ExplorationExecutive::onIdleEnter()
{
    last_mission_command_.reset();
}

void ExplorationExecutive::onIdle()
{
    utacc_msgs::MissionStatus idle_status;
    idle_status.header.seq = status_seqno_++;
    idle_status.header.stamp = ros::Time::now();
    idle_status.header.frame_id = "";
    idle_status.address = utacc_msgs::MissionCodes::E2_UUID;
    idle_status.msg = utacc_msgs::MissionCodes::IDLE;
    idle_status.battery_pct = 100.0f;
    planner_status_pub_.publish(idle_status);

    if (last_mission_command_ && last_mission_command_->address == utacc_msgs::MissionCodes::MMEP_UUID) {
        next_status_ = MOVING_TO_START;
    }
}

void ExplorationExecutive::onIdleExit()
{

}

void ExplorationExecutive::onMovingToStartEnter()
{

}

void ExplorationExecutive::onMovingToStart()
{
    next_status_ = EXPLORING;
}

void ExplorationExecutive::onMovingToStartExit()
{

}

void ExplorationExecutive::onExploringEnter()
{

}

void ExplorationExecutive::onExploring()
{
    if (!goal_pending_) {
        if (goalid_ > 0) { // goalid = 0 -> haven't sent a goal yet
            goal_pub_.publish(goals_);
            exploration_thread_.publish_maps();
        }
        exploration_thread_.compute_goals(std::bind(&ExplorationExecutive::goals_callback, this, std::placeholders::_1));
        goal_pending_ = true;
        ++goalid_;
    }
}

void ExplorationExecutive::onExploringExit()
{

}

} // namespace utacc

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Exploration");
    return utacc::ExplorationExecutive().run();
}

