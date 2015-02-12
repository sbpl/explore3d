//// Jonathan Butzke
///(c) 2014 Ros wrapper header

#ifndef ExplorationThread_h
#define ExplorationThread_h

#include <functional>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include "exploration.hpp"
#include "exploration_structs.hpp"

/// @class Class for asynchronously computing exploration goals for multiple robots
///
/// This class is not thread-safe and assumes that there is one calling thread
/// for this class.
///
/// Only one asynchronous request can be handled at a time.
class ExplorationThread
{
public:

    typedef std::function<void(nav_msgs::Path& goal_poses)> GoalPosesCallback;
    typedef std::function<void(geometry_msgs::PoseStamped& goal_pose)> GoalPoseCallback;

    ExplorationThread();
    ~ExplorationThread();

    bool initialize();
    int run();
    void shutdown();

    const ExpParams_c& params() const { return params_; }

    void update_map(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud);
    void update_poses(const nav_msgs::PathConstPtr& robot_poses);

    bool ready_to_plan() const;
    bool busy() const;

    /// @brief Asynchronously compute a new goal for a particular robot given the current map and robot poses
    /// @param ridx Index of the robot to compute a goal for in [0, this->params().robots.size())
    /// @param callback Callback to be called by the Exploration Thread when the
    ///     goal is finished being computed
    /// @return true if the Exploration Planner is ready to run (as given by
    ///     this->ready_to_plan()) and is not busy computing another goal (as
    ///     given by this->busy()); false otherwise
    bool compute_goal(std::size_t ridx, const GoalPoseCallback& callback);

    /// @brief Asynchronously compute new goals for all robots given the current map and robot poses.
    /// @brief callback Callback to be called by the Exploration Thread when
    ///     the goals are finished being computed.
    /// @return true if the planner is to plan (as given by
    ///     this->ready_to_plan()) and is not busy computing another goal (as
    ///     given by this->busy()); false otherwise
    bool compute_all_goals(const GoalPosesCallback& callback);

    /// @brief Publishes planner maps for debugging; blocking if exploration planner is currently planning.
    void publish_maps();

private:

    struct MapElementCompare
    {
        bool operator()(const MapElement_c& e, const MapElement_c& f) const
        {
            return std::tie(e.x, e.y, e.z) < std::tie(f.x, f.y, f.z);
        }
    };

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    ExplorationPlanner EP_;
    ExpParams_c params_;
    std::vector<Locations_c> curr_locations_;

    std::map<MapElement_c, unsigned char, MapElementCompare> curr_map_points_;

    /// @name Mutex-guarded communication with planning thread
    /// @{
    std::mutex data_mutex_;
    std::vector<Locations_c> plannerthread_curr_locations_;
    std::vector<MapElement_c> plannerthread_map_points_;
    GoalPosesCallback goal_poses_callback_;
    GoalPoseCallback goal_pose_callback_;
    nav_msgs::Path last_goals_;
    bool goals_requested_;
    std::size_t goal_ridx_; // the index of the robot whose goal we are computing, params_.robots.size() => all robots
    /// @}

    double planner_rate_hz_;

    /// @name Map parameters
    /// @{
    std::string frame_id_;
    double resolution_;
    double angle_resolution_;
    double origin_x_;
    double origin_y_;
    double origin_z_;
    int size_x_;
    int size_y_;
    int size_z_;
    /// @}

    bool got_first_map_update_;
    bool got_first_pose_update_;

    std::thread ep_thread_;

    bool initialized_;

    /// @name Debug Visualization Publishers
    /// @{ 
    ros::Publisher goal_point_cloud_pub_;
    ros::Publisher frontier_map_pub_;
    std::vector<ros::Publisher> coverage_map_pub_;
    std::vector<ros::Publisher> dist_transform_pub_;
    std::vector<ros::Publisher> cost_map_pub_;
    std::vector<ros::Publisher> counts_map_pub_;
    std::vector<ros::Publisher> score_map_pub_;
    /// @}

    bool read_map_params();
    bool read_cost_params();
    bool read_robot_params();
    bool construct_robot_from_config(XmlRpc::XmlRpcValue& params, Robot_c& robot);
    void log_robots(const std::vector<Robot_c>& robots);
    bool initialize_debug_publishers();
    bool initialized() const;

    void plannerthread();

    template<typename T> bool bounds_check(const T & point) const;

    void quaternion_to_yaw(const geometry_msgs::Quaternion & quat, double & yaw) const;
    void yaw_to_quaternion(const double & yaw, geometry_msgs::Quaternion & quat) const;
    int continuous_angle_to_discrete(double cont, double res) const;
    int discrete_anngle_to_continuous(int disc, double res) const;
    int continuous_to_discrete(double cont, double res) const;
    double discrete_to_continuous(int disc, double res) const;

    void goal_locations_to_path_msg(const std::vector<Locations_c>& goal_locations, nav_msgs::Path& msg) const;
    void publish_goal_cloud();

    void get_occupancy_grid_from_coverage_map(int ridx, const ros::Time& time, nav_msgs::OccupancyGrid& map) const;

    // Create an occupancy grid from a costmap. Invalid values have the value
    // 0xFF in the occupancy grid, normal values have a value between 0 and 100
    // representing the value between the minimum cost and the maximum cost in
    // the costmap.
    void get_occupancy_grid_from_costmap(
            const ExplorationPlanner::CostMap& costmap,
            const ros::Time& time,
            nav_msgs::OccupancyGrid& map) const;

    void get_occupancy_grid_from_countmap(
            const ExplorationPlanner::CountMap& countmap,
            const ros::Time& time,
            nav_msgs::OccupancyGrid& map) const;

    void get_occupancy_grid_from_distance_transform(
            const CoverageMap_c& coverage_map,
            int ridx,
            const ros::Time& now,
            nav_msgs::OccupancyGrid& map) const;

    template<typename T>
    void get_point_cloud_from_map(const std::vector<T> &map, std::vector<pcl::PointXYZI> & points);

    template<typename T>
    void get_point_cloud_from_inner_dim(
            const std::vector<T> &map,
            std::vector<pcl::PointXYZI> & points,
            std::vector<int>& indicies,
            int depth);

    template<typename T>
    void get_point_cloud_from_inner_dim(
            const T& val,
            std::vector<pcl::PointXYZI>& points,
            std::vector<int>& indicies, int depth);

    template<typename T>
    void get_point_cloud_from_points(const std::vector<T> & point_list, std::vector<pcl::PointXYZI> & points);

    void publish_point_cloud(const std::vector<pcl::PointXYZI> & points, const ros::Publisher & publisher);

    void publish_planner_maps();
};

#endif

