/// Jonathan Butzke
///(c) 2014 Ros wrapper header
#include "exploration.hpp"
#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <nav_msgs/OccupancyGrid.h>

#include <tf/transform_listener.h>

// Assumptions:
//   There is one main thread that is calling functions listed in the public API of this class.
class ExplorationThread
{
public:

    ExplorationThread();
    ~ExplorationThread();

    bool initialize();
    int run();
    void shutdown();

    void update_map(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud);
    void update_poses(const nav_msgs::PathConstPtr& robot_poses);

    /// @brief Asynchronously compute new goals given the current map and robot poses
    /// @return Whether the planner is ready to run (currently, has received a map and poses)
    typedef std::function<void(nav_msgs::Path& goal_poses)> GoalPosesCallback;
    bool compute_goals(const GoalPosesCallback& goals_callback);

    /// @brief Publishes planner maps for debugging; blocking if exploration planner is currently planning.
    void publish_maps();

private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    ExplorationPlanner EP_;
    ExpParams_c params_;
    std::vector<Locations_c> curr_locations_;
    std::vector<MapElement_c> curr_map_points_;

    /// @name Mutex-guarded communication with planning thread
    /// @{
    std::mutex data_mutex_;
    std::vector<Locations_c> plannerthread_curr_locations_;
    std::vector<MapElement_c> plannerthread_map_points_;
    GoalPosesCallback goal_poses_callback_;
    nav_msgs::Path last_goals_;
    bool goals_requested_;
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

    bool ready_to_plan() const;

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
