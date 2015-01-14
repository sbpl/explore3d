///Jonathan Michael Butzke
///(c) 2014
///ROS Wrapper for 3-D exploration module

#include "exploration_main.hpp"

#include <cassert>
#include <chrono>
#include <unordered_map>

#if PCL_MINOR_VERSION == 6 and PCL_MAJOR_VERSION == 1
#define ros_to_pcl_time_now ros::Time::now();
#else
#define ros_to_pcl_time_now ros::Time::now().toNSec();
#endif


void EP_wrapper::plannerthread(void)
{
    ros::Rate looprate(planner_rate);
    std::vector<MapElement_c> pts;
    std::vector<Locations_c> robot_loc;

    while (ros::ok()) {
        looprate.sleep();

        //do not execute planner until first map and pose updates are received
        if (!(got_first_map_update && got_first_pose_update)) {
            ROS_INFO("planner_thread: waiting for first map and pose update");
            continue;
        }

        std::chrono::time_point<std::chrono::high_resolution_clock> start, finish;

        //update map and poses
        ROS_DEBUG("Updating map and poses...");
        start = std::chrono::high_resolution_clock::now();
        data_mutex_.lock();
        pts = MapPts_;
        robot_loc = CurrentLocations_;
        data_mutex_.unlock();
        finish = std::chrono::high_resolution_clock::now();
        ROS_DEBUG("Updating map and poses took %0.3f seconds", std::chrono::duration<double>(finish - start).count());

        EP.PartialUpdateMap(pts);

        //retrieve goals
        ROS_INFO("Computing goals...");
        start = std::chrono::high_resolution_clock::now();
        for (size_t ridx = 0; ridx < robot_loc.size(); ridx++) {
            ROS_INFO("poses r%li: %s", ridx, to_string(robot_loc[ridx]).c_str());
        }
        std::vector<Locations_c> goals = EP.NewGoals(robot_loc);
        for (size_t ridx = 0; ridx < goals.size(); ridx++) {
            ROS_INFO("goals r%li: %s", ridx, to_string(goals[ridx]).c_str());
        }
        finish = std::chrono::high_resolution_clock::now();
        ROS_INFO("Computing goals took %0.3f seconds", std::chrono::duration<double>(finish - start).count());

        publish_goal_list(goals);
        publish_planner_maps();

        //clear the map points
        MapPts_.clear();
    }
}

void EP_wrapper::PoseCallback(const nav_msgs::PathConstPtr& msg)
{
    Locations_c SegLoc, HexaLoc;

    auto poses = msg->poses;

    if (poses.size() < 2) {
        ROS_ERROR("ERROR poses list provided is size %d when it should be size %d", (int)poses.size(), 2);
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

    // bounds check both robot locationss
    if (!bounds_check(SegLoc)) {
        segbot_bound_check = false;
        ROS_ERROR("ERROR: segbot location at discrete location %d %d %d outside of mapbounds %u %u %u", SegLoc.x, SegLoc.y, SegLoc.z, size_x, size_y, size_y);
    }
    if (!bounds_check(HexaLoc)) {
        hexa_bound_check = false;
        ROS_ERROR("ERROR: hexa location at discrete location %d %d %d outside of mapbounds %u %u %u", HexaLoc.x, HexaLoc.y, HexaLoc.z, size_x, size_y, size_y);
    }

    // don't update locations if one is outside map bounds
    if (!(hexa_bound_check && segbot_bound_check)) {
        return;
    }

    {
        std::unique_lock<std::mutex> data_lock(data_mutex_);
        CurrentLocations_.clear();
        CurrentLocations_.push_back(SegLoc);
        CurrentLocations_.push_back(HexaLoc);
    }

    got_first_pose_update = true;
}

void EP_wrapper::MapCallback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& msg)
{
    ROS_DEBUG("map upate callback for seq %d", (int ) msg->header.seq);
    std::unique_lock<std::mutex> data_lock(data_mutex_);
    //TODO: this is inefficient
    MapElement_c pt;
    for (size_t pidx = 0; pidx < msg->points.size(); pidx++) {
        //convert coordinates to discrete in map frame
        pt.x = continuous_to_discrete(msg->points[pidx].x - origin_x, resolution);
        pt.y = continuous_to_discrete(msg->points[pidx].y - origin_y, resolution);
        pt.z = continuous_to_discrete(msg->points[pidx].z - origin_z, resolution);
        pt.data = msg->points[pidx].intensity;

        //dont add points outside of map bounds
        if (!bounds_check(pt)) {
            continue;
        }

        MapPts_.push_back(pt);
    }
    data_lock.unlock();
    got_first_map_update = true;
    ROS_DEBUG("map upate callback finished");
}

bool EP_wrapper::init(void)
{
    ph.param("planner_rate", planner_rate, 0.5);
    ph.param("scale", scale, 20.0);

    got_first_map_update = false;
    got_first_pose_update = false;

    //////////////////////////////////////////////////////////////////////
    // Read in map parameters
    //////////////////////////////////////////////////////////////////////

    ph.param<std::string>("frame_id", frame_id, "/map");
    ph.param<int>("size_x", size_x, 500);
    ph.param<int>("size_y", size_y, 500);
    ph.param<int>("size_z", size_z, 50);
    ph.param<double>("origin_x", origin_x, 0);
    ph.param<double>("origin_y", origin_y, 0);
    ph.param<double>("origin_z", origin_z, 0);
    ph.param<double>("resolution", resolution, 0.5);

    params.size_x = size_x;
    params.size_y = size_y;
    params.size_z = size_z;

    //////////////////////////////////////////////////////////////////////
    // Read in cost parameters
    //////////////////////////////////////////////////////////////////////

    int objectmaxelev, obs, freespace, unk, numangles, mindist;
    double backwards_penalty;
    ph.param<int>("objectmaxelev", objectmaxelev, (uint) 1.5 * scale); // max height to consider for the OOI (cells)
    ph.param<int>("obsvalue", obs, 100); // values for obstacles, freespace, unknown
    ph.param<int>("freevalue", freespace, 50);
    ph.param<int>("unkvalue", unk, 0);
    ph.param<int>("numangles", numangles, 16); // number of thetas
    ph.param<int>("mindist", mindist, (uint) 1.2 * scale); // closest robots should operate without penalty (cells)
    ph.param<double>("backwards_penalty", backwards_penalty, 1.0);

    params.ObjectMaxElev = objectmaxelev;
    params.obs = obs;
    params.freespace = freespace;
    params.unk = unk;
    params.NumAngles = numangles;
    params.MinDist = mindist;
    params.backwards_penalty = backwards_penalty;

    angle_resolution = (2 * M_PI) / numangles;

    //////////////////////////////////////////////////////////////////////
    // Read in topic name parameters
    //////////////////////////////////////////////////////////////////////

    ph.param<std::string>("goal_topic", goal_topic_, "goal_list");
    ph.param<std::string>("map_topic", map_topic_, "combined_map");
    ph.param<std::string>("pose_topic", pose_topic_, "combined_pose");
    ph.param<std::string>("goal_point_cloud_topic", goal_point_cloud_topic, "goal_point_cloud");

    //////////////////////////////////////////////////////////////////////
    // Read in robot parameters.
    //////////////////////////////////////////////////////////////////////

    // NOTE: robot parameters must be read after the map parameters

    XmlRpc::XmlRpcValue robot_params;
    ph.getParam("robot_params", robot_params);
    if (robot_params.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        return false;
    }

    ROS_INFO("Reading in %d robots from config", robot_params.size());

    std::vector<Robot_c> robots;
    for (int i = 0; i < robot_params.size(); ++i) {
        XmlRpc::XmlRpcValue& params = robot_params[i];
        if (params.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            return false;
        }
        Robot_c robot;
        if (!this->create_robot_from_config(params, scale, robot)) {
            return false;
        }
        robots.push_back(robot);
    }
    params.robots = robots;

    this->log_robots(params.robots);

    EP.Init(params);
    EP_thread_ = new std::thread(&EP_wrapper::plannerthread, this);

    ROS_ERROR("subscribed to %s and %s", pose_topic_.c_str(), map_topic_.c_str());
    ROS_ERROR("dims is %d %d %d", size_x, size_y, size_z);
    ROS_ERROR("resolution is %f", resolution);

    Pose_sub_ = nh.subscribe(pose_topic_, 1, &EP_wrapper::PoseCallback, this);
    Map_sub_ = nh.subscribe(map_topic_, 20, &EP_wrapper::MapCallback, this);
    Goal_pub_ = nh.advertise<nav_msgs::Path>(goal_topic_, 1);
    Goal_point_cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(goal_point_cloud_topic, 1);

//    coverage_map_pub = ph.advertise<pcl::PointCloud<pcl::PointXYZI> >("coverage_map", 1);
    frontier_map_pub = ph.advertise<pcl::PointCloud<pcl::PointXYZI> >("frontier_map", 1);

    cost_map_pub.reserve(params.robots.size());
    counts_map_pub.reserve(params.robots.size());
    score_map_pub.reserve(params.robots.size());
    for (const Robot_c& robot : params.robots) {
        ros::Publisher robot_costmap_pub = ph.advertise<nav_msgs::OccupancyGrid>(robot.name + "_cost_map", 1);
        ros::Publisher dist_transform_pub = ph.advertise<nav_msgs::OccupancyGrid>(robot.name + "_dist_transform", 1);
        ros::Publisher robot_counts_map_pub = ph.advertise<nav_msgs::OccupancyGrid>(robot.name + "_counts_map", 1);
        ros::Publisher robot_coverage_map_pub = ph.advertise<nav_msgs::OccupancyGrid>(robot.name + "_coverage_map", 1);
        ros::Publisher robot_score_map_pub = ph.advertise<nav_msgs::OccupancyGrid>(robot.name + "_score_map", 1);
        coverage_map_pub_.push_back(robot_coverage_map_pub);
        dist_transform_pub_.push_back(dist_transform_pub);
        cost_map_pub.push_back(robot_costmap_pub);
        counts_map_pub.push_back(robot_counts_map_pub);
        score_map_pub.push_back(robot_score_map_pub);
    }

    return true;
}

EP_wrapper::EP_wrapper() :
    nh(),
    ph("~")
{
    if (!init()) {
        ROS_ERROR("Failed to initialize EP wrapper");
    }
    else {
        ROS_INFO("Successfully initialized EP wrapper");
    }
}

EP_wrapper::~EP_wrapper()
{
    if (EP_thread_ && EP_thread_->joinable()) {
        EP_thread_->join();
        delete EP_thread_;
    }
}

template<typename T>
bool EP_wrapper::bounds_check(const T& point)
{
    if ((int) point.x < 0 || (int) point.x >= size_x ||
        (int) point.y < 0 || (int) point.y >= size_y ||
        (int) point.z < 0 || (int) point.z >= size_z)
    {
        return false;
    }
    return true;
}

int main(int argc, char** argv)
{
    ROS_ERROR("start");
    ros::init(argc, argv, "Exploration");
    ROS_ERROR("ros init done");
    EP_wrapper EPW;
    ROS_ERROR("made EP wrapper");

    while (ros::ok()) {
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
    for (auto & pose : goal_list.poses) {
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
    if (cont < 0) {
        cont = 2 * pi + cont;
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
    cloud.header.stamp = ros_to_pcl_time_now;
    cloud.header.frame_id = frame_id;
    cloud.height = 1;

    cloud.points.reserve(points.size());
    for (auto & p : points) {
        cloud.points.push_back(p);
    }
    cloud.width = cloud.points.size();
    publisher.publish(cloud);
}

void EP_wrapper::publish_planner_maps()
{
    ROS_DEBUG("publishing maps....");

    ros::Time now = ros::Time::now();

    std::vector<pcl::PointXYZI> frontier_points;
    get_point_cloud_from_points(EP.Frontier3d_, frontier_points);
    publish_point_cloud(frontier_points, frontier_map_pub);

    const auto& grid = EP.coverage_.map_;
    for (std::size_t i = 0; i < params.robots.size(); ++i) {
        ROS_DEBUG("Publishing maps for robot %zd", i);
        nav_msgs::OccupancyGrid motion_height_grid;
        this->get_occupancy_grid_from_map_at_height(grid, now, motion_height_grid, params.robots[i].MotionHeight_);
        coverage_map_pub_.at(i).publish(motion_height_grid);

        nav_msgs::OccupancyGrid distance_transform_grid;
        const auto& coverage_map = EP.coverage_;
        this->get_occupancy_grid_from_distance_transform(coverage_map, i, now, distance_transform_grid);
        dist_transform_pub_.at(i).publish(distance_transform_grid);

        ROS_DEBUG("  Publishing costmap...");
        nav_msgs::OccupancyGrid costmap_grid;
        this->get_occupancy_grid_from_costmap(EP.CostToPts_.at(i), now, costmap_grid);
        cost_map_pub.at(i).publish(costmap_grid);

        ROS_DEBUG("  Publishing countmap...");
        nav_msgs::OccupancyGrid count_grid;
        get_occupancy_grid_from_countmap(EP.counts_.at(i), now, count_grid);
        counts_map_pub.at(i).publish(count_grid);

        ROS_DEBUG("  Publishing scoremap");
        nav_msgs::OccupancyGrid score_grid;
        get_occupancy_grid_from_countmap(EP.scores_.at(i), now, score_grid);
        score_map_pub.at(i).publish(score_grid);
    }

    ROS_DEBUG("done");
}

void EP_wrapper::get_occupancy_grid_from_map_at_height(
    const ExplorationPlanner::Map& epmap,
    const ros::Time& time,
    nav_msgs::OccupancyGrid& map,
    uint height)
{
    assert(epmap.size(0) == size_x && epmap.size(1) == size_y);

    map.header.frame_id = frame_id;
    map.header.stamp = time;

    map.info.width = size_x;
    map.info.height = size_y;
    map.info.origin.position.x = origin_x;
    map.info.origin.position.y = origin_y;
    map.info.origin.position.z = origin_z;
    map.info.resolution = resolution;
    map.data.resize(size_x * size_y);

    for (std::size_t x = 0; x < epmap.size(0); ++x) {
        for (std::size_t y = 0; y < epmap.size(1); ++y) {
            char val = epmap(x, y, height);
            if ((unsigned char)val == params.unk) {
                map.data[y * size_x + x] = -1;
            }
            else if ((unsigned char)val == params.freespace) {
                map.data[y * size_x + x] = 0;
            }
            else if ((unsigned char)val == params.obs) {
                map.data[y * size_x + x] = 100;
            }
            else {
                ROS_ERROR("Unexpected 3-D map value");
            }
        }
    }
}

void EP_wrapper::get_occupancy_grid_from_costmap(
    const ExplorationPlanner::CostMap& costmap,
    const ros::Time& time,
    nav_msgs::OccupancyGrid& map) const
{
    map.header.frame_id = frame_id;
    map.header.stamp = time;

    map.info.width = size_x;
    map.info.height = size_y;
    map.info.origin.position.x = origin_x;
    map.info.origin.position.y = origin_y;
    map.info.origin.position.z = origin_z;
    map.info.resolution = resolution;

    map.data.resize(size_x * size_y);

    // find the minimum and maximum non-infinite costs in the costmap
    CostType min_cost = -1.0, max_cost = -1.0;
    for (std::size_t x = 0; x < costmap.size(0); ++x) {
        for (std::size_t y = 0; y < costmap.size(1); ++y) {
            CostType cost = costmap(x, y);
            if (cost == MaxCost) {
                continue;
            }

            if (min_cost == -1.0 || cost < min_cost) {
                min_cost = cost;
            }
            if (max_cost == -1.0 || cost > max_cost) {
                max_cost = cost;
            }
        }
    }

    // log a histogram of the costs in the map
    std::unordered_map<CostType, int> cost_histogram;
    for (std::size_t x = 0; x < costmap.size(0); ++x) {
        for (std::size_t y = 0; y < costmap.size(1); ++y) {
            CostType cost = costmap(x, y);
            auto cit = cost_histogram.find(cost);
            if (cit == cost_histogram.end()) {
                cost_histogram[cost] = 1;
            }
            else {
                ++cost_histogram[cost];
            }
        }
    }

    double span = max_cost - min_cost;
    ROS_DEBUG("    Costs span %0.3f (min: %0.3f, max: %0.3f)", span, min_cost, max_cost);

    for (std::size_t x = 0; x < costmap.size(0); ++x) {
        for (std::size_t y = 0; y < costmap.size(1); ++y) {
            CostType cost = costmap(x, y);
            if (cost == MaxCost) {
                map.data[y * size_x + x] = 0xFF;
            }
            else {
                if (span == 0.0) {
                    map.data[y * size_x + x] = -1;
//                    map.data[y * size_x + x] = 100;
                }
                else {
                    map.data[y * size_x + x] = (100 * ((cost - min_cost) / span));
                }
            }
        }
    }
}

void EP_wrapper::get_occupancy_grid_from_countmap(
    const ExplorationPlanner::CountMap& countmap,
    const ros::Time& time,
    nav_msgs::OccupancyGrid& map) const
{
    // downproject countmap to costmap

    ExplorationPlanner::CostMap costmap;

    costmap.resize(countmap.size(0), countmap.size(1));
    costmap.assign(0);

    for (std::size_t x = 0; x < countmap.size(0); ++x) {
        for (std::size_t y = 0; y < countmap.size(1); ++y) {
            for (std::size_t a = 0; a < countmap.size(2); ++a) {
                costmap(x, y) += std::max(costmap(x, y), countmap(x, y, a)); //countmap(x, y, a);
            }
        }
    }

    return this->get_occupancy_grid_from_costmap(costmap, time, map);
}

void EP_wrapper::get_occupancy_grid_from_distance_transform(
    const CoverageMap_c& coverage_map,
    int ridx,
    const ros::Time& now,
    nav_msgs::OccupancyGrid& map) const
{
    ExplorationPlanner::CostMap dist_transform;
    dist_transform.resize(coverage_map.x_size_, coverage_map.y_size_);
    for (std::size_t x = 0; x < coverage_map.x_size_; ++x) {
        for (std::size_t y = 0; y < coverage_map.y_size_; ++y) {
            double dval = coverage_map.ReturnDistToObs(ridx, x, y);
            dist_transform(x, y) = dval;
        }
    }

    this->get_occupancy_grid_from_costmap(dist_transform, now, map);

    // invert map values
    for (auto& datum : map.data) {
        datum = 100 - datum;
    }
}

template<typename T>
void EP_wrapper::get_point_cloud_from_map(const std::vector<T> &map, std::vector<pcl::PointXYZI> & points)
{
    std::vector<int> indicies;
    get_point_cloud_from_inner_dim(map, points, indicies, 0);
}

template<typename T>
void EP_wrapper::get_point_cloud_from_inner_dim(
    const std::vector<T>& map,
    std::vector<pcl::PointXYZI>& points, std::vector<int> & indicies,
    int depth)
{
    size_t dim_size = map.size();
    for (size_t s = 0; s < dim_size; s++) {
        if (indicies.size() < (size_t) depth + 1) {
            indicies.push_back(0);
        }
        indicies[depth] = s;
        get_point_cloud_from_inner_dim(map.at(s), points, indicies, depth + 1);
    }
}

template<typename T>
void EP_wrapper::get_point_cloud_from_inner_dim(
    const T& val,
    std::vector<pcl::PointXYZI>& points,
    std::vector<int> & indicies,
    int depth)
{
    static int count = 0;
    count++;
    pcl::PointXYZI point;
    if (depth > 0) {
        point.x = discrete_to_continuous(indicies[0], resolution) + origin_x;
    }

    if (depth > 1) {
        point.y = discrete_to_continuous(indicies[1], resolution) + origin_y;
    }

    if (depth > 2) {
        point.z = discrete_to_continuous(indicies[2], resolution) + origin_z;
    }
    point.intensity = static_cast<double>(val);
    if (point.intensity > 0) {
        points.push_back(point);
    }
}

template<typename T>
inline void EP_wrapper::get_point_cloud_from_points(const std::vector<T>& point_list, std::vector<pcl::PointXYZI>& points)
{
    //convert to 3d points
    for (auto & lp : point_list) {
        pcl::PointXYZI p;
        p.x = discrete_to_continuous(lp.x, resolution) + origin_x;
        p.y = discrete_to_continuous(lp.y, resolution) + origin_y;
        p.z = discrete_to_continuous(lp.z, resolution) + origin_z;
        p.intensity = lp.cost;
        points.push_back(p);
    }
}

bool EP_wrapper::create_robot_from_config(XmlRpc::XmlRpcValue& params, double scale, Robot_c& robot)
{
    if (!params.hasMember("name")               || params["name"].getType() != XmlRpc::XmlRpcValue::TypeString ||
        !params.hasMember("motionheight")       || params["motionheight"].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
        !params.hasMember("sensorheight")       || params["sensorheight"].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
        !params.hasMember("horizontalfov_degs") || params["horizontalfov_degs"].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
        !params.hasMember("verticalfov_degs")   || params["verticalfov_degs"].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
        !params.hasMember("detectionrange")     || params["detectionrange"].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
        !params.hasMember("circularsize")       || params["circularsize"].getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
        ROS_ERROR("Robot config does not support minimum requirements");
        return false;
    }

    double robot_motionheight;
    double robot_sensorheight;
    double robot_detectionrange;
    double robot_circularsize;

    robot_motionheight = double(params["motionheight"]);
    robot_sensorheight = double(params["sensorheight"]);
    robot_detectionrange = double(params["detectionrange"]);
    robot_circularsize = double(params["circularsize"]);

    robot.HorizontalFOV_ = double(params["horizontalfov_degs"]);
    robot.VerticalFOV_ = double(params["verticalfov_degs"]);
    robot.HorizontalFOV_ *= M_PI / 180.0;
    robot.VerticalFOV_ *= M_PI / 180.0;

    robot.name = std::string(params["name"]);
    robot.MotionHeight_     = (uint)(robot_motionheight / resolution);
    robot.SensorHeight_     = (uint)(robot_sensorheight / resolution);
    robot.DetectionRange_   = (uint)(robot_detectionrange);
    robot.CircularSize_     = (uint)(robot_circularsize / resolution);

    return true;
}

void EP_wrapper::log_robots(const std::vector<Robot_c>& robots)
{
    ROS_INFO("Robots:");
    for (const Robot_c& robot : robots) {
        ROS_INFO("  %s", robot.name.c_str());
        ROS_INFO("    Vertical FOV: %0.3f", robot.VerticalFOV_);
        ROS_INFO("    Horizontal FOV: %0.3f", robot.HorizontalFOV_);
        ROS_INFO("    Circular Size: %u", robot.CircularSize_);
        ROS_INFO("    Detection Range: %u", robot.DetectionRange_);
        ROS_INFO("    Motion Height: %u", robot.MotionHeight_);
        ROS_INFO("    Sensor Height: %u", robot.SensorHeight_);
    }
}
