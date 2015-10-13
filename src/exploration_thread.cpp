///Jonathan Michael Butzke
///(c) 2014
///ROS Wrapper for 3-D exploration module

#include <sys/time.h>
#include <explore3d/exploration_thread.hpp>

#include <cassert>
#include <chrono>
#include <unordered_map>

#if PCL_MINOR_VERSION == 6 and PCL_MAJOR_VERSION == 1
#define ros_to_pcl_time_now ros::Time::now();
#else
#define ros_to_pcl_time_now ros::Time::now().toNSec();
#endif

ExplorationThread::ExplorationThread() :
    nh_(),
    ph_("~"),
    EP_(),
    params_(),
    curr_locations_(),
    curr_map_points_(),
    data_mutex_(),
    plannerthread_curr_locations_(),
    plannerthread_map_points_(),
    result_callback_(),
    last_goals_(),
    goals_requested_(false),
    planner_rate_hz_(0.0),
    frame_id_(),
    resolution_(0.0),
    angle_resolution_(0.0),
    origin_x_(0.0),
    origin_y_(0.0),
    origin_z_(0.0),
    size_x_(0),
    size_y_(0),
    size_z_(0),
    got_first_map_update_(false),
    got_first_pose_update_(false),
    ep_thread_(),
    initialized_(false),
    goal_point_cloud_pub_(),
    frontier_map_pub_(),
    coverage_map_pub_(),
    dist_transform_pub_(),
    cost_map_pub_(),
    counts_map_pub_(),
    score_map_pub_(),
    m_map_stats_log(nullptr),
    m_received_any_request(false)
{
}

ExplorationThread::~ExplorationThread()
{
    this->shutdown();
}

bool ExplorationThread::initialize()
{
    ph_.param("planner_rate_hz", planner_rate_hz_, 0.5);

    if (!this->read_map_params()) {
        return false;
    }
    ROS_INFO("dims are %d %d %d", size_x_, size_y_, size_z_);
    ROS_INFO("resolution_ is %f", resolution_);

    if (!this->read_cost_params()) {
        return false;
    }

    // NOTE: robot parameters must be read after the map parameters
    if (!this->read_robot_params()) {
        return false;
    }

    // Initialize Exploration Planner after all the above parameters have been read in.
    this->log_robots(params_.robots);
    EP_.Init(params_);

    if (!this->initialize_debug_publishers()) {
        return false;
    }

    timeval t;
    if (gettimeofday(&t, NULL) != 0) {
        ROS_ERROR("Failed to get time of day. WTF");
        return false;
    }

    std::stringstream stats_fname_stream;
    std::string stats_dir = "/home/utacc/utacc_stats/";
    stats_fname_stream << stats_dir << "map_stats_" << t.tv_sec; // TODO: parameterize this statistics gathering and directory
    m_map_stats_log = fopen(stats_fname_stream.str().c_str(), "w");
    if (!m_map_stats_log) {
        ROS_ERROR("Failed to open file %s for writing", stats_fname_stream.str().c_str());
        return false;
    }

    initialized_ = true;
    return initialized_;
}

int ExplorationThread::run()
{
    if (!this->initialized()) {
        return 1;
    }

    ep_thread_ = std::thread(&ExplorationThread::plannerthread, this);
    return 0;
}

void ExplorationThread::shutdown()
{
    if (ep_thread_.joinable()) {
        ep_thread_.join();
    }
}

void ExplorationThread::update_map(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud)
{
    int num_cells_free = 0, num_cells_obstacle = 0, num_cells_unknown = 0;
    for (size_t pidx = 0; pidx < cloud->points.size(); pidx++) {
        // convert coordinates to discrete in map frame
        MapElement_c pt;
        pt.x = continuous_to_discrete(cloud->points[pidx].x - origin_x_, resolution_);
        pt.y = continuous_to_discrete(cloud->points[pidx].y - origin_y_, resolution_);
        pt.z = continuous_to_discrete(cloud->points[pidx].z - origin_z_, resolution_);
        // pt.data = cloud->points[pidx].intensity;

        // dont add points outside of map bounds
        if (!bounds_check(pt)) {
            continue;
        }

        // only keep around unique coordinates with the most recent data field
        auto res = curr_map_points_.insert(std::make_pair(pt, 0x00));
        res.first->second = cloud->points[pidx].intensity;

        if (cloud->points[pidx].intensity == params_.obs) {
            ++num_cells_obstacle;
        }
        else if (cloud->points[pidx].intensity == params_.freespace) {
            ++num_cells_free;
        }
        else if (cloud->points[pidx].intensity == params_.unk) {
            ++num_cells_unknown;
        }
    }

    if (m_received_any_request) {
        // log map statics to file
        timeval t;
        if (gettimeofday(&t, NULL) != 0) {
            ROS_ERROR("Faild to get time of day");
        }

        // log map statistics
        if (t.tv_sec != 0) {
            if (fprintf(m_map_stats_log, "time: %ld, freespace: %d, obstacle: %d, unknown: %d\n", t.tv_sec, num_cells_free, num_cells_obstacle, num_cells_unknown) < 0) {
                ROS_ERROR("Error writing to map stats log file");
            }
            else {
                fflush(m_map_stats_log);
            }
        }
    }

    got_first_map_update_ = true;
    // ROS_DEBUG("map upate callback finished");
}

void ExplorationThread::update_poses(const nav_msgs::PathConstPtr& robot_poses)
{
    Locations_c SegLoc, HexaLoc;

    auto poses = robot_poses->poses;

    if (poses.size() < 2) {
        ROS_ERROR("ERROR poses list provided is size %d when it should be size %d", (int)poses.size(), 2);
    }

    //Convert poses from world continuous to map discrete
    double yaw;
    SegLoc.x = continuous_to_discrete(poses[0].pose.position.x - origin_x_, resolution_);
    SegLoc.y = continuous_to_discrete(poses[0].pose.position.y - origin_y_, resolution_);
    SegLoc.z = continuous_to_discrete(poses[0].pose.position.z - origin_z_, resolution_);
    quaternion_to_yaw(poses[0].pose.orientation, yaw);
    SegLoc.theta = continuous_angle_to_discrete(yaw, angle_resolution_);

    HexaLoc.x = continuous_to_discrete(poses[1].pose.position.x - origin_x_, resolution_);
    HexaLoc.y = continuous_to_discrete(poses[1].pose.position.y - origin_y_, resolution_);
    HexaLoc.z = continuous_to_discrete(poses[1].pose.position.z - origin_z_, resolution_);
    quaternion_to_yaw(poses[1].pose.orientation, yaw);
    HexaLoc.theta = continuous_angle_to_discrete(yaw, angle_resolution_);

    bool hexa_bound_check = true;
    bool segbot_bound_check = true;

    // bounds check both robot locationss
    if (!bounds_check(SegLoc)) {
        segbot_bound_check = false;
        ROS_ERROR("ERROR: segbot location at discrete location %d %d %d outside of mapbounds %u %u %u", SegLoc.x, SegLoc.y, SegLoc.z, size_x_, size_y_, size_z_);
    }
    if (!bounds_check(HexaLoc)) {
        hexa_bound_check = false;
        ROS_ERROR("ERROR: hexa location at discrete location %d %d %d outside of mapbounds %u %u %u", HexaLoc.x, HexaLoc.y, HexaLoc.z, size_x_, size_y_, size_z_);
    }

    // don't update locations if one is outside map bounds
    if (!(hexa_bound_check && segbot_bound_check)) {
        return;
    }

    curr_locations_.clear();
    curr_locations_.push_back(SegLoc);
    curr_locations_.push_back(HexaLoc);
    got_first_pose_update_ = true;
}

bool ExplorationThread::compute_goal(std::size_t ridx, const ResultCallback& callback)
{
    std::unique_lock<std::mutex> lock(data_mutex_);

    if (ridx >= params_.robots.size()) {
        ROS_ERROR("Invalid robot index given to ExplorationThread::compute_goal");
        return false;
    }

    if (!this->ready_to_plan()) {
        ROS_INFO("Exploration Thread is not ready to plan");
        return false;
    }

    if (this->busy()) {
        ROS_INFO("Exploration Thread is busy working on another goal");
        return false;
    }

    m_received_any_request = true;

    // under above lock, save state of robot poses and map
    ROS_INFO("Request to compute goals received");
    plannerthread_curr_locations_ = curr_locations_;
    plannerthread_map_points_.clear();
    plannerthread_map_points_.reserve(curr_map_points_.size());
    for (const auto& entry : curr_map_points_) {
        MapElement_c e = entry.first;
        e.data = entry.second;
        plannerthread_map_points_.push_back(e);
    }

    goal_ridx_ = ridx;

    // marks thread as busy
    goals_requested_ = true;

    // register goal callback for when goal computing is done
    result_callback_ = callback;
    return true;
}

bool ExplorationThread::compute_all_goals(const ResultCallback& callback)
{
    if (!this->ready_to_plan()) {
        ROS_INFO("Exploration Thread is not ready to plan");
        return false;
    }

    if (this->busy()) {
        ROS_INFO("Exploration Thread is busy working on another goal");
        return false;
    }

    m_received_any_request = true;

    std::unique_lock<std::mutex> lock(data_mutex_);

    // under above lock, save state of robot poses and map
    ROS_INFO("Request to compute goals received");
    plannerthread_curr_locations_ = curr_locations_;
    plannerthread_map_points_.clear();
    plannerthread_map_points_.reserve(curr_map_points_.size());
    for (const auto& entry : curr_map_points_) {
        MapElement_c e = entry.first;
        e.data = entry.second;
        plannerthread_map_points_.push_back(e);
    }

    goal_ridx_ = params_.robots.size(); // signals to compute goals for all robots

    // marks thread as busy
    goals_requested_ = true;

    // register goal callback for when goal computing is done
    result_callback_ = callback;
    return true;
}

void ExplorationThread::publish_maps()
{
    std::unique_lock<std::mutex> lock(data_mutex_);
    this->publish_goal_cloud();
    this->publish_planner_maps();
}

void ExplorationThread::quick_log(const std::string& message)
{
    if (!this->initialized()) {
        ROS_ERROR("Failed to quick log on uninitialized exploration thread");
        return;
    }

    timeval t;
    if (gettimeofday(&t, NULL) != 0) {
        ROS_ERROR("Failed to get time of day. WTF");
        return;
    }

    if (fprintf(m_map_stats_log, "[QUICKLOG] %ld: %s\n", t.tv_sec, message.c_str()) < 0) {
        ROS_ERROR("Failed to write '%s' to file", message.c_str());
        return;
    }
}

bool ExplorationThread::construct_robot_from_config(XmlRpc::XmlRpcValue& params, Robot_c& robot)
{
    if (!params.hasMember("name")               || params["name"].getType() != XmlRpc::XmlRpcValue::TypeString ||
        !params.hasMember("motionheight")       || params["motionheight"].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
        !params.hasMember("motionlevelbottom")  || params["motionlevelbottom"].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
        !params.hasMember("motionleveltop")     || params["motionleveltop"].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
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
    double robot_motionlevelbottom;
    double robot_motionleveltop;
    double robot_sensorheight;
    double robot_detectionrange;
    double robot_circularsize;

    robot_motionheight = double(params["motionheight"]);
    robot_motionlevelbottom = double(params["motionlevelbottom"]);
    robot_motionleveltop = double(params["motionleveltop"]);
    robot_sensorheight = double(params["sensorheight"]);
    robot_detectionrange = double(params["detectionrange"]);
    robot_circularsize = double(params["circularsize"]);

    robot.HorizontalFOV_ = double(params["horizontalfov_degs"]);
    robot.VerticalFOV_ = double(params["verticalfov_degs"]);
    robot.HorizontalFOV_ *= M_PI / 180.0;
    robot.VerticalFOV_ *= M_PI / 180.0;

    robot.name = std::string(params["name"]);
    robot.MotionHeight_         = (uint)(robot_motionheight / resolution_);
    robot.MotionLevelBottom_    = (uint)(robot_motionlevelbottom / resolution_);
    robot.MotionLevelTop_       = (uint)(robot_motionleveltop / resolution_);
    robot.SensorHeight_         = (uint)(robot_sensorheight / resolution_);
    robot.DetectionRange_       = (uint)(robot_detectionrange / resolution_);
    robot.CircularSize_         = (uint)(robot_circularsize / resolution_);

    return true;
}

void ExplorationThread::log_robots(const std::vector<Robot_c>& robots)
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

bool ExplorationThread::initialized() const
{
    return initialized_;
}

bool ExplorationThread::read_map_params()
{
    int numangles;
    ph_.param<std::string>("frame_id", frame_id_, "/map");
    ph_.param<int>("map_size_x", size_x_, 500);
    ph_.param<int>("map_size_y", size_y_, 500);
    ph_.param<int>("map_size_z", size_z_, 50);
    ph_.param<double>("map_origin_x", origin_x_, 0);
    ph_.param<double>("map_origin_y", origin_y_, 0);
    ph_.param<double>("map_origin_z", origin_z_, 0);
    ph_.param<double>("map_resolution", resolution_, 0.5);
    ph_.param<int>("numangles", numangles, 16); // number of thetas

    params_.size_x = size_x_;
    params_.size_y = size_y_;
    params_.size_z = size_z_;
    params_.NumAngles = numangles;
    angle_resolution_ = (2 * M_PI) / numangles;
    return true;
}

bool ExplorationThread::read_robot_params()
{
    XmlRpc::XmlRpcValue robot_params;
    ph_.getParam("robot_params", robot_params);
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
        if (!this->construct_robot_from_config(params, robot)) {
            return false;
        }
        robots.push_back(robot);
    }
    params_.robots = robots;
    return true;
}

bool ExplorationThread::read_cost_params()
{
    // assumes read_map_params should have been read by now

    const double scale = 20.0;
    int objectmaxelev, obs, freespace, unk, mindist;
    double backwards_penalty;
    ph_.param<int>("objectmaxelev", objectmaxelev, (uint) 1.5 * scale); // max height to consider for the OOI (cells)
    ph_.param<int>("obsvalue", obs, 100); // values for obstacles, freespace, unknown
    ph_.param<int>("freevalue", freespace, 50);
    ph_.param<int>("unkvalue", unk, 0);
    ph_.param<int>("mindist", mindist, (uint) 1.2 * scale); // closest robots should operate without penalty (cells)
    ph_.param<double>("backwards_penalty", backwards_penalty, 1.0);

    // by default the starting line is outside the map
    double room_min_x;
    double room_min_y;
    double room_max_x;
    double room_max_y;
    ph_.param<double>("room_min_x", room_min_x, origin_x_ - 2 * resolution_);
    ph_.param<double>("room_min_y", room_min_y, origin_y_ - 2 * resolution_);
    ph_.param<double>("room_max_x", room_max_x, origin_x_ - 2 * resolution_);
    ph_.param<double>("room_max_y", room_max_y, origin_y_ - 2 * resolution_);

    params_.ObjectMaxElev = objectmaxelev;
    params_.obs = obs;
    params_.freespace = freespace;
    params_.unk = unk;
    params_.MinDist = mindist;
    params_.backwards_penalty = backwards_penalty;
    params_.room_min_x = continuous_to_discrete(room_min_x - origin_x_, resolution_);
    params_.room_min_y = continuous_to_discrete(room_min_y - origin_y_, resolution_);
    params_.room_max_x = continuous_to_discrete(room_max_x - origin_x_, resolution_);
    params_.room_max_y = continuous_to_discrete(room_max_y - origin_y_, resolution_);

    ROS_INFO("Room: [%d, %d] x [%d,  %d]",
            params_.room_min_x, params_.room_min_y, params_.room_max_x, params_.room_max_y);

    return true;
}

bool ExplorationThread::initialize_debug_publishers()
{
    goal_point_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("goal_point_cloud", 1);
    frontier_map_pub_ = ph_.advertise<pcl::PointCloud<pcl::PointXYZI> >("frontier_map", 1);
    cost_map_pub_.reserve(params_.robots.size());
    counts_map_pub_.reserve(params_.robots.size());
    score_map_pub_.reserve(params_.robots.size());
    for (const Robot_c& robot : params_.robots) {
        ros::Publisher robot_costmap_pub = ph_.advertise<nav_msgs::OccupancyGrid>(robot.name + "_cost_map", 1);
        ros::Publisher dist_transform_pub = ph_.advertise<nav_msgs::OccupancyGrid>(robot.name + "_dist_transform", 1);
        ros::Publisher robot_counts_map_pub = ph_.advertise<nav_msgs::OccupancyGrid>(robot.name + "_counts_map", 1);
        ros::Publisher robot_coverage_map_pub = ph_.advertise<nav_msgs::OccupancyGrid>(robot.name + "_coverage_map", 1);
        ros::Publisher robot_score_map_pub = ph_.advertise<nav_msgs::OccupancyGrid>(robot.name + "_score_map", 1);
        coverage_map_pub_.push_back(robot_coverage_map_pub);
        dist_transform_pub_.push_back(dist_transform_pub);
        cost_map_pub_.push_back(robot_costmap_pub);
        counts_map_pub_.push_back(robot_counts_map_pub);
        score_map_pub_.push_back(robot_score_map_pub);
    }
    return true;
}

void ExplorationThread::plannerthread()
{
    ros::Rate looprate(planner_rate_hz_);
    while (ros::ok()) {
        data_mutex_.lock();
        if (goals_requested_) {
            assert(this->ready_to_plan() && !this->busy());

            ROS_INFO("Processing goals request");

            // update exploration planner state
            ROS_DEBUG("Updating exploration planner map...");
            std::chrono::time_point<std::chrono::high_resolution_clock> start, finish;
            start = std::chrono::high_resolution_clock::now();
            EP_.PartialUpdateMap(plannerthread_map_points_);
            finish = std::chrono::high_resolution_clock::now();
            ROS_DEBUG("Updating exploration planner map took %0.3f seconds", std::chrono::duration<double>(finish - start).count());

            // log start poses
            for (size_t ridx = 0; ridx < plannerthread_curr_locations_.size(); ridx++) {
                ROS_DEBUG("poses r%li: %s", ridx, to_string(plannerthread_curr_locations_[ridx]).c_str());
            }

            // compute goals
            // TODO: implement ExplorationPlanner::NewGoal to compute goal only for the requested robot
            ROS_INFO("Computing goals...");
            start = std::chrono::high_resolution_clock::now();
            std::vector<Locations_c> goals = EP_.NewGoals(plannerthread_curr_locations_);
            finish = std::chrono::high_resolution_clock::now();
            ROS_INFO("Computing goals took %0.3f seconds", std::chrono::duration<double>(finish - start).count());

            if (goals.empty()) {
                ROS_ERROR("Failed to compute new exploration goals");
            }

            // log goal poses
            for (size_t ridx = 0; ridx < goals.size(); ridx++) {
                ROS_DEBUG("goals r%li: %s", ridx, to_string(goals[ridx]).c_str());
            }

            // convert goals to nav_msgs::Path
            this->goal_locations_to_path_msg(goals, last_goals_);

            // prepare the result message
            Result result;
            if (goals.empty()) {
                ROS_ERROR("Failed to compute new exploration goals");
                result.success = false;
            }
            else {
                result.success = true;
                result.goals = last_goals_;
                if (goal_ridx_ < params_.robots.size()) { 
                    // truncate goals result to the requested goal
                    if (goal_ridx_ != 0) {
                        std::swap(result.goals.poses[0], result.goals.poses[goal_ridx_]);
                    }
                    result.goals.poses.resize(1); // only return goal for
                }
            }

            result_callback_(result);

            goals_requested_ = false; // signals no longer handling request
        }
        data_mutex_.unlock();

        looprate.sleep();
    }
}

bool ExplorationThread::ready_to_plan() const
{
    return got_first_map_update_ && got_first_pose_update_;
}

bool ExplorationThread::busy() const
{
    return goals_requested_;
}

template<typename T>
bool ExplorationThread::bounds_check(const T& point) const
{
    if ((int) point.x < 0 || (int) point.x >= size_x_ ||
        (int) point.y < 0 || (int) point.y >= size_y_ ||
        (int) point.z < 0 || (int) point.z >= size_z_)
    {
        return false;
    }
    return true;
}

int ExplorationThread::continuous_to_discrete(double cont, double res) const
{
    double v = cont / res;
    v >= 0 ? v = floor(v) : v = ceil(v - 1);
    int d = static_cast<int>(v);
    return d;
}

double ExplorationThread::discrete_to_continuous(int disc, double res) const
{
    double s = (static_cast<double>(disc) * res) + (res / 2.0);
    return s;
}

void ExplorationThread::goal_locations_to_path_msg(
    const std::vector<Locations_c>& goal_locations,
    nav_msgs::Path& msg) const
{
    msg.header.frame_id = frame_id_;
    msg.header.stamp = ros::Time::now();
    msg.poses.resize(2);

    //convert goals to world frame, continuous
    msg.poses[0].pose.position.x = discrete_to_continuous(goal_locations[0].x, resolution_) + origin_x_;
    msg.poses[0].pose.position.y = discrete_to_continuous(goal_locations[0].y, resolution_) + origin_y_;
    msg.poses[0].pose.position.z = discrete_to_continuous(goal_locations[0].z, resolution_) + origin_z_;
    yaw_to_quaternion(discrete_anngle_to_continuous(goal_locations[0].theta, angle_resolution_), msg.poses[0].pose.orientation);

    msg.poses[1].pose.position.x = discrete_to_continuous(goal_locations[1].x, resolution_) + origin_x_;
    msg.poses[1].pose.position.y = discrete_to_continuous(goal_locations[1].y, resolution_) + origin_y_;
    msg.poses[1].pose.position.z = discrete_to_continuous(goal_locations[1].z, resolution_) + origin_z_;
    yaw_to_quaternion(discrete_anngle_to_continuous(goal_locations[1].theta, angle_resolution_), msg.poses[1].pose.orientation);
}

void ExplorationThread::publish_goal_cloud()
{
    //convert to list of points and publish point cloud
    std::vector<pcl::PointXYZI> points;
    for (auto & pose : last_goals_.poses) {
        pcl::PointXYZI p;
        p.x = pose.pose.position.x;
        p.y = pose.pose.position.y;
        p.z = pose.pose.position.z;
        points.push_back(p);
    }
    publish_point_cloud(points, goal_point_cloud_pub_);
}

void ExplorationThread::quaternion_to_yaw(const geometry_msgs::Quaternion& quat, double& yaw) const
{
    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(quat, tf_quat);
    yaw = tf::getYaw(tf_quat);
}

void ExplorationThread::yaw_to_quaternion(const double& yaw, geometry_msgs::Quaternion& quat) const
{
    const geometry_msgs::Quaternion q;
    tf::Quaternion tf_quat = tf::createQuaternionFromRPY(0, 0, yaw);
    tf::quaternionTFToMsg(tf_quat, quat);
}

int ExplorationThread::continuous_angle_to_discrete(double cont, double res) const
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

int ExplorationThread::discrete_anngle_to_continuous(int disc, double res) const
{
    double scaled = static_cast<double>(disc);
    return scaled * res;
}

void ExplorationThread::publish_point_cloud(const std::vector<pcl::PointXYZI>& points, const ros::Publisher& publisher)
{
    //make point cloud for goals
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.header.stamp = ros_to_pcl_time_now;
    cloud.header.frame_id = frame_id_;
    cloud.height = 1;

    cloud.points.reserve(points.size());
    for (auto & p : points) {
        cloud.points.push_back(p);
    }
    cloud.width = cloud.points.size();
    publisher.publish(cloud);
}

void ExplorationThread::publish_planner_maps()
{
    ROS_DEBUG("publishing maps....");

    ros::Time now = ros::Time::now();

    std::vector<pcl::PointXYZI> frontier_points;
    get_point_cloud_from_points(EP_.Frontier3d_, frontier_points);
    publish_point_cloud(frontier_points, frontier_map_pub_);

    for (std::size_t i = 0; i < params_.robots.size(); ++i) {
        ROS_DEBUG("Publishing maps for robot %zd", i);
        nav_msgs::OccupancyGrid motion_height_grid;
        this->get_occupancy_grid_from_coverage_map(i, now, motion_height_grid);
        coverage_map_pub_.at(i).publish(motion_height_grid);

        nav_msgs::OccupancyGrid distance_transform_grid;
        const auto& coverage_map = EP_.coverage_;
        this->get_occupancy_grid_from_distance_transform(coverage_map, i, now, distance_transform_grid);
        dist_transform_pub_.at(i).publish(distance_transform_grid);

        ROS_DEBUG("  Publishing costmap...");
        nav_msgs::OccupancyGrid costmap_grid;
        this->get_occupancy_grid_from_costmap(EP_.CostToPts_.at(i), now, costmap_grid);
        cost_map_pub_.at(i).publish(costmap_grid);

        ROS_DEBUG("  Publishing countmap...");
        nav_msgs::OccupancyGrid count_grid;
        get_occupancy_grid_from_countmap(EP_.counts_.at(i), now, count_grid);
        counts_map_pub_.at(i).publish(count_grid);

        ROS_DEBUG("  Publishing scoremap");
        nav_msgs::OccupancyGrid score_grid;
        get_occupancy_grid_from_countmap(EP_.scores_.at(i), now, score_grid);
        score_map_pub_.at(i).publish(score_grid);
    }

    ROS_DEBUG("done");
}

void ExplorationThread::get_occupancy_grid_from_coverage_map(
    int ridx,
    const ros::Time& time,
    nav_msgs::OccupancyGrid& map) const
{
    assert((int)EP_.coverage_.x_size_ == size_x_ && (int)EP_.coverage_.y_size_ == size_y_);

    map.header.frame_id = frame_id_;
    map.header.stamp = time;

    map.info.width = size_x_;
    map.info.height = size_y_;
    map.info.origin.position.x = origin_x_;
    map.info.origin.position.y = origin_y_;
    map.info.origin.position.z = origin_z_;
    map.info.resolution = resolution_;
    map.data.resize(size_x_ * size_y_);

    for (std::size_t x = 0; x < (std::size_t)size_x_; ++x) {
        for (std::size_t y = 0; y < (std::size_t)size_y_; ++y) {
            char val = EP_.coverage_.GetMotionLevelValue(ridx, x, y);
            if ((unsigned char)val == params_.unk) {
                map.data[y * size_x_ + x] = -1;
            }
            else if ((unsigned char)val == params_.freespace) {
                map.data[y * size_x_ + x] = 0;
            }
            else if ((unsigned char)val == params_.obs) {
                map.data[y * size_x_ + x] = 100;
            }
            else {
                ROS_ERROR("Unexpected 3-D map value");
            }
        }
    }
}

void ExplorationThread::get_occupancy_grid_from_costmap(
    const ExplorationPlanner::CostMap& costmap,
    const ros::Time& time,
    nav_msgs::OccupancyGrid& map) const
{
    map.header.frame_id = frame_id_;
    map.header.stamp = time;

    map.info.width = size_x_;
    map.info.height = size_y_;
    map.info.origin.position.x = origin_x_;
    map.info.origin.position.y = origin_y_;
    map.info.origin.position.z = origin_z_;
    map.info.resolution = resolution_;

    map.data.resize(size_x_ * size_y_);

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
                map.data[y * size_x_ + x] = 0xFF;
            }
            else {
                if (span == 0.0) {
                    map.data[y * size_x_ + x] = -1;
//                    map.data[y * size_x_ + x] = 100;
                }
                else {
                    map.data[y * size_x_ + x] = (100 * ((cost - min_cost) / span));
                }
            }
        }
    }
}

void ExplorationThread::get_occupancy_grid_from_countmap(
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
                costmap(x, y) = std::max(costmap(x, y), countmap(x, y, a)); //countmap(x, y, a);
            }
        }
    }

    return this->get_occupancy_grid_from_costmap(costmap, time, map);
}

void ExplorationThread::get_occupancy_grid_from_distance_transform(
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
void ExplorationThread::get_point_cloud_from_map(const std::vector<T> &map, std::vector<pcl::PointXYZI> & points)
{
    std::vector<int> indicies;
    get_point_cloud_from_inner_dim(map, points, indicies, 0);
}

template<typename T>
void ExplorationThread::get_point_cloud_from_inner_dim(
    const std::vector<T>& map,
    std::vector<pcl::PointXYZI>& points,
    std::vector<int> & indicies,
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
void ExplorationThread::get_point_cloud_from_inner_dim(
    const T& val,
    std::vector<pcl::PointXYZI>& points,
    std::vector<int> & indicies,
    int depth)
{
    static int count = 0;
    count++;
    pcl::PointXYZI point;
    if (depth > 0) {
        point.x = discrete_to_continuous(indicies[0], resolution_) + origin_x_;
    }

    if (depth > 1) {
        point.y = discrete_to_continuous(indicies[1], resolution_) + origin_y_;
    }

    if (depth > 2) {
        point.z = discrete_to_continuous(indicies[2], resolution_) + origin_z_;
    }
    point.intensity = static_cast<double>(val);
    if (point.intensity > 0) {
        points.push_back(point);
    }
}

template<typename T>
inline void ExplorationThread::get_point_cloud_from_points(
    const std::vector<T>& point_list,
    std::vector<pcl::PointXYZI>& points)
{
    // convert to 3d points
    for (auto & lp : point_list) {
        pcl::PointXYZI p;
        p.x = discrete_to_continuous(lp.x, resolution_) + origin_x_;
        p.y = discrete_to_continuous(lp.y, resolution_) + origin_y_;
        p.z = discrete_to_continuous(lp.z, resolution_) + origin_z_;
        p.intensity = lp.cost;
        points.push_back(p);
    }
}
