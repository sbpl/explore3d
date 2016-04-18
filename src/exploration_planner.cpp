/// 3-D Exploration Library
/// (c) 2014 Jonathan Michael Butzke
///
/// Functions written by Jonathan Michael Butzke except where otherwise indicated
/// uses Bresenham line algorithm for rays
/// (c) 2009, 2014

#include <explore3d/exploration_planner.hpp>

#include <cassert>
#include <algorithm>

#include <ros/console.h>
#include <explore3d/Grid.h>

ExplorationPlanner::ExplorationPlanner() :
    completion_pct_(0.5)
{

}

ExplorationPlanner::~ExplorationPlanner() {
  fclose(datafile_);
}


void ExplorationPlanner::Init(ExpParams_c initparams)
{
    FREESPACE = initparams.freespace;
    UNK = initparams.unk;
    OBS = initparams.obs;
    ObjectMaxElev_ = initparams.ObjectMaxElev;
    NumAngles_ = initparams.NumAngles;
    MinDist_ = initparams.MinDist;

    robots_ = initparams.robots;
    coverage_.Init(
            initparams.size_x,
            initparams.size_y,
            initparams.size_z,
            FREESPACE,
            UNK,
            OBS,
            &robots_);

    backwards_penalty_ = initparams.backwards_penalty;

    room_min_x_ = initparams.room_min_x;
    room_min_y_ = initparams.room_min_y;
    room_max_x_ = initparams.room_max_x;
    room_max_y_ = initparams.room_max_y;

    CostToPts_.resize(robots_.size());
    counts_.resize(robots_.size());
    scores_.resize(robots_.size());
    for (size_t ridx = 0; ridx < robots_.size(); ridx++) {
        CostToPts_[ridx].resize(coverage_.x_size_, coverage_.y_size_);
        counts_[ridx].resize(coverage_.x_size_, coverage_.y_size_, NumAngles_);
        scores_[ridx].resize(coverage_.x_size_, coverage_.y_size_, NumAngles_);
    }

    goal_.resize(robots_.size());

    GenMotionSteps();
    GenVisibilityRing();
}

void ExplorationPlanner::GenMotionSteps(void)
{
    mp_.clear();
    SearchPts_c temp;
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            if (dx != 0 || dy != 0) {
                temp.x = dx;
                temp.y = dy;
                temp.z = 0;
                temp.cost = sqrt(dx * dx + dy * dy);
                mp_.push_back(temp);
            }
        }
    }
}

void ExplorationPlanner::GenVisibilityRing(void)
{
    VisibilityRings_.resize(robots_.size());
    for (size_t ridx = 0; ridx < VisibilityRings_.size(); ridx++) {
        VisibilityRings_[ridx].resize(coverage_.z_size_);
        for (size_t zidx = 0; zidx < VisibilityRings_[ridx].size(); zidx++) {
            double height, radius;
            height = fabs((double) robots_[ridx].SensorHeight_ - (double) zidx);
            radius = height / tan(robots_[ridx].VerticalFOV_ / 2);
            if (radius < 2) {
                radius = 2;
            }
//            printf("robot %i height=%f radius = %f\n", (int)ridx, height, radius);
            if (radius < robots_[ridx].DetectionRange_) {
                pts2d temp_pt;
                for (double aidx = 0; aidx < 2 * M_PI; aidx += 0.01) {
                    temp_pt.x = radius * sin(aidx) + 0.5;
                    temp_pt.y = radius * cos(aidx) + 0.5;
                    double tangle = atan2(temp_pt.y, temp_pt.x);
                    if (tangle < 0) {
                        tangle += 2 * M_PI;
                    }
                    temp_pt.theta = (int) (tangle * (double) NumAngles_ / (2 * M_PI));
                    VisibilityRings_[ridx][zidx].push_back(temp_pt);
                }
                std::sort(VisibilityRings_[ridx][zidx].begin(), VisibilityRings_[ridx][zidx].end(), ptssort);
                std::vector<pts2d>::iterator it;
                it = std::unique(VisibilityRings_[ridx][zidx].begin(), VisibilityRings_[ridx][zidx].end(), ptscompare);
                VisibilityRings_[ridx][zidx].resize(std::distance(VisibilityRings_[ridx][zidx].begin(), it));
//                for (const auto& ring : VisibilityRings_[ridx][zidx]) {
//                    printf("r%i z%i x%i y%i a%i\n", (int)ridx, (int)zidx, ring.x, ring.y, ring.theta);
//                }
            }
        }
    }
}

void ExplorationPlanner::ClearCounts()
{
    for (size_t ridx = 0; ridx < robots_.size(); ++ridx) {
        counts_[ridx].assign(0.0);
        scores_[ridx].assign(0.0);
    }
}

void ExplorationPlanner::ClearInformationGain(int ridx)
{
    counts_[ridx].assign(0.0);
    counts_[ridx].assign(0.0);
}

void ExplorationPlanner::printMap(int h)
{
    for (int yidx = coverage_.y_size_ - 1; yidx >= 0; yidx--) {
        printf("%4i ", yidx);
        for (uint xidx = 0; xidx < coverage_.x_size_; xidx++) {
            if (coverage_.Getval(xidx, yidx, h) == UNK)
                printf("u");
            else if (coverage_.Getval(xidx, yidx, h) == OBS)
                printf("#");
            else if (coverage_.Getval(xidx, yidx, h) == FREESPACE)
                printf("_");
        }
        printf("\n");
    }
}

void ExplorationPlanner::printCosts(uint x0, uint y0, uint x1, uint y1, uint rn)
{
    for (uint yidx = y1 - 1; yidx >= y0; yidx--) {
        printf("%4i ", yidx);
        for (uint xidx = x0; xidx < x1; xidx++) {
            printf("%4.0f ", CostToPts_[rn](xidx, yidx));
        }
        printf("\n");
    }
}

void ExplorationPlanner::printCounts(uint x0, uint y0, uint x1, uint y1, uint rn)
{
    for (uint yidx = y1 - 1; yidx >= y0; yidx--) {
        printf("%4i ", yidx);
        for (uint xidx = x0; xidx < x1; xidx++) {
            int c = 0;
            for (uint aidx = 0; aidx < NumAngles_; aidx++) {
                c += counts_[rn](xidx, yidx, aidx);
            }
            printf("%4i ", c);
        }
        printf("\n");
    }
}

bool ExplorationPlanner::ComputeTraversalCosts(const Locations_c& start, int ridx)
{
    Locations_c cfree_start;
    if (!FindNearestCollisionFreeCell(start, ridx, cfree_start)) {
        ROS_ERROR("Failed to compute collision free start state");
        return false;
    }

    if (cfree_start != start) {
        ROS_WARN("Exploration Planner cell cost expansion start state invalid: %s. Found nearby valid state: %s", to_string(start).c_str(), to_string(cfree_start).c_str());
    }

    // pre-allocate and initialize search state table
    au::Grid<2, SearchPtState> states(coverage_.x_size_, coverage_.y_size_);
    for (std::size_t x = 0; x < states.size(0); ++x) {
        for (std::size_t y = 0; y < states.size(1); ++y) {
            states(x, y).set_x(x);
            states(x, y).set_y(y);
            states(x, y).set_z(start.z);
            states(x, y).set_theta(start.theta);
            states(x, y).set_cost(MaxCost);
            states(x, y).set_closed(false);
        }
    }

    // initialize the cell expansion
    CHeap OPEN;
    SearchPtState& cfree_start_state = states(cfree_start.x, cfree_start.y);
    cfree_start_state.set_cost(10.0); // ANDREW: why is this 10?
    CKey collision_free_start_key = CreateKey(cfree_start_state.cost());
    OPEN.insertheap(&cfree_start_state, collision_free_start_key);

    // perform the cell expansion
    while (!OPEN.emptyheap()) {
        SearchPtState* curr = (SearchPtState*)OPEN.deleteminheap();
        curr->set_closed(true);

        for (size_t midx = 0; midx < mp_.size(); midx++) {
            // skip out-of-bounds cells
            int succ_x = curr->x() + mp_[midx].x;
            int succ_y = curr->y() + mp_[midx].y;
            if (succ_x < 0 || succ_y < 0 ||
                succ_x >= (int)states.size(0) || succ_y >= (int)states.size(1))
            {
                continue;
            }

            SearchPtState& succ = states(succ_x, succ_y);

            // check that the robot isn't in collision in this cell
            const int robot_size = robots_[ridx].CircularSize_;
            const bool is_valid = coverage_.OnInflatedMap(
                    succ.x(), succ.y(), succ.z(), ridx, robot_size);
            if (!is_valid) {
                continue;
            }

            // check that this cell is an explored cell (freespace is a bit of
            // a misnomer)
            const bool is_freespace =
                    coverage_.Getval(succ.x(), succ.y(), succ.z()) == FREESPACE;
            if (!is_freespace) {
                continue;
            }

            // valid successor at this point

            if (!succ.closed()) {
                CostType penalty = ComputeMotionPenalty(start, *curr, succ);
                CostType new_cost = curr->cost() + penalty * mp_[midx].cost;
                if (new_cost < succ.cost()) {
                    succ.set_cost(new_cost);
                    CKey succkey = CreateKey(new_cost);
                    if (OPEN.inheap(&succ)) {
                        OPEN.updateheap(&succ, succkey);
                    }
                    else {
                        OPEN.insertheap(&succ, succkey);
                    }
                }
            }
        }
    }

    // mark additional cells as invalid places for the robot to go to

    // invalidate cells outside of the defined 'room'
    for (std::size_t x = 0; x < states.size(0); ++x) {
        for (std::size_t y = 0; y < states.size(1); ++y) {
            if (!inside_room(x, y)) {
                states(x, y).set_cost(MaxCost);
            }
        }
    }

    // invalidate cells that are within 'neighbor' cells of the start cell
    const int neighbor = 8;
    for (int x = -neighbor; x <= neighbor; ++x) {
        for (int y = -neighbor; y <= neighbor; ++y) {
            double dist = sqrt((double)(x * x + y * y));
            if (dist > neighbor) {
                continue;
            }

            int x_coord = start.x + x;
            int y_coord = start.y + y;
            if (x_coord < 0 || x_coord > (int)states.size(0) - 1 ||
                y_coord < 0 || y_coord > (int)states.size(1) - 1)
            {
                continue;
            }

            states(x_coord, y_coord).set_cost(MaxCost);
        }
    }

    // copy state costs-to-go into costmap
    CostMap& costmap = CostToPts_[ridx];
    for (size_t x = 0; x < states.size(0); ++x) {
        for (size_t y = 0; y < states.size(1); ++y) {
            costmap(x, y) = states(x, y).cost();
        }
    }

    return true;
}

bool ExplorationPlanner::FindNearestCollisionFreeCell(
    const Locations_c& start,
    int robotnum,
    Locations_c& out) const
{
    // TODO: check if the input start state is free beforehand to avoid
    // allocating unnecessary resources

    ROS_INFO("Finding nearest collision-free cell to (%d, %d, %d)", start.x, start.y, start.z);

    // Preallocate search state table
    au::Grid<2, SearchPtState> states(coverage_.x_size_, coverage_.y_size_);
    for (std::size_t x = 0; x < states.size(0); ++x) {
        for (std::size_t y = 0; y < states.size(1); ++y) {
            states(x, y).set_x(x);
            states(x, y).set_y(y);
            states(x, y).set_z(start.z);
            states(x, y).set_theta(start.theta);
            states(x, y).set_cost(MaxCost);
            states(x, y).set_closed(false);
        }
    }

    // initialize the dijkstra search
    CHeap OPEN;
    SearchPtState& start_state = states(start.x, start.y);
    start_state.set_cost(0.0);
    CKey startkey = CreateKey(start_state.cost());
    OPEN.insertheap(&start_state, startkey);

    // the goal state
    Locations_c cfree_start;
    cfree_start.x = -1;
    cfree_start.y = -1;
    cfree_start.z = -1;
    cfree_start.theta = -1;

    // perform the dijkstra search to find the nearest collision-free cell
    int num_expands = 0;
    while (!OPEN.emptyheap()) {
        SearchPtState* curr = (SearchPtState*)OPEN.deleteminheap();
        curr->set_closed(true);

        // check for goal state
        const int robot_size = robots_[robotnum].CircularSize_;
        const bool curr_is_valid = coverage_.OnInflatedMap(curr->x(), curr->y(), curr->z(), robotnum, robot_size);
        // shouldn't this be redundant with the above check?
        const bool is_freespace = curr_is_valid && coverage_.Getval(curr->x(), curr->y(), curr->z()) == FREESPACE;
        const bool is_valid = curr_is_valid && is_freespace;
        if (is_valid) {
            // update the real collision-free start state and exit the search
            cfree_start.x = curr->x();
            cfree_start.y = curr->y();
            cfree_start.z = curr->z();
            cfree_start.theta = curr->theta();
            break;
        }

        ++num_expands;
        for (size_t midx = 0; midx < mp_.size(); midx++) {
            int succ_x = curr->x() + mp_[midx].x;
            int succ_y = curr->y() + mp_[midx].y;
            if (succ_x < 0 || succ_y < 0 || succ_x >= (int)states.size(0) || succ_y >= (int)states.size(1)) {
                continue;
            }
            SearchPtState& succ = states(succ_x, succ_y);

            // note: no is_valid check for start-in-collision search

            if (!succ.closed()) {
                CostType succ_cost = succ.cost();
                // note: remove arbitrary penalty
                CostType new_cost = curr->cost() + mp_[midx].cost;
                if (new_cost < succ_cost) {
                    succ.set_cost(new_cost);
                    CKey succkey = CreateKey(new_cost);
                    if (OPEN.inheap(&succ)) {
                        OPEN.updateheap(&succ, succkey);
                    }
                    else {
                        OPEN.insertheap(&succ, succkey);
                    }
                }
            }
        }
    }

    if (cfree_start.x >= 0) {
        assert(cfree_start.y >= 0 && cfree_start.z >= 0 && cfree_start.theta >= 0);
        out = cfree_start;
        ROS_INFO("Found a collision-free start cell at (%d, %d, %d)", cfree_start.x, cfree_start.y, cfree_start.z);
        return true;
    }
    else {
        ROS_ERROR("Failed to find a collision free start state after %d expansions", num_expands);
        return false;
    }
}

CostType ExplorationPlanner::EvalFxn(uint x, uint y, uint z, uint a, uint rn) const
{
    if (CostToPts_[rn](x, y) == MaxCost) {
        return 0;
    }

    double dist = 1e99;
    for (size_t ridx = 0; ridx < goal_.size(); ridx++) {
        if (ridx != rn) {
            double temp_dist =
                    (x - goal_[ridx].x) * (x - goal_[ridx].x) +
                    (y - goal_[ridx].y) * (y - goal_[ridx].y) +
                    (z - goal_[ridx].z) * (z - goal_[ridx].z);
            if (temp_dist < dist) {
                dist = temp_dist;
            }
        }
    }

    if (dist < MinDist_ * MinDist_) {
        dist /= (MinDist_ * MinDist_);
    }
    else {
        dist = 1;
    }

    return counts_[rn](x, y, a) * CostToPts_[rn](x, y) * dist;
}

bool ExplorationPlanner::ComputeInformationGain(int ridx)
{
    Frontier3d_ = coverage_.GetFrontier3d();
    return ComputeInformationGain(ridx, Frontier3d_);
}

bool ExplorationPlanner::ComputeInformationGain(
    int ridx,
    const std::vector<SearchPts_c>& frontier)
{
    counts_[ridx].assign(0.0);

    // TODO: various hacks to enforce different behavior for the
    // hexacopter...factor out per-robot cost functions or something instead of
    // making it the default behavior for robot 2 -_-
    const size_t HEXA_IDX = 1;

    // for each frontier cell, determine viewing cells by raycasting
    // outwards to intersect robot sensor planes
    for (size_t pidx = 0; pidx < frontier.size(); pidx++) {
        if (ridx == HEXA_IDX) {
            raycast3d_hexa(frontier[pidx], ridx);
        }
        else {
            raycast3d(frontier[pidx], ridx);
        }
    }

    if (ridx == HEXA_IDX) {
        // check if there are no cells that can be seen exclusively by the
        // hexacopter...
        bool no_counts = true;
        for (auto git = counts_[HEXA_IDX].begin(); git != counts_[HEXA_IDX].end(); ++git) {
            if (*git != 0.0) {
                no_counts = false;
                break;
            }
        }

        if (no_counts) {
            ROS_WARN("No cells only visible by hexacopter...reverting to normal cost function");
            // apply the normal cost function
            for (size_t pidx = 0; pidx < frontier.size(); pidx++) {
                raycast3d(frontier[pidx], ridx);
            }
        }
    }

    return true;
}

void ExplorationPlanner::SelectGoal(size_t ridx)
{
    goal_[ridx].cost = 0;
    goal_[ridx].z = robots_[ridx].MotionHeight_;

    // clear scores
    scores_[ridx].assign(0.0);

    // evaluate the scores for each cell
    for (auto sit = scores_[ridx].begin(); sit != scores_[ridx].end(); ++sit) {
        (*sit) = EvalFxn(sit.coord(0), sit.coord(1), goal_[ridx].z, sit.coord(2), ridx);
    }

    // take the cell with the highest score
    auto maxe = std::max_element(scores_[ridx].begin(), scores_[ridx].end());
    assert(maxe != scores_[ridx].end());

    // fill in the goal
    goal_[ridx].cost = *maxe;
    goal_[ridx].x = maxe.coord(0);
    goal_[ridx].y = maxe.coord(1);
    goal_[ridx].theta = maxe.coord(2);
}

bool ExplorationPlanner::NewGoal(
    size_t ridx,
    std::vector<Locations_c> robot_poses,
    Locations_c& goal)
{
    Locations_c robot_pose = robot_poses[ridx];
    robot_pose.z = robots_[ridx].MotionHeight_;

    if (!ComputeTraversalCosts(robot_pose, ridx)) {
        ROS_ERROR("Failed to compute traversal costs for robot %zu", ridx);
        return false;
    }

    if (!ComputeInformationGain(ridx)) {
        ROS_ERROR("Failed to compute information gain for robot %zu", ridx);
        return false;
    }

    SelectGoal(ridx);

    // copy over goal
    goal.x = goal_[ridx].x;
    goal.y = goal_[ridx].y;
    goal.z = goal_[ridx].z;
    goal.theta = goal_[ridx].theta;

LogData(robot_poses);
    return true;
}

std::vector<Locations_c> ExplorationPlanner::NewGoals(
    std::vector<Locations_c> robot_poses)
{
    for (size_t ridx = 0; ridx < robot_poses.size(); ++ridx) {
        robot_poses[ridx].z = robots_[ridx].MotionHeight_;
        if (!ComputeTraversalCosts(robot_poses[ridx], ridx)) {
            ROS_ERROR("Failed to compute cell cost expansion");
            return { };
        }
    }

    Frontier3d_ = coverage_.GetFrontier3d();
    for (size_t ridx = 0; ridx < robot_poses.size(); ++ridx) {
        if (!ComputeInformationGain(ridx, Frontier3d_)) {
            ROS_ERROR("Failed to compute information gain for robot %zu", ridx);
            return { };
        }
    }

    for (size_t ridx = 0; ridx < robots_.size(); ridx++) {
        SelectGoal(ridx);
    }

    std::vector<Locations_c> goals;
    goals.resize(robots_.size());
    for (size_t ridx = 0; ridx < goal_.size(); ridx++) {
        goals[ridx].x = goal_[ridx].x;
        goals[ridx].y = goal_[ridx].y;
        goals[ridx].z = goal_[ridx].z;
        goals[ridx].theta = goal_[ridx].theta;
    }
    return goals;
}

double ExplorationPlanner::EstimatedCompletionPercent() const
{
    return completion_pct_.load();
}

void ExplorationPlanner::UpdateMap(CoverageMap_c newmap)
{
    coverage_ = newmap;
    coverage_.UpdateDistances();
}

void ExplorationPlanner::PartialUpdateMap(const std::vector<MapElement_c>& pts)
{
    for (size_t pidx = 0; pidx < pts.size(); pidx++) {
        coverage_.Setval(pts[pidx].x, pts[pidx].y, pts[pidx].z, pts[pidx].data);
    }
    coverage_.UpdateDistances();
}

CKey ExplorationPlanner::CreateKey(double val) const
{
    const int fpscale = 1000;
    CKey key;
    key.key[0] = fpscale * val;
    return key;
}

CostType ExplorationPlanner::ComputeMotionPenalty(
    const Locations_c& start,
    const SearchPtState& s,
    const SearchPtState& t)
{
    // TODO: fixme to take in cost parameter in cells derived from meters
    const double preferred_min_distance_cells = 3;

    // Capture a bunch of hacky rules about traversal costs between states
    const int dx = t.x() - start.x;
    const int dy = t.y() - start.y;
    double end_theta = atan2((double)dy, (double)dx);
    double start_theta = DiscTheta2Cont(start.theta, NumAngles_);
    double angle_diff = computeMinUnsignedAngleDiff(end_theta, start_theta);
    double final_backwards_penalty = backwards_penalty_ * angle_diff / M_PI;

    double distance = sqrt((double)(dx * dx + dy * dy));
    const double arbitrary_short_distance_penalty = 1000.0;
    double close_penalty = 1.0; // = distance < (double)preferred_min_distance_cells ? arbitrary_short_distance_penalty : 1.0;
    if (distance < (double)preferred_min_distance_cells) {
        close_penalty = arbitrary_short_distance_penalty;
    }

    return std::max({ close_penalty, final_backwards_penalty, 1.0 });
}

bool ExplorationPlanner::bresenham_line_3D(int x1, int y1, int z1, int x2, int y2, int z2)
{
    /// code from http://www.ict.griffith.edu.au/anthony/info/graphics/bresenham.procs as accessed 28oct2014 - copyright retained by original author
    int i, dx, dy, dz, l, m, n, x_inc, y_inc, z_inc, err_1, err_2, dx2, dy2, dz2;
    int pixel[3];

    pixel[0] = x1;
    pixel[1] = y1;
    pixel[2] = z1;
    dx = x2 - x1;
    dy = y2 - y1;
    dz = z2 - z1;

    x_inc = (dx < 0) ? -1 : 1;
    l = abs(dx);
    y_inc = (dy < 0) ? -1 : 1;
    m = abs(dy);
    z_inc = (dz < 0) ? -1 : 1;
    n = abs(dz);

    dx2 = l << 1;
    dy2 = m << 1;
    dz2 = n << 1;

    if ((l >= m) && (l >= n)) {
        err_1 = dy2 - l;
        err_2 = dz2 - l;
        for (i = 0; i < l; i++) {
            if (coverage_.Getval(pixel[0], pixel[1], pixel[2]) == OBS) {
                return false;
            }
            if (err_1 > 0) {
                pixel[1] += y_inc;
                err_1 -= dx2;
            }
            if (err_2 > 0) {
                pixel[2] += z_inc;
                err_2 -= dx2;
            }
            err_1 += dy2;
            err_2 += dz2;
            pixel[0] += x_inc;
        }
    }
    else if ((m >= l) && (m >= n)) {
        err_1 = dx2 - m;
        err_2 = dz2 - m;
        for (i = 0; i < m; i++) {
            if (coverage_.Getval(pixel[0], pixel[1], pixel[2]) == OBS) {
                return false;
            }
            if (err_1 > 0) {
                pixel[0] += x_inc;
                err_1 -= dy2;
            }
            if (err_2 > 0) {
                pixel[2] += z_inc;
                err_2 -= dy2;
            }
            err_1 += dx2;
            err_2 += dz2;
            pixel[1] += y_inc;
        }
    }
    else {
        err_1 = dy2 - n;
        err_2 = dx2 - n;
        for (i = 0; i < n; i++) {
            if (coverage_.Getval(pixel[0], pixel[1], pixel[2]) == OBS) {
                return false;
            }
            if (err_1 > 0) {
                pixel[1] += y_inc;
            err_1 -= dz2;
            }
            if (err_2 > 0) {
                pixel[0] += x_inc;
                err_2 -= dz2;
            }
            err_1 += dy2;
            err_2 += dx2;
            pixel[2] += z_inc;
        }
    }

    return true;
}

void ExplorationPlanner::raycast3d(const SearchPts_c& start, int robotnum)
{
    if (!inside_room(start.x, start.y)) {
        return;
    }
    // TODO: multi-thread the casting to different points

    // place the visibility ring around the frontier point and attempt to raycast to it
    auto& count_map = counts_[robotnum];
    for (size_t pidx = 0; pidx < VisibilityRings_[robotnum][start.z].size(); pidx++) {
        int x, y, z, theta;
        x = VisibilityRings_[robotnum][start.z][pidx].x + start.x;
        y = VisibilityRings_[robotnum][start.z][pidx].y + start.y;
        if (!inside_room(start.x, start.y) || !inside_room(x, y)) {
            continue;
        }
        z = robots_[robotnum].SensorHeight_;
        // note: leaving this here to serve as a reminder of the worst bug of the UTACC Project
        // theta = (VisibilityRings_[robotnum][start.z][pidx].theta) % NumAngles_;
        theta = (VisibilityRings_[robotnum][start.z][pidx].theta + (NumAngles_ >> 1)) % NumAngles_;
        if (coverage_.Getval(x, y, z) == FREESPACE) {
            bool result = bresenham_line_3D(start.x, start.y, start.z, x, y, z);
            if (result) {
                if (start.z <= z) { // HAX!: double-count cells at or beneath the sensor height
                    count_map(x, y, theta) += 2.0;
                }
                else {
                    count_map(x, y, theta) += 1.0;
                }
            }
        }
    }
}

void ExplorationPlanner::raycast3d_hexa(const SearchPts_c& start, int hexanum)
{
    if (!inside_room(start.x, start.y)) {
        return;
    }

    const int HEXA_IDX = 1;

    // for every robot
    for (int ridx = 0; ridx < (int)robots_.size(); ++ridx) {
        // other than the hexa
        if (ridx != HEXA_IDX) {
            if (coverage_.GetMotionLevelValue(ridx, start.x, start.y) != OBS) {
                // skip if there is not on obstacle on this robot's coverage map at this (x, y) coord
                return;
            }
        }
    }

    auto& count_map = counts_[hexanum];
    for (size_t pidx = 0; pidx < VisibilityRings_[hexanum][start.z].size(); pidx++) {
        int x, y, z, theta;
        x = VisibilityRings_[hexanum][start.z][pidx].x + start.x;
        y = VisibilityRings_[hexanum][start.z][pidx].y + start.y;
        z = robots_[hexanum].SensorHeight_;
        // note: leaving this here to serve as a reminder of the worst bug of the UTACC Project
        // theta = (VisibilityRings_[hexanum][start.z][pidx].theta) % NumAngles_;
        theta = (VisibilityRings_[hexanum][start.z][pidx].theta + (NumAngles_ >> 1)) % NumAngles_;
        if (coverage_.Getval(x, y, z) == FREESPACE) {
            bool result = bresenham_line_3D(start.x, start.y, start.z, x, y, z);
            if (result) {
                count_map(x, y, theta) += 1.0;
            }
        }
    }
}

bool ExplorationPlanner::inside_room(int x, int y) const
{
    if (room_min_x_ < 0 || room_min_y_ < 0 || room_max_x_ > (int)coverage_.x_size_ - 1 || room_max_y_ > (int)coverage_.y_size_ - 1) {
        ROS_WARN_THROTTLE(1.0, "Invalid room boundaries [%d,%d] X [%d,%d]", room_min_x_, room_min_y_, room_max_x_, room_max_y_);
        return true;
    }
    return x >= room_min_x_ && x <= room_max_x_ && y >= room_min_y_ && y <= room_max_y_;
}

void ExplorationPlanner::LogData(std::vector<Locations_c> robot_poses) {
  if (!data_file_open) {
    std::ostringstream fname;
    fname << "logfile" << ros::Time::now().toSec();
    datafile_ = fopen(fname.str().c_str(), "w");
    if (datafile_!= NULL) { data_file_open= true; }
    fprintf(datafile_, "time (seconds)\tSegbot pose (discretized x,y,z,theta)\tHexa pose (discretized x,y,z,theta)\tPercent Explored\n");
  } else {
    //TODO: use actual indexes vice hardcoded numbers
    //TODO: Add percent explored
    fprintf(datafile_, "%5.1f %i %i %i %i %i %i %i %i\n", (double)ros::Time::now().toSec(), robot_poses[0].x, robot_poses[0].y, robot_poses[0].z, robot_poses[0].theta, robot_poses[1].x, robot_poses[1].y, robot_poses[1].z, robot_poses[1].theta);
  }
}
