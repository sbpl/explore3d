/// 3-D Exploration Library
/// (c) 2014 Jonathan Michael Butzke
#include "exploration.hpp"

/* inputs:
 * h *eight of hexa flight
 * height of hexa sensor
 * depression/FOV angles of hexa sensor
 * detection range of hexa sensor
 * height of segbot motion
 * height of segbot sensor
 * elevation/FOV  angles of segbot sensor
 * detection range of segbot sensor
 * occupancy grid of world
 */

//Get map
//x run Dijkstra on robot level to get cost to frontier points (and find accessable areas)
//x block off unaccessable areas (leave just the known/unknown boundary)
//x find areas in 3-d that are not surrounded by obstacles and border free space
//x  determine vertical frontiers based on 3-d ray casting to ground or hexa sensor level (use different depression angles for each out to detection range)

// if early in the process:
// weed out small groups
// do soemthing smart

// if not
// go to closest

#include <ros/console.h>
#include <sbpl/headers.h>
#include "Grid.h"

void ExplorationPlanner::Init(ExpParams_c initparams)
{
    FREESPACE = initparams.freespace;
    UNK = initparams.unk;
    OBS = initparams.obs;
    ObjectMaxElev_ = initparams.ObjectMaxElev;
    NumAngles_ = initparams.NumAngles;
    MinDist_ = initparams.MinDist;

    robots_ = initparams.robots;
    coverage_.Init(initparams.size_x, initparams.size_y, initparams.size_z, FREESPACE, UNK, OBS, &robots_);

    backwards_penalty_ = initparams.backwards_penalty;

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
            // printf("robot %i height=%f radius = %f\n", ridx, height, radius);
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
                // for (const auto& ring : VisibilityRings_[ridx][zidx]) {
                //     printf("r%i z%i x%i y%i a%i\n", ridx, zidx, ring.x, ring.y, ring.theta);
                // }
            }
        }
    }
}

void ExplorationPlanner::ClearCounts(void)
{
    for (size_t ridx = 0; ridx < robots_.size(); ridx++) {
        for (uint xidx = 0; xidx < coverage_.x_size_; xidx++) {
            for (uint yidx = 0; yidx < coverage_.y_size_; yidx++) {
                for (uint aidx = 0; aidx < NumAngles_; aidx++) {
                    counts_[ridx](xidx, yidx, aidx) = 0;
                    scores_[ridx](xidx, yidx, aidx) = 0;
                }
            }
        }
    }
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

void ExplorationPlanner::Dijkstra(const Locations_c& start, int robotnum)
{
    struct SearchPtState : public AbstractSearchState
    {
    public:

        SearchPtState() : AbstractSearchState(), search_pt_(), closed_(false)
        { heapindex = 0; search_pt_.theta = 0; search_pt_.cost = MaxCost; }

        int x() const { return search_pt_.x; }
        int y() const { return search_pt_.y; }
        int z() const { return search_pt_.z; }
        int theta() const { return search_pt_.theta; }
        CostType cost() const { return search_pt_.cost; }
        bool closed() const { return closed_; }

        void set_x(int x) { search_pt_.x = x; }
        void set_y(int y) { search_pt_.y = y; }
        void set_z(int z) { search_pt_.z = z; }
        void set_theta(int theta) { search_pt_.theta = theta; }
        void set_cost(CostType cost) { search_pt_.cost = cost; }
        void set_closed(bool closed) { closed_ = closed; }

        const SearchPts_c& search_pt() const { return search_pt_; }

    private:

        SearchPts_c search_pt_;
        bool closed_;
    };

    // Preallocate search state table
    au::Grid<2, SearchPtState> states(coverage_.x_size_, coverage_.y_size_);
    for (std::size_t x = 0; x < states.size(0); ++x) {
        for (std::size_t y = 0; y < states.size(1); ++y) {
            states(x, y).set_x(x);
            states(x, y).set_y(y);
            states(x, y).set_z(start.z);
            states(x, y).set_cost(MaxCost);
            states(x, y).set_closed(false);
        }
    }

    auto CreateKey = [](double val) -> CKey
    {
        const int fpscale = 1000;
        CKey key;
        key.key[0] = fpscale * val;
        return key;
    };

    SearchPtState& start_state = states(start.x, start.y);
    start_state.set_cost(10.0);

    CHeap OPEN;
    CKey startkey = CreateKey(start_state.cost());
    OPEN.insertheap(&start_state, startkey);

    // TODO: fixme to take in cost parameter in cells derived from meters
    const double preferred_min_distance_cells = 3; 

    auto compute_motion_penalty = [&](const SearchPtState& s, const SearchPtState& t)
    {
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
    };

    while (!OPEN.emptyheap()) {
        SearchPtState* curr = (SearchPtState*)OPEN.deleteminheap();
        curr->set_closed(true);

        for (size_t midx = 0; midx < mp_.size(); midx++) {
            SearchPtState& succ = states(curr->x() + mp_[midx].x, curr->y() + mp_[midx].y);

            const int robot_size = robots_[robotnum].CircularSize_;
            bool is_valid = coverage_.OnInflatedMap(succ.x(), succ.y(), succ.z(), robotnum, robot_size);
            bool is_freespace = is_valid && coverage_.Getval(succ.x(), succ.y(), succ.z()) == FREESPACE;
            const bool valid = is_valid && is_freespace;

            if (!valid) {
                continue;
            }

            if (!succ.closed()) {
                CostType succ_cost = succ.cost();
                CostType new_cost = curr->cost() + compute_motion_penalty(*curr, succ) * mp_[midx].cost;
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

    // copy over costmap
    CostMap& costmap = CostToPts_[robotnum];
    for (std::size_t x = 0; x < costmap.size(0); ++x) {
        for (std::size_t y = 0; y < costmap.size(1); ++y) {
            costmap(x, y) = states(x, y).cost();
        }
    }

    // disallow sending goals right up your butt
    const int neighbor = 4;
    const CostType FUCKTON = 10000.0;
    for (int x = -neighbor; x <= neighbor; ++x) {
        for (int y = -neighbor; y <= neighbor; ++y) {
            double dist = sqrt((double)(x * x + y * y));
            if (dist > neighbor) {
                continue;
            }
            int x_coord = start.x + x;
            int y_coord = start.y + y;
            if (x_coord < 0 || x_coord > costmap.size(0) -1 || y_coord < 0 || y_coord > costmap.size(1) -1) {
                continue;
            }
            else {
                costmap(x_coord, y_coord) = FUCKTON;
            }
        }
    }
}

CostType ExplorationPlanner::EvalFxn(uint x, uint y, uint z, uint a, uint rn)
{
    double dist = 1e99;
    for (size_t ridx = 0; ridx < goal_.size(); ridx++) {
        if (ridx != rn) {
            double temp_dist = (x - goal_[ridx].x) * (x - goal_[ridx].x) + (y - goal_[ridx].y) * (y - goal_[ridx].y) + (z - goal_[ridx].z) * (z - goal_[ridx].z);
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

    CostType rtnval = counts_[rn](x, y, a) / CostToPts_[rn](x, y) * dist;
    return rtnval;
}

void ExplorationPlanner::CreateFrontier(void)
{
    Frontier3d_ = coverage_.GetFrontier3d();
    ClearCounts();

    for (size_t ridx = 0; ridx < robots_.size(); ridx++) {
        // for each frontier cell, determine viewing cells by raycasting outwards to intersect robot sensor planes
        for (size_t pidx = 0; pidx < Frontier3d_.size(); pidx++) {
            raycast3d(Frontier3d_[pidx], ridx);
        }

        goal_[ridx].cost = 0;
        goal_[ridx].z = robots_[ridx].MotionHeight_;
        //printf("z is %i\n", goal_[ridx].z);
        for (size_t xidx = 0; xidx < coverage_.x_size_; xidx++) {
            for (size_t yidx = 0; yidx < coverage_.y_size_; yidx++) {
                for (uint aidx = 0; aidx < NumAngles_; aidx++) {
                    if (CostToPts_[ridx](xidx, yidx) != MaxCost)
                    {
                        auto score = EvalFxn(xidx, yidx, goal_[ridx].z, aidx, ridx);
                        scores_[ridx](xidx, yidx, aidx) = score;
                        if (score > goal_[ridx].cost) {
                            goal_[ridx].cost = score;
                            goal_[ridx].x = xidx;
                            goal_[ridx].y = yidx;
                            goal_[ridx].theta = aidx;
                            // printf("new goal: r%li x%li y%li z%i a%i count:%i cost:%f total:%f  obs:%f\n", ridx, xidx, yidx, goal_[ridx].z, aidx,counts_[ridx][xidx][yidx][aidx], CostToPts_[ridx][xidx][yidx], goal_[ridx].cost, coverage_.ReturnDistToObs(ridx, xidx, yidx) );
                        }
                    }
                }
            }
        }
    }
}

std::vector<Locations_c> ExplorationPlanner::NewGoals(
    std::vector<Locations_c> RobotLocations)
{
    for (size_t ridx = 0; ridx < RobotLocations.size(); ridx++) {
        RobotLocations[ridx].z = robots_[ridx].MotionHeight_;
        Dijkstra(RobotLocations[ridx], ridx);
        //printMap(RobotLocations[ridx].z);
    }

    CreateFrontier();

    // for (uint ridx =0; ridx < robots_.size(); ridx++) {
    //   printf("Robot %i\n", ridx);
    //   for (uint pidx=0; pidx < VisibilityRings_[ridx][robots_[ridx].SensorHeight_].size(); pidx++) {
    //     printf("(%i %i)", VisibilityRings_[ridx][robots_[ridx].SensorHeight_][pidx].x, VisibilityRings_[ridx][robots_[ridx].SensorHeight_][pidx].y);
    //   }
    //   printf("\n");
    // }

    std::vector<Locations_c> goals;
    goals.resize(robots_.size());
    for (size_t ridx = 0; ridx < goal_.size(); ridx++) {
        goals[ridx].x = goal_[ridx].x;
        goals[ridx].y = goal_[ridx].y;
        goals[ridx].z = goal_[ridx].z;
        goals[ridx].theta = goal_[ridx].theta;
//        printf("goals r%li X%i Y%i Z%i a%i cost:%f\n", ridx, goals[ridx].x, goals[ridx].y, goals[ridx].z, goals[ridx].theta, goal_[ridx].cost);
    }
    return goals;
}

void ExplorationPlanner::UpdateMap(CoverageMap_c newmap)
{
    coverage_ = newmap;
    coverage_.UpdateDistances();
}

void ExplorationPlanner::PartialUpdateMap(std::vector<MapElement_c> pts)
{
    for (size_t pidx = 0; pidx < pts.size(); pidx++) {
        coverage_.Setval(pts[pidx].x, pts[pidx].y, pts[pidx].z, pts[pidx].data);
    }
    coverage_.UpdateDistances();
}
