/// 3-D Exploration Header
/// (c) 2014 Jonathan Michael Butzke

#ifndef ExplorationPlanner_h
#define ExplorationPlanner_h

#include <vector>
#include <string>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <queue>
#include <cstdio>
#include <mutex>
#include <thread>

#include <sbpl/headers.h>

#include <explore3d/Grid.h>
#include <explore3d/coverage_map.hpp>
#include <explore3d/exploration_structs.hpp>

class ExpParams_c
{
public:

    std::vector<Robot_c> robots;
    uint size_x, size_y, size_z;
    uint ObjectMaxElev;
    uint obs, freespace, unk;
    uint NumAngles;
    uint MinDist;

    /// @name Cost Function Parameters
    /// @{
    double backwards_penalty;
    /// @}

    int room_min_x;
    int room_min_y;
    int room_max_x;
    int room_max_y;
};

class ExplorationPlanner
{
public:

    typedef CoverageMap_c::Map Map;
    typedef au::Grid<2, CostType> CostMap;
    typedef au::Grid<3, CostType> CountMap;
    typedef au::Grid<3, CostType> ScoreMap;

    CoverageMap_c coverage_;
    uint ObjectMaxElev_, NumAngles_;
    unsigned char FREESPACE, OBS, UNK;
    uint MinDist_;
    double backwards_penalty_;
    int room_min_x_;
    int room_min_y_;
    int room_max_x_;
    int room_max_y_;

    std::vector<SearchPts_c> mp_;                                       //motion primitives
    std::vector<CostMap> CostToPts_;                                    //[robot](x, y)
    std::vector<SearchPts_c> goal_;                                     //[robot]
    std::vector<SearchPts_c> Frontier3d_;
    std::vector<CountMap> counts_;                                      // [robot][x][y][angle]
    std::vector<ScoreMap> scores_;                                      // [robot](x, y, yaw)
    std::vector<std::vector<std::vector<pts2d> > > VisibilityRings_;    // [robot][z][points]

    ExplorationPlanner();
    ~ExplorationPlanner();

    void Init(ExpParams_c initparams);

    void UpdateMap(CoverageMap_c new_map);
    void PartialUpdateMap(const std::vector<MapElement_c>& pts);

    /// \brief Compute a new exploration goal for a single robot
    ///
    /// \param robot_pose Copy required since the height of the robot can be
    ///     modified to the assumed nominal height
    bool NewGoal(size_t ridx, std::vector<Locations_c> robot_poses, Locations_c& goal);

    /// \brief Compute new exploration goals for both robots.
    ///
    /// The exploration goals are returned in a vector whose elements correspond
    /// to the robot with the associated index. Failure to compute new goals is
    /// indicated by a returned empty vector.
    ///
    /// \param robot_poses Copy required since the height of the robot can be
    ///     modified to the assumed nominal height
    std::vector<Locations_c> NewGoals(std::vector<Locations_c> robot_poses);

    double EstimatedCompletionPercent() const;

private:

    std::vector<Robot_c> robots_;

    friend class CoverageMap_c;

    // Adapt SearchPts_c for use with CHeap
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

    std::atomic<double> completion_pct_;

    void PrecalcVisibilityCircles(void);

    // Fill CostToPts_[robotnum] with the shortest path cost to all cells.
    //
    // This function uses the projected coverage map corresponding to the input
    // robot index to compute the shortest path costs to all cells from the
    // input start state. The path costs are stored in CostToPts_[robotnum].
    //
    // This function does its best to be robust to starts-in-collision by first
    // finding the nearest cell to the start state that is collision-free. In
    // the nominal case this is just the start cell. If the
    // escape-from-collision search is unsuccessful (there are no cells in the
    // projected coverage map that are valid), then this function will return
    // false. All other situations should return true.
    //
    // param start The starting cell
    // param robotnum The index of the robot to compute shortest paths for
    bool ComputeTraversalCosts(const Locations_c& start, int robotnum);

    bool FindNearestCollisionFreeCell(
        const Locations_c& start,
        int robotnum,
        Locations_c& out) const;

    // Update the frontier and compute the information gain for a given robot.
    bool ComputeInformationGain(int ridx);

    // Compute the information gain for a given robot and frontier.
    //
    // This variant is useful to avoid computing the frontier multiple times
    // when computing multiple goals simultaneously.
    bool ComputeInformationGain(
        int ridx,
        const std::vector<SearchPts_c>& frontier);

    // Select the next exploration goal for the given robot.
    //
    // This function takes into account the current state of the robot's
    // costmap, the robot's score map, and the most recent goals for the other
    // robots.
    void SelectGoal(size_t ridx);

    void GenMotionSteps();
    void GenVisibilityRing();
    void ClearCounts();
    void ClearInformationGain(int ridx);
    bool bresenham_line_3D(int x1, int y1, int z1, int x2, int y2, int z2);
    void printMap(int height);
    void printCosts(uint x0, uint y0, uint x1, uint y1, uint rn);
    void printCounts(uint x0, uint y0, uint x1, uint y1, uint rn);
    void LogData(std::vector<Locations_c> robot_poses);
    FILE* datafile_;
    bool data_file_open;

    // Compute the score for a given cell based on the values in CostToPts_,
    // counts_ at the given robot index, and the values of goal_ at all other
    // robot indices.
    CostType EvalFxn(uint x, uint y, uint z, uint a, uint rn) const;

    // convert a floating-point cost value to a fixed point value for use with
    // CHeap
    CKey CreateKey(double val) const;
    CostType ComputeMotionPenalty(
        const Locations_c& start,
        const SearchPtState& s,
        const SearchPtState& t);

    // Fill counts_[robotnum] with the information gain for each cell.
    void raycast3d(const SearchPts_c& start, int robotnum);

    // Fill counts_[hexanum] with the information gain for each cell.
    //
    // This variant reasons that no information gain is available if the given
    // cell is visible to another robot.
    void raycast3d_hexa(const SearchPts_c& start, int hexanum);

    bool inside_room(int x, int y) const;
};

#endif
