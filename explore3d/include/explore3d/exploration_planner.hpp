/// 3-D Exploration Header
/// (c) 2014 Jonathan Michael Butzke

#ifndef ExplorationPlanner_h
#define ExplorationPlanner_h

#include <vector>
#include <string>
#include <algorithm>
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

    std::vector<SearchPts_c> mp_;                                       //motion primitives
    std::vector<CostMap> CostToPts_;                                    //[robot](x, y)
    std::vector<SearchPts_c> goal_;                                     //[robot]
    std::vector<SearchPts_c> Frontier3d_;
    std::vector<CountMap> counts_;                                      // [robot][x][y][angle]
    std::vector<ScoreMap> scores_;                                      // [robot](x, y, yaw)
    std::vector<std::vector<std::vector<pts2d> > > VisibilityRings_;    // [robot][z][points]

    void Init(ExpParams_c initparams);
    
    void UpdateMap(CoverageMap_c new_map);
    void PartialUpdateMap(std::vector<MapElement_c> pts);

    Locations_c NewGoal(std::size_t ridx, std::vector<Locations_c>& RobotLocations);
    std::vector<Locations_c> NewGoals(std::vector<Locations_c> RobotLocations);

protected:

    std::vector<Robot_c> robots_;

private:

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

    void PrecalcVisibilityCircles(void);
    bool Dijkstra(const Locations_c& start, int robotnum);
    bool FindNearestCollisionFreeCell(const Locations_c& start, int robotnum, Locations_c& out);
    void CreateFrontier(void);
    void GenMotionSteps(void);
    void GenVisibilityRing(void);
    void ClearCounts(void);
    void raycast3d(SearchPts_c start, int robotnum);
    bool bresenham_line_3D(int x1, int y1, int z1, int x2, int y2, int z2);
    void printMap(int height);
    void printCosts(uint x0, uint y0, uint x1, uint y1, uint rn);
    void printCounts(uint x0, uint y0, uint x1, uint y1, uint rn);
    CostType EvalFxn(uint x, uint y, uint z, uint a, uint rn);

    // convert a floating-point cost value to a fixed point value for use with CHeap
    CKey CreateKey(double val);
    CostType ComputeMotionPenalty(const Locations_c& start, const SearchPtState& s, const SearchPtState& t);
};

#endif
