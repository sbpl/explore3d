/// 3-D Exploration Header
/// (c) 2014 Jonathan Michael Butzke

#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <queue>
#include <cstdio>
#include <mutex>
#include <thread>

#include "Grid.h"
#include "coverage_map.hpp"
#include "exploration_structs.hpp"

class ExpParams_c
{
public:

    std::vector<Robot_c> robots;
    uint size_x, size_y, size_z;
    uint ObjectMaxElev;
    uint obs, freespace, unk;
    uint NumAngles;
    uint MinDist;
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
    std::vector<Locations_c> NewGoals(std::vector<Locations_c> RobotLocations);

protected:

    std::vector<Robot_c> robots_;

private:

    friend class CoverageMap_c;

    void PrecalcVisibilityCircles(void);
    void Dijkstra(const Locations_c& start, int robotnum);
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
};
