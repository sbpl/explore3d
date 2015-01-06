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

typedef double CostType;
#define MaxCost 1e99

class Locations_c;
class SearchPts_c;
class pts2d;
class Robot_c;

/// Create a grid with dimension \dimensions.size() and fill with value \val.
template <typename MapType, typename T>
void InitGrid(MapType& map, const std::vector<std::size_t>& dims, T val);

template <typename MapType, typename T>
void InitGridRecursive(MapType& map, const std::vector<std::size_t>& dims, T val, std::size_t dim);

class MapElement_c {
public:
  uint x, y, z;
  unsigned char data;
};

class Locations_c {
public:
  int x, y, z;
  int theta;
};

class pts2d {
public:
  int x, y;
  int theta;  // discretiszed angle back to center

  friend bool ptscompare(const pts2d & pt1, const pts2d & pt2);
  friend bool ptssort(const pts2d & pt1, const pts2d & pt2);
};

class SearchPts_c {
public:
  int x,y,z;
  int theta;
  CostType cost;

  friend bool operator< (const SearchPts_c & pt1, const SearchPts_c & pt2);
  friend bool operator> (const SearchPts_c & pt1, const SearchPts_c & pt2);
  friend bool EqualLocation (const SearchPts_c & pt1, const SearchPts_c & pt2);
  //friend bool comparepts (const SearchPts_c & pt1, const SearchPts_c & pt2);
};

class SPCompare {
public:
  bool operator ()(const SearchPts_c& lhs, SearchPts_c& rhs) const   {
	return lhs.cost > rhs.cost;
  }
};

/// @brief Maintains a three-dimensional obstacle map along with a two-dimensional distance transform for each robot at
///        the robot's primary motion height.
///
/// All cells within obstacles will return 0 as their nearest distance away from obstacles, cells adjacent to obstacle
/// cells will return 1.0 if they are incident at an edge, 1.41 if they are diagonal, and distances are propagated forth
/// in an 8-connected manner.
class CoverageMap_c
{
public:

    /// @brief Initialize the coverage map of size (\x * \y * \z) cells.
    /// @param x, y, z The dimensions of the three-dimensional coverage map
    /// @param fs The value of a freespace cell
    /// @param unk The value of an unknown cell
    /// @param ob The value of an obstacle cell
    /// @param RobotPtr A pointer to a sequence of robots
    bool Init(
            uint x, uint y, uint z,
            unsigned char fs,
            unsigned char unk,
            unsigned char ob,
            std::vector<Robot_c>* RobotPtr);

    /// @brief Return whether the cell (\x, \y, \z) is within the map boundaries.
    bool OnMap(int x, int y, int z)
    {
        return (x < x_size_ && y < y_size_ && z < z_size_ && x >= 0 && y >= 0 && z >= 0);
    }

    void Setval(int x, int y, int z, char val)
    {
        if (OnMap(x,y,z)) {
            map_(x, y, z) = val;
        }
    }

    unsigned char Getval(int x, int y, int z)
    {
        if (OnMap(x, y, z)) {
            return map_(x, y, z);
        }
        else {
            return 255;
        }
    }

    /// @brief Return all unknown cells that border a freespace cell.
    std::vector<SearchPts_c> GetFrontier3d();

    /// @brief Refresh the two-dimensional distance transform.
    void UpdateDistances();

    /// @brief Return whether cell (\x, \y, \z) is \robot_size cells away from obstacle cells for robot \rn.
    bool OnInflatedMap(int x, int y, int z, int rn, int robot_size)
    {
        return OnMap(x, y, z) && DistToObs_[rn](x, y) >= robot_size * 100;
    }

    /// @brief Return the distance to the nearest obstacle, in cells, from cell (\x, \y) or -1 if the cell is off the map.
    double ReturnDistToObs(int rn, int x, int y)
    {
        if (OnMap(x,y,0)) {
            return (double)DistToObs_[rn](x, y) / 100.0;
        }
        else {
            return -1.0;
        }
    }

    uint x_size_;
    uint y_size_;
    uint z_size_;

    au::Grid<3, char> map_; // (x, y, z)

private:

    std::vector<au::Grid<2, int>> DistToObs_; // 8-connected distances to obstacle cells in (cells * 100)
    unsigned char FREESPACE, OBS, UNK;
    std::vector<Robot_c>* robotsPtr_;
};

class Robot_c{
public:
  // distances in cells, angles in radians
  uint MotionHeight_;
  uint SensorHeight_;
  double HorizontalFOV_;
  double VerticalFOV_;
  uint DetectionRange_;
  uint CircularSize_;
  std::string name;
};

class ExpParams_c {
public:
  std::vector<Robot_c> robots;
  uint size_x, size_y, size_z;
  uint ObjectMaxElev;
  uint obs, freespace, unk;
  uint NumAngles;
  uint MinDist;
};

class ExplorationPlanner {
  friend class CoverageMap_c;

protected:
  std::vector<Robot_c> robots_;
private:

  void PrecalcVisibilityCircles(void);
  void Dijkstra(Locations_c start, int robotnum);
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

public:

  typedef au::Grid<2, CostType> CostMap;
  typedef au::Grid<3, CostType> CountMap;

  CoverageMap_c coverage_;
  uint ObjectMaxElev_, NumAngles_;
  unsigned char FREESPACE, OBS, UNK;
  uint MinDist_;

  std::vector<SearchPts_c> mp_;  //motion primitives
  std::vector<CostMap> CostToPts_;   //[robot](x, y)
  std::vector<SearchPts_c> goal_;  //[robot]
  std::vector<SearchPts_c> Frontier3d_;
  std::vector<CountMap> counts_;						// [robot][x[][y][angle]
  std::vector<std::vector<std::vector<pts2d> > > VisibilityRings_;		// [robot][z][points]

  void Init(ExpParams_c initparams);
  void UpdateMap(CoverageMap_c new_map);
  void PartialUpdateMap(std::vector<MapElement_c> pts);
  std::vector<Locations_c> NewGoals(std::vector<Locations_c> RobotLocations);

};


template <typename MapType, typename T>
void InitGrid(MapType& map, const std::vector<std::size_t>& dims, T val)
{
//    InitGridRecursive(map, dims, val, 0);
}

template <typename MapType, typename T>
void InitGridRecursive(MapType& map, const std::vector<std::size_t>& dims, T val, std::size_t dim)
{
//    if (dim == dims.size() - 1) {
//        map[dim].resize(dims[dim]);
//        for (std::size_t i = 0; i < map[dim].size(); ++i) {
//            map[dim][i] = val;
//        }
//    }
//    else {
//        map[dim].resize(dims[dim]);
//        InitGridRecursive(map, dims, val, dim + 1);
//    }
}
