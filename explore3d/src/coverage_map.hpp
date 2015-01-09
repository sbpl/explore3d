#ifndef CoverageMap_hpp
#define CoverageMap_hpp

#include <vector>

#include "Grid.h"
#include "exploration_structs.hpp"

/// @brief Maintains a three-dimensional obstacle map along with a two-dimensional distance transform for each robot at
///        the robot's primary motion height.
///
/// All cells within obstacles will return 0 as their nearest distance away from obstacles, cells adjacent to obstacle
/// cells will return 1.0 if they are incident at an edge, 1.41 if they are diagonal, and distances are propagated forth
/// in an 8-connected manner.
class CoverageMap_c
{
public:

    typedef au::Grid<3, char> Map;

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
    bool OnMap(int x, int y, int z) const
    {
        return (x < (int)x_size_ && y < (int)y_size_ && z < (int)z_size_ && x >= 0 && y >= 0 && z >= 0);
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
    double ReturnDistToObs(int rn, int x, int y) const
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

#endif
