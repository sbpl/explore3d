/// 3-D Exploration Header
/// (c) 2014 Jonathan Michael Butzke

#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <queue>
#include <cstdio>

typedef double CostType;
#define MaxCost 1e99

class Locations_c;
class SearchPts_c;
class pts2d;
class Robot_c;

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

class CoverageMap_c {
public:
  bool Init(uint x, uint y, uint z, unsigned char fs, unsigned char unk, unsigned char ob, std::vector<Robot_c>* RobotPtr);
  bool OnMap(int x, int y, int z) {return (x< x_size_ && y<y_size_ && z<z_size_ && x>=0 && y>=0 && z>=0);};
  void Setval(int x, int y, int z, char val) {if (OnMap(x,y,z)) map_[x][y][z]=val;};
  unsigned char Getval(int x, int y, int z) {if (OnMap(x, y, z)) return map_[x][y][z]; else return 255;};
  std::vector<SearchPts_c> GetFrontier3d(void);
  void UpdateDistances(void);
  bool OnInflatedMap(int x, int y, int z, int rn, int robot_size) {return OnMap(x,y,z) && DistToObs_[rn][x][y] >= robot_size*100;};
  double ReturnDistToObs(int rn, int x, int y) {if (OnMap(x,y,0)) {return (double)DistToObs_[rn][x][y]/100.0;} };

  uint x_size_;
  uint y_size_;
  uint z_size_;
private:
  std::vector<std::vector<std::vector<char> > > map_;     //[x][y][z]
  std::vector<std::vector<std::vector<int> > > DistToObs_;   //[robot][x][y]
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
  double MinDist;
};

class ExplorationPlanner {
  friend class CoverageMap_c;

protected:
  std::vector<Robot_c> robots_;
private:
  CoverageMap_c coverage_;
  uint ObjectMaxElev_, NumAngles_;
  unsigned char FREESPACE, OBS, UNK;
  double MinDist_;

  std::vector<SearchPts_c> mp_;  //motion primitives
  std::vector<std::vector<std::vector<CostType> > > CostToPts_;   //[robot][x][y]
  std::vector<SearchPts_c> goal_;  //[robot]
  std::vector<SearchPts_c> Frontier3d_;
  std::vector<std::vector<std::vector<std::vector<int> > > > counts_;						// [robot][x[][y][angle]
  std::vector<std::vector<std::vector<pts2d> > > VisibilityRings_;		// [robot][z][points]

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
  void Init(ExpParams_c initparams);
  void UpdateMap(CoverageMap_c new_map);
  void PartialUpdateMap(std::vector<MapElement_c> pts);
  std::vector<Locations_c> NewGoals(std::vector<Locations_c> RobotLocations);
};
