#ifndef exploration_classes_hpp
#define exploration_classes_hpp

#include <cstdlib>
#include <string>

#define MaxCost 1e99

typedef double CostType;

class Locations_c;
class SearchPts_c;
class pts2d;
class Robot_c;

bool operator<(const SearchPts_c & pt1, const SearchPts_c & pt2);
bool operator>(const SearchPts_c & pt1, const SearchPts_c & pt2);
bool EqualLocation(const SearchPts_c & pt1, const SearchPts_c & pt2);
bool ptscompare(const  pts2d & pt1, const  pts2d & pt2);
bool ptssort(const pts2d & pt1, const pts2d & pt2);

class MapElement_c
{
public:
  uint x, y, z;
  unsigned char data;
};

class Pose3d
{
public:
  
    double x, y, z, theta;
};

class Locations_c
{
public:
  int x, y, z;
  int theta;
};

class pts2d
{
public:
  int x, y;
  int theta;  // discretiszed angle back to center

  friend bool ptscompare(const pts2d & pt1, const pts2d & pt2);
  friend bool ptssort(const pts2d & pt1, const pts2d & pt2);
};

class SearchPts_c
{
public:
  int x,y,z;
  int theta;
  CostType cost;

  friend bool operator< (const SearchPts_c & pt1, const SearchPts_c & pt2);
  friend bool operator> (const SearchPts_c & pt1, const SearchPts_c & pt2);
  friend bool EqualLocation (const SearchPts_c & pt1, const SearchPts_c & pt2);
  //friend bool comparepts (const SearchPts_c & pt1, const SearchPts_c & pt2);
};

class SPCompare
{
public:
    bool operator ()(const SearchPts_c& lhs, SearchPts_c& rhs) const
    {
        return lhs.cost > rhs.cost;
    }
};

class Robot_c
{
public:

  // distances in cells, angles in radians
  uint MotionHeight_;
  uint MotionLevelBottom_;
  uint MotionLevelTop_;

  uint SensorHeight_;
  double HorizontalFOV_;
  double VerticalFOV_;
  uint DetectionRange_;
  uint CircularSize_;
  std::string name;
};

std::string to_string(const Locations_c& l);
std::string to_string(const SearchPts_c& p);

#endif
