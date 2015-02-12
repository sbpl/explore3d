#include <explore3d/exploration_structs.hpp>

#include <sstream>
#include <tuple>

bool operator<(const SearchPts_c& pt1, const SearchPts_c& pt2)
{
    return pt1.cost < pt2.cost;
}

bool operator>(const SearchPts_c & pt1, const SearchPts_c & pt2)
{
    return pt1.cost > pt2.cost;
}

bool operator==(const Locations_c& s, const  Locations_c& t)
{
    return std::tie(s.x, s.y, s.z, s.theta) == std::tie(t.x, t.y, t.z, t.theta);
}

bool operator!=(const Locations_c& s, const Locations_c& t)
{
    return !operator==(s, t);
}

bool EqualLocation (const SearchPts_c & pt1, const SearchPts_c & pt2)
{ return (pt1.x==pt2.x && pt1.y==pt2.y && pt1.z==pt2.z); }

bool ptscompare(const  pts2d & pt1, const  pts2d & pt2)
{
    return (pt1.x==pt2.x && pt1.y==pt2.y && pt1.theta==pt2.theta);
}

bool ptssort(const pts2d & pt1, const pts2d & pt2)
{
    if (pt1.x < pt2.x)
        return true;
    else if (pt1.x > pt2.x)
        return false;
    else if (pt1.y < pt2.y)
        return true;
    else
        return false;
}

std::string to_string(const Locations_c& l)
{
    std::stringstream ss;
    ss << "{ X" << l.x << " Y" << l.y << " Z" << l.z << " a" << l.theta << " }";
    return ss.str();
}

std::string to_string(const SearchPts_c& p)
{
    std::stringstream ss;
    ss << "( " << p.x << ", " << p.y << ", " << p.z << ", " << p.theta << ", " << p.cost << " )";
    return ss.str();
}
