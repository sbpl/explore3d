#include "coverage_map.hpp"

bool CoverageMap_c::Init(
    uint x, uint y, uint z,
    unsigned char fs,
    unsigned char unk,
    unsigned char ob,
    std::vector<Robot_c> *RobotPtr)
{
    x_size_ = x;
    y_size_ = y;
    z_size_ = z;
    FREESPACE = fs;
    OBS = ob;
    UNK = unk;
    robotsPtr_ = RobotPtr;

    DistToObs_.resize(robotsPtr_->size());
    for (std::size_t i = 0; i < robotsPtr_->size(); ++i) {
        DistToObs_[i].resize(x, y);
        DistToObs_[i].assign(0);
    }

    map_.resize(x, y, z);
    map_.assign(UNK);

    return true;
}

std::vector<SearchPts_c> CoverageMap_c::GetFrontier3d()
{
    std::vector<SearchPts_c> fpts;
    SearchPts_c temp;

    for (uint xidx = 1; xidx < x_size_ - 1; xidx++) {
        for (uint yidx = 1; yidx < y_size_ - 1; yidx++) {
            for (uint zidx = 0; zidx < z_size_; zidx++) {
                if (map_(xidx, yidx, zidx) == UNK) {
                    if ((zidx + 1 < z_size_ && map_(xidx, yidx, zidx + 1) == FREESPACE) ||
                         (zidx > 0 && map_(xidx, yidx, zidx - 1) == FREESPACE) ||
                         map_(xidx, yidx + 1, zidx) == FREESPACE ||
                         map_(xidx, yidx - 1, zidx) == FREESPACE ||
                         map_(xidx + 1, yidx, zidx) == FREESPACE ||
                         map_(xidx - 1, yidx, zidx) == FREESPACE)
                    {
                        temp.x = xidx;
                        temp.y = yidx;
                        temp.z = zidx;
                        fpts.push_back(temp);
                        // printf("(%i %i %i)", xidx, yidx, zidx);
                    }
                }
            }
        }
    }

    return fpts;
}

void CoverageMap_c::UpdateDistances()
{
    for (size_t ridx = 0; ridx < DistToObs_.size(); ridx++) {
        uint zz = (*robotsPtr_)[ridx].MotionHeight_;

        for (uint xidx = 0; xidx < x_size_; xidx++) {
            for (uint yidx = 0; yidx < y_size_; yidx++) {
                if (xidx == 0 || yidx == 0 || map_(xidx, yidx, zz) == OBS) {
                    DistToObs_[ridx](xidx, yidx) = 0;
                }
                else {
                    int temp1 = std::min(DistToObs_[ridx](xidx - 1, yidx) + 100, DistToObs_[ridx](xidx, yidx - 1) + 100);
                    int temp2 = std::min(DistToObs_[ridx](xidx - 1, yidx - 1) + 141, DistToObs_[ridx](xidx - 1, yidx + 1) + 141);
                    DistToObs_[ridx](xidx, yidx) = std::min(temp1, temp2);
                }
            }
        }

        for (int xidx = x_size_ - 1; xidx >= 0; xidx--) {
            for (int yidx = y_size_ - 1; yidx >= 0; yidx--) {
                if (xidx == x_size_ - 1 || yidx == y_size_ - 1 || map_(xidx, yidx, zz) == OBS) {
                    DistToObs_[ridx](xidx, yidx) = 0;
                }
                else {
                    int temp1 = std::min(DistToObs_[ridx](xidx + 1, yidx) + 100, DistToObs_[ridx](xidx, yidx + 1) + 100);
                    int temp2 = std::min(DistToObs_[ridx](xidx + 1, yidx + 1) + 141, DistToObs_[ridx](xidx + 1, yidx - 1) + 141);
                    int temp3 = std::min(temp1, temp2);
                    DistToObs_[ridx](xidx, yidx) = std::min(temp3, DistToObs_[ridx](xidx, yidx));
                }
            }
        }
    }
}
