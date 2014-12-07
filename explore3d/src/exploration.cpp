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



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Minor Classes
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool CoverageMap_c::Init(uint x, uint y, uint z, unsigned char fs, unsigned char unk, unsigned char ob, std::vector<Robot_c> *RobotPtr)  {
  x_size_=x; y_size_=y; z_size_=z;
  FREESPACE = fs;
  OBS = ob;
  UNK = unk;
  robotsPtr_ = RobotPtr;

  DistToObs_.resize(robotsPtr_->size() );
  for (size_t ridx=0; ridx < DistToObs_.size(); ridx++) {
	DistToObs_[ridx].resize(x);
	for (uint xidx=0; xidx < x; xidx++) {
	  DistToObs_[ridx][xidx].resize(y);
	}
  }

  map_.resize(x);
  for (uint xidx=0; xidx < x; xidx++) {
	map_[xidx].resize(y);
	for (uint yidx=0; yidx< y; yidx++) {
	  map_[xidx][yidx].resize(z);
	  for (uint zidx=0; zidx< z; zidx++) {
		map_[xidx][yidx][zidx] = UNK;
	  }
	}
  }
  return true;
}

std::vector<SearchPts_c> CoverageMap_c::GetFrontier3d(void) {
  std::vector<SearchPts_c> fpts;
  SearchPts_c temp;

  for (uint xidx=1; xidx < x_size_-1; xidx++) {
	for (uint yidx=1; yidx < y_size_-1; yidx++) {
	  for (uint zidx=0; zidx < z_size_; zidx++) {
		if (map_[xidx][yidx][zidx] == UNK) {
		  if ((zidx+1 < z_size_ && map_[xidx][yidx][zidx+1] == FREESPACE) || (zidx>0 && map_[xidx][yidx][zidx-1] == FREESPACE) || map_[xidx][yidx+1][zidx] == FREESPACE || map_[xidx][yidx-1][zidx] == FREESPACE || map_[xidx+1][yidx][zidx] == FREESPACE || map_[xidx-1][yidx][zidx] == FREESPACE) {
			temp.x = xidx; temp.y = yidx; temp.z = zidx;
			fpts.push_back(temp);
			//	printf("(%i %i %i)", xidx, yidx, zidx);
		  }
		}
	  }
	}
  }

  return fpts;
}

bool operator< (const SearchPts_c & pt1, const SearchPts_c & pt2)
{ return pt1.cost < pt2.cost; }

bool operator> (const SearchPts_c & pt1, const SearchPts_c & pt2)
{ return pt1.cost > pt2.cost; }

bool EqualLocation (const SearchPts_c & pt1, const SearchPts_c & pt2)
{ return (pt1.x==pt2.x && pt1.y==pt2.y && pt1.z==pt2.z); }



//bool comparepts (const SearchPts_c & pt1, const SearchPts_c & pt2)
//{ return pt1.cost < pt2.cost; }


bool ptscompare(const  pts2d & pt1, const  pts2d & pt2) {return (pt1.x==pt2.x && pt1.y==pt2.y && pt1.theta==pt2.theta);}

bool ptssort(const  pts2d & pt1, const  pts2d & pt2) {
  if (pt1.x < pt2.x)
	return true;
  else if (pt1.x > pt2.x)
	return false;
  else if (pt1.y < pt2.y)
	return true;
  else
	return false;
}

void CoverageMap_c::UpdateDistances(void) {
  for (size_t ridx=0; ridx < DistToObs_.size(); ridx++) {
	uint zz = (*robotsPtr_)[ridx].MotionHeight_;

	for (uint xidx=0; xidx < x_size_; xidx++) {
	  for (uint yidx=0; yidx < y_size_; yidx++) {
		if (xidx==0 || yidx==0 || map_[xidx][yidx][zz] == OBS) {
		  DistToObs_[ridx][xidx][yidx] = 0;
		} else {
		  int temp1 = std::min(DistToObs_[ridx][xidx-1][yidx]+100, DistToObs_[ridx][xidx][yidx-1]+100);
		  int temp2 = std::min(DistToObs_[ridx][xidx-1][yidx-1]+141, DistToObs_[ridx][xidx-1][yidx+1]+141);
		  DistToObs_[ridx][xidx][yidx] = std::min(temp1, temp2);
		}
	  }
	}

	for (int xidx=x_size_-1; xidx >=0; xidx--) {
	  for (int yidx=y_size_-1; yidx >= 0; yidx--) {
		if (xidx==x_size_-1 || yidx==y_size_-1 || map_[xidx][yidx][zz] == OBS) {
		  DistToObs_[ridx][xidx][yidx] = 0;
		} else {
		  int temp1 = std::min(DistToObs_[ridx][xidx+1][yidx]+100, DistToObs_[ridx][xidx][yidx+1]+100);
		  int temp2 = std::min(DistToObs_[ridx][xidx+1][yidx+1]+141, DistToObs_[ridx][xidx+1][yidx-1]+141);
		  int temp3 = std::min(temp1, temp2);
		  DistToObs_[ridx][xidx][yidx] = std::min(temp3, DistToObs_[ridx][xidx][yidx]);
		}
	  }
	}

	//   for (uint yidx=150; yidx >98; yidx--) {
	  //   for (uint xidx=99; xidx < 150; xidx++) {
		// 	printf("%4i ",DistToObs_[ridx][xidx][yidx]);
		//   }
		//   printf("\n");
		//   }
		//   printf("\n\n");
		   }
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //  Initialization  Fxns
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void ExplorationPlanner::Init(ExpParams_c initparams) {
	FREESPACE = initparams.freespace;
	UNK = initparams.unk;
	OBS = initparams.obs;
	ObjectMaxElev_ = initparams.ObjectMaxElev;
	NumAngles_ = initparams.NumAngles;
	MinDist_ = initparams.MinDist;

	robots_ = initparams.robots;
	coverage_.Init(initparams.size_x, initparams.size_y, initparams.size_z, FREESPACE, UNK, OBS, &robots_);

	CostToPts_.resize(robots_.size());
	counts_.resize(robots_.size());
	goal_.resize(robots_.size());
	for (size_t ridx=0; ridx < CostToPts_.size(); ridx++) {
	  CostToPts_[ridx].resize(coverage_.x_size_);
	  counts_[ridx].resize(coverage_.x_size_);
	  for(size_t xidx=0; xidx < coverage_.x_size_; xidx++) {
		CostToPts_[ridx][xidx].resize(coverage_.y_size_);
		counts_[ridx][xidx].resize(coverage_.y_size_);
		for(uint yidx=0; yidx < coverage_.y_size_; yidx++) {
		  counts_[ridx][xidx][yidx].resize(NumAngles_);
		}
	  }
	}

	GenMotionSteps();
	GenVisibilityRing();
  }

  void ExplorationPlanner::GenMotionSteps(void) {
	mp_.clear();
	SearchPts_c temp;
	for (int dx=-1; dx <= 1; dx++) {
	  for(int dy=-1; dy <= 1; dy++) {
		if (dx!=0 || dy!=0 ) {
		  temp.x = dx;
		  temp.y = dy;
		  temp.z = 0;
		  temp.cost = sqrt(dx*dx+dy*dy);
		  mp_.push_back(temp);
		}
	  }
	}
  }


  void ExplorationPlanner::GenVisibilityRing(void) {
	VisibilityRings_.resize(robots_.size() );
	for (size_t ridx=0; ridx < VisibilityRings_.size(); ridx++) {
	  VisibilityRings_[ridx].resize(coverage_.z_size_);
	  for (size_t zidx=0; zidx < VisibilityRings_[ridx].size(); zidx++) {
		double height, radius;
		height = fabs((double)robots_[ridx].SensorHeight_ - (double) zidx);
		radius = height/tan(robots_[ridx].VerticalFOV_/2);
		if (radius < 2) {
		  radius = 2;
		}
		//  printf("robot %i height=%f radius = %f\n", ridx, height, radius);
		if (radius < robots_[ridx].DetectionRange_) {
		  pts2d temp_pt;
		  for (double aidx=0; aidx < 2*M_PI; aidx+=0.01) {
			temp_pt.x = radius * sin(aidx) + 0.5;
			temp_pt.y = radius * cos(aidx) + 0.5;
			double tangle = atan2(temp_pt.y,temp_pt.x);
			if (tangle < 0) {
			  tangle += 2*M_PI;
			}
			temp_pt.theta = (int)(tangle*(double)NumAngles_/(2*M_PI));
			//	  printf("r%i z%i x%i y%i a%i\n", ridx, zidx, temp_pt.x, temp_pt.y, temp_pt.theta);
			VisibilityRings_[ridx][zidx].push_back(temp_pt);
		  }
		  std::sort(VisibilityRings_[ridx][zidx].begin(), VisibilityRings_[ridx][zidx].end(), ptssort);
		  std::vector<pts2d>::iterator it;
		  it = std::unique(VisibilityRings_[ridx][zidx].begin(), VisibilityRings_[ridx][zidx].end(), ptscompare);
		  VisibilityRings_[ridx][zidx].resize(std::distance(VisibilityRings_[ridx][zidx].begin(), it) );
		}
	  }
	}
  }

  void ExplorationPlanner::ClearCounts(void) {
	for (size_t ridx=0; ridx < robots_.size(); ridx++) {
	  for (uint xidx=0; xidx < coverage_.x_size_; xidx++) {
		for (uint yidx=0; yidx < coverage_.y_size_; yidx++) {
		  for (uint aidx=0; aidx < NumAngles_; aidx++) {
			counts_[ridx][xidx][yidx][aidx] = 0;
		  }
		}
	  }
	}
  }

  void ExplorationPlanner::printMap(int h) {
	for (int yidx= coverage_.y_size_-1; yidx >=0; yidx--) {
	  printf("%4i ", yidx);
	  for (uint xidx=0; xidx < coverage_.x_size_; xidx++) {
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

  void ExplorationPlanner::printCosts(uint x0, uint y0, uint x1, uint y1, uint rn) {
	for (uint yidx=y1-1; yidx >=y0; yidx--) {
	  printf("%4i ", yidx);
	  for (uint xidx=x0; xidx < x1; xidx++) {
		printf("%4.0f ", CostToPts_[rn][xidx][yidx]);
	  }
	  printf("\n");
	}
  }

  void ExplorationPlanner::printCounts(uint x0, uint y0, uint x1, uint y1, uint rn) {
	for (uint yidx=y1-1; yidx >=y0; yidx--) {
	  printf("%4i ", yidx);
	  for (uint xidx=x0; xidx < x1; xidx++) {
		int c=0;
		for (uint aidx=0; aidx < NumAngles_; aidx++) {
		  c += counts_[rn][xidx][yidx][aidx];
		}
		printf("%4i ", c);
	  }
	  printf("\n");
	}
  }


  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //  Minor private Fxns
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  void ExplorationPlanner::Dijkstra(Locations_c start, int robotnum) {
	std::priority_queue<SearchPts_c, std::vector<SearchPts_c>, SPCompare > OPEN;
	std::vector<std::vector<bool> > CLOSED;
	SearchPts_c current, temp;
	//printf("Dijkstra\n");
	CLOSED.resize(coverage_.x_size_);
	for (uint xidx=0; xidx < coverage_.x_size_; xidx++) {
	  CLOSED[xidx].resize(coverage_.y_size_);
	  for (uint yidx=0; yidx < coverage_.y_size_; yidx++) {
		CLOSED[xidx][yidx] = false;
	  }
	}

	//for (size_t ridx=0; ridx < robots_.size(); ridx++) {
	  for (uint xidx=0; xidx < coverage_.x_size_; xidx++) {
		for (uint yidx=0; yidx < coverage_.y_size_; yidx++) {
		  CostToPts_[robotnum][xidx][yidx] = MaxCost;
		}
	  }
	  // }

	  int robot_size = robots_[robotnum].CircularSize_;
	  printf("%i size is %i\n", robotnum, robot_size);

	  temp.x = start.x;
	  temp.y = start.y;
	  temp.z = start.z;
	  temp.theta = start.theta;
	  temp.cost = 10;
	  OPEN.push(temp);

	  while (!OPEN.empty() ) {
		current = OPEN.top();
		OPEN.pop();

		if (!CLOSED[current.x][current.y]) {
		  CLOSED[current.x][current.y] = true;
		  //printf("(%i %i)", current.x, current.y);
		  CostToPts_[robotnum][current.x][current.y] = current.cost;

		  for(size_t midx=0; midx < mp_.size(); midx++) {
			temp.x = current.x + mp_[midx].x;
			temp.y = current.y + mp_[midx].y;
			temp.z = current.z;
			temp.theta = 0;
			temp.cost = current.cost + mp_[midx].cost;

			if (coverage_.OnInflatedMap(temp.x, temp.y, temp.z, robotnum, robot_size ) && coverage_.Getval(temp.x, temp.y, temp.z) == FREESPACE) {
			  OPEN.push(temp);
			}
		  }
		}
	  }
	  //printf("\n");
  }

  CostType ExplorationPlanner::EvalFxn(uint x, uint y, uint z, uint a, uint rn) {
	double dist = 1e99;
	for (size_t ridx=0; ridx < goal_.size(); ridx++) {
	  if (ridx != rn) {
		double temp_dist = (x-goal_[ridx].x)*(x-goal_[ridx].x) + (y-goal_[ridx].y)*(y-goal_[ridx].y) + (z-goal_[ridx].z)*(z-goal_[ridx].z);
		if (temp_dist < dist) {
		  dist = temp_dist;
		}
	  }
	}

	if (dist < MinDist_*MinDist_) {
	  dist /= (MinDist_*MinDist_);
	} else {
	  dist = 1;
	}

	CostType rtnval = counts_[rn][x][y][a]/CostToPts_[rn][x][y] * dist;
	return rtnval;
  }

  void ExplorationPlanner::CreateFrontier(void) {
	Frontier3d_ = coverage_.GetFrontier3d();
	ClearCounts();

	for (size_t ridx=0; ridx < robots_.size(); ridx++) {
	  // for each frontier cell, determine viewing cells by raycasting outwards to intersect robot sensor planes
	  for (size_t pidx=0; pidx < Frontier3d_.size(); pidx++) {
		raycast3d(Frontier3d_[pidx], ridx);
	  }

	  // 	printf("Coverage for %li\n", ridx);
	  // 	printCosts(99, 99, 150, 150, ridx);
	  //
	  // 	printf("Counts for %li\n", ridx);
	  // 	printCounts(99, 99, 150, 150, ridx);

	  goal_[ridx].cost = 0;
	  goal_[ridx].z = robots_[ridx].MotionHeight_;
	  //printf("z is %i\n", goal_[ridx].z);
	  for (size_t xidx=0; xidx< coverage_.x_size_; xidx++) {
		for (size_t yidx=0; yidx< coverage_.y_size_; yidx++) {
		  for (uint aidx=0; aidx< NumAngles_; aidx++) {
			// 		  if (xidx==150 && yidx==150) {
			  // 			printf("%li angle:%i cost:%f  counts:%i ratio:%f goal:%f\n", ridx, aidx, CostToPts_[ridx][xidx][yidx], counts_[ridx][xidx][yidx][aidx], counts_[ridx][xidx][yidx][aidx]/CostToPts_[ridx][xidx][yidx], goal_[ridx].cost);
			  // 		  }
			  if (CostToPts_[ridx][xidx][yidx] !=MaxCost && EvalFxn(xidx, yidx, goal_[ridx].z, aidx, ridx)  > goal_[ridx].cost) {
				goal_[ridx].cost = counts_[ridx][xidx][yidx][aidx]/CostToPts_[ridx][xidx][yidx];
				goal_[ridx].x = xidx;
				goal_[ridx].y = yidx;
				goal_[ridx].theta = aidx;
				//	printf("new goal: r%li x%li y%li z%i a%i count:%i cost:%f total:%f  obs:%f\n", ridx, xidx, yidx, goal_[ridx].z, aidx,counts_[ridx][xidx][yidx][aidx], CostToPts_[ridx][xidx][yidx], goal_[ridx].cost, coverage_.ReturnDistToObs(ridx, xidx, yidx) );
			  }
		}
	  }
	}
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  public  Fxns
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<Locations_c> ExplorationPlanner::NewGoals(std::vector<Locations_c> RobotLocations) {
  for (size_t ridx=0; ridx<RobotLocations.size(); ridx++) {
	printf("poses r%li X%i Y%i Z%i a%i \n", ridx, RobotLocations[ridx].x, RobotLocations[ridx].y, RobotLocations[ridx].z, RobotLocations[ridx].theta);
	Dijkstra(RobotLocations[ridx], ridx);
	//printMap(RobotLocations[ridx].z);

  }
  CreateFrontier();
  // for (uint ridx =0; ridx < robots_.size(); ridx++) {
	//   printf("Robot %i\n", ridx);
  // for (uint pidx=0; pidx < VisibilityRings_[ridx][robots_[ridx].SensorHeight_].size(); pidx++) {
	//   printf("(%i %i)", VisibilityRings_[ridx][robots_[ridx].SensorHeight_][pidx].x, VisibilityRings_[ridx][robots_[ridx].SensorHeight_][pidx].y);
  // }
  // printf("\n");
  //
  // }




  std::vector<Locations_c> goals;
  goals.resize(robots_.size());
  for (size_t ridx=0; ridx< goal_.size(); ridx++) {
	goals[ridx].x=goal_[ridx].x;
	goals[ridx].y=goal_[ridx].y;
	goals[ridx].z=goal_[ridx].z;
	goals[ridx].theta = goal_[ridx].theta;
	printf("goals r%li X%i Y%i Z%i a%i cost:%f\n", ridx, goals[ridx].x, goals[ridx].y, goals[ridx].z, goals[ridx].theta, goal_[ridx].cost);
  }
  return goals;
}


void ExplorationPlanner::UpdateMap(CoverageMap_c newmap) {
  coverage_ = newmap;
  coverage_.UpdateDistances();
}

void ExplorationPlanner::PartialUpdateMap(std::vector<MapElement_c> pts) {
  for (size_t pidx=0; pidx < pts.size(); pidx++) {
	coverage_.Setval(pts[pidx].x, pts[pidx].y, pts[pidx].z, pts[pidx].data);
  }
  coverage_.UpdateDistances();
}
