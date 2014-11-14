///Jonathan Michael Butzke
/// (c) 2014
// g++ -o testEP exploration.cpp raycast.cpp testmain.cpp  -std=c++0x -g -pedantic -Wall



#include "exploration.hpp"

int main(void) {
  double scale = 20;  //cells per meter


  Robot_c Segway;
  Robot_c Hexa;
  Locations_c SegLoc;
  Locations_c HexLoc;

  ExpParams_c params;
  CoverageMap_c map;


  Segway.MotionHeight_ = 0*scale;
  Segway.SensorHeight_ = 0.2*scale;
  Segway.HorizontalFOV_ = M_PI/8;
  Segway.VerticalFOV_ = M_PI/10;
  Segway.DetectionRange_ = 5*scale;
  Segway.CircularSize_ = 1.0*scale;
  Segway.name = "Segway";

  Hexa.MotionHeight_ = 1.2*scale;
  Hexa.SensorHeight_ = 1.2*scale;
  Hexa.HorizontalFOV_ = M_PI/8;
  Hexa.VerticalFOV_ = M_PI/10;
  Hexa.DetectionRange_ = 5*scale;
  Hexa.CircularSize_ = 0.5*scale;
  Hexa.name = "Hexa";

  SegLoc.x = 150;
  SegLoc.y = 150;
  SegLoc.z = 0;
  SegLoc.theta = 0;
  HexLoc.x = 350;
  HexLoc.y = 350;
  HexLoc.z = 25;
  HexLoc.theta = 0;

  params.robots.push_back(Segway);
  params.robots.push_back(Hexa);
  std::vector<Locations_c> robot_loc;
  robot_loc.push_back(SegLoc);
  robot_loc.push_back(HexLoc);


  params.size_x = params.size_y = 30*scale;
  params.size_z = 5*scale;
  params.ObjectMaxElev = 15;
  params.obs = 100;
  params.freespace = 50;
  params.unk = 0;
  params.NumAngles = 16;
  params.MinDist = 100;

  //map.Init(params.size_x, params.size_y, params.size_z, params.freespace, params.unk, params.obs, params.robot);

  std::vector<MapElement_c> pts;

  for (uint xidx=0; xidx < 500; xidx++) {
	for (uint yidx=0; yidx < 500; yidx++) {
	  for (uint zidx=0; zidx < 50; zidx++) {
		MapElement_c pt;
		pt.x = xidx;
		pt.y = yidx;
		pt.z = zidx;
		pt.data = params.obs;
		pts.push_back(pt);
	//	map.Setval(xidx, yidx, zidx, params.unk);
	  }
	}
  }

  for (uint xidx=101; xidx < 399; xidx++) {
	for (uint yidx=101; yidx < 399; yidx++) {
	  for (uint zidx=0; zidx < 50; zidx++) {
		MapElement_c pt;
		pt.x = xidx;
		pt.y = yidx;
		pt.z = zidx;
		pt.data = params.unk;
		pts.push_back(pt);
		//	map.Setval(xidx, yidx, zidx, params.unk);
	  }
	}
  }



  for (uint xidx=102; xidx < 398; xidx++) {
	for (uint yidx=102; yidx < 398; yidx++) {
	  for (uint zidx=0; zidx < 50; zidx++) {
		MapElement_c pt;
		pt.x = xidx;
		pt.y = yidx;
		pt.z = zidx;
		pt.data = params.freespace;
		pts.push_back(pt);
	//	map.Setval(xidx, yidx, zidx, params.freespace);
	  }
	}
  }

  for (uint xidx=130; xidx < 135; xidx++) {
	for (uint yidx=130; yidx < 135; yidx++) {
	  for (uint zidx=0; zidx < 50; zidx++) {
		MapElement_c pt;
		pt.x = xidx;
		pt.y = yidx;
		pt.z = zidx;
		pt.data = params.obs;
		pts.push_back(pt);
		//	map.Setval(xidx, yidx, zidx, params.unk);
	  }
	}
  }




//   for (int i=0; i < 100000; i++) {
// 	MapElement_c pt;
// 	pt.x = rand() % params.size_x;
// 	pt.y = rand() % params.size_y;
// 	pt.z = rand() % params.size_z;
// 	pt.data = params.unk;
// 	pts.push_back(pt);
// 	//map.Setval(pt.x, pt.y, pt.z, params.unk);
// 	printf("%i %i %i\n", pt.x, pt.y, pt.z);
//   }

//   for (uint xidx=240; xidx < 245; xidx++) {
// 	for (uint yidx=150; yidx < 300; yidx++) {
// 	  for (uint zidx=0; zidx < 40; zidx++) {
// 		MapElement_c pt;
// 		pt.x = xidx;
// 		pt.y = yidx;
// 		pt.z = zidx;
// 		pt.data = params.obs;
// 		pts.push_back(pt);
// 		map.Setval(xidx, yidx, zidx, params.obs);
// 	  }
// 	}
//   }
//
//   for (uint xidx=260; xidx < 265; xidx++) {
// 	for (uint yidx=150; yidx < 300; yidx++) {
// 	  for (uint zidx=0; zidx < 40; zidx++) {
// 		MapElement_c pt;
// 		pt.x = xidx;
// 		pt.y = yidx;
// 		pt.z = zidx;
// 		pt.data = params.obs;
// 		pts.push_back(pt);
// 		map.Setval(xidx, yidx, zidx, params.obs);
// 	  }
// 	}
//   }


  ExplorationPlanner EP;
  EP.Init(params);
   EP.PartialUpdateMap(pts);
//  EP.UpdateMap(map);
  std::vector<Locations_c> goals = EP.NewGoals(robot_loc);

}
