#include <explore3d/exploration.hpp>

/* Functions written by Jonathan Michael Butzke except where otherwise indicated
 * uses Bresenham line algorithm for rays
 * (c) 2009, 2014
 */

bool ExplorationPlanner::bresenham_line_3D(int x1, int y1, int z1, int x2, int y2, int z2) {
  /// code from http://www.ict.griffith.edu.au/anthony/info/graphics/bresenham.procs as accessed 28Oct2014 - copyright retained by original author
  int i, dx, dy, dz, l, m, n, x_inc, y_inc, z_inc, err_1, err_2, dx2, dy2, dz2;
  int pixel[3];

  pixel[0] = x1;
  pixel[1] = y1;
  pixel[2] = z1;
  dx = x2 - x1;
  dy = y2 - y1;
  dz = z2 - z1;

  x_inc = (dx < 0) ? -1 : 1;
  l = abs(dx);
  y_inc = (dy < 0) ? -1 : 1;
  m = abs(dy);
  z_inc = (dz < 0) ? -1 : 1;
  n = abs(dz);

  dx2 = l << 1;
  dy2 = m << 1;
  dz2 = n << 1;

  if ((l >= m) && (l >= n)) {
	err_1 = dy2 - l;
	err_2 = dz2 - l;
	for (i = 0; i < l; i++) {
	  if (coverage_.Getval(pixel[0], pixel[1], pixel[2]) == OBS) {
		return false;
	  }
	  if (err_1 > 0) {
		pixel[1] += y_inc;
		err_1 -= dx2;
	  }
	  if (err_2 > 0) {
		pixel[2] += z_inc;
		err_2 -= dx2;
	  }
	  err_1 += dy2;
	  err_2 += dz2;
	  pixel[0] += x_inc;
	}
  } else if ((m >= l) && (m >= n)) {
	err_1 = dx2 - m;
	err_2 = dz2 - m;
	for (i = 0; i < m; i++) {
	  if (coverage_.Getval(pixel[0], pixel[1], pixel[2]) == OBS) {
		return false;
	  }
	  if (err_1 > 0) {
		pixel[0] += x_inc;
		err_1 -= dy2;
	  }
	  if (err_2 > 0) {
		pixel[2] += z_inc;
		err_2 -= dy2;
	  }
	  err_1 += dx2;
	  err_2 += dz2;
	  pixel[1] += y_inc;
	}
  } else {
	err_1 = dy2 - n;
	err_2 = dx2 - n;
	for (i = 0; i < n; i++) {
	  if (coverage_.Getval(pixel[0], pixel[1], pixel[2]) == OBS) {
		return false;
	  }
	  if (err_1 > 0) {
		pixel[1] += y_inc;
		err_1 -= dz2;
	  }
	  if (err_2 > 0) {
		pixel[0] += x_inc;
		err_2 -= dz2;
	  }
	  err_1 += dy2;
	  err_2 += dx2;
	  pixel[2] += z_inc;
	}
  }
  return true;
}

void ExplorationPlanner::raycast3d(SearchPts_c start, int robotnum)
{
    // multi-thread the casting to different points
    // printf("unk loc x%i y%i z%i\n", start.x, start.y, start.z);
    for (size_t pidx = 0; pidx < VisibilityRings_[robotnum][start.z].size(); pidx++) {
        int x, y, z;
        x = VisibilityRings_[robotnum][start.z][pidx].x + start.x;
        y = VisibilityRings_[robotnum][start.z][pidx].y + start.y;
        z = robots_[robotnum].SensorHeight_;
        // printf("x%i y%i z%i", x, y, z);
        if (coverage_.Getval(x, y, z) == FREESPACE) {
            // printf(" Free");
            bool result = bresenham_line_3D(start.x, start.y, start.z, x, y, z);
            if (result) {
                // printf("r%i x%i y%i z%i p%i a%i c%i",
                // robotnum, x, y, start.z, pidx,
                // VisibilityRings_[robotnum][start.z][pidx].angle,
                // counts_[robotnum][x][y][VisibilityRings_[robotnum][start.z][pidx].theta]);
                counts_[robotnum](x, y, VisibilityRings_[robotnum][start.z][pidx].theta) += 1.0;
            }
        }
        // printf("\n");
    }
}
















































//
//bool cast_old_single_ray(int x0, int y0, int x1, int y1, int & xend, int & yend, int & score, unsigned char cover_map[],  unsigned char cost_map[])  {
  //    int xhat, yhat; // current position being checked
  //    int cx, deltax, xstep, xt; // copy, delta & step size, temp
  //    int cy, deltay, ystep, yt;
  //    int error, st; // error and steep flag
  //
  //    // find largest delta for pixel steps
  //    st = (abs(y1 - y0) > abs(x1 - x0)); // st is true if steep
  //
  //    // if deltay > deltax then swap x,y
  //    if (st) {   (xt = y0); (y0 = x0); (x0 = xt); // swap(x0, y0);
  //        (yt = y1); (y1 = x1); (x1 = yt);   } // swap(x1, y1);
  //
  //    deltax = abs(x1 - x0);
  //    deltay = abs(y1 - y0);
  //    error  = (deltax / 2);
  //    yhat = y0;
  //
  //    // set steps in correct direction
  //    if (x0 > x1) { xstep = -1; }
  //    else         { xstep =  1; }
  //
  //    if (y0 > y1) { ystep = -1; }
  //    else         { ystep =  1; }
  //
  //    for ((xhat = x0); (xhat != (x1 + xstep)); (xhat += xstep))  {
	//        // if x,y swapped above, swap them back now
	//        if (st) { (cx = yhat); (cy = xhat); }
	//        else { cx = xhat; cy = yhat; }
	//
	//        //check to see if map grid is occupied
	//        switch(cover_map[cx*map_size_y + cy]) 	{
	  //            case 255:
	  //            case 254:
	  //            case 253:
	  //            case 252:
	  //            case KNOWN: {
		//                    break;
		//                    }
		//            case UNKNOWN:  {
		  //                       cover_map[cx*map_size_y+cy] = KNOWN;
		  //                       score += UNKSCORE;
		  //                       break;
		  //                       }
		  //            case OBSTACLE: { xend = cx; yend = cy;  return true; }
		  //            default: {
			//                     score += cover_map[cx*map_size_y+cy];
			//                     cover_map[cx*map_size_y+cy] = KNOWN;
			//                     break;
			//                     }
			//        } // switch on map
			//
			//        // take step
			//        (error -= deltay); // converge toward end of line
			//
			//        if (error < 0) { // not done yet
			//            (yhat += ystep);
			//            (error += deltax);
			//        } // if error
			//    } //for xhat
			//return false;
			//} // cast single ray
			//
			//

			// 			void GPLAN::cast_single_ray(int x0, int y0, int x1, int y1, int & score, unsigned char cover_map[], const int16_t elev[])  {
			  // 			  // function performs Bresenham line algorithm but enforces 4 connected movement instead of 8.
			  //
			  // 			  int xhat, yhat; // current position being checked
			  // 			  int cx, deltax, xstep, xt; // copy, delta & step size, temp
			  // 			  int cy, deltay, ystep, yt;
			  // 			  int error, st; // error and steep flag
			  // 			  int16_t height = elev[x0+ map_size_x*y0] + sensor_height;
			  //
			  // 			  //xend = 0; yend = 0;
			  //
			  // 			  // find largest delta for pixel steps
			  // 			  st = (abs(y1 - y0) > abs(x1 - x0)); // st is true if steep
			  //
			  // 			  // if deltay > deltax then swap x,y
			  // 			  if (st) {   (xt = y0); (y0 = x0); (x0 = xt); // swap(x0, y0);
			  // 		(yt = y1); (y1 = x1); (x1 = yt);   } // swap(x1, y1);
			  //
			  // 			  deltax = abs(x1 - x0);
			  // 			  deltay = abs(y1 - y0);
			  // 			  error  = (deltax / 2);
			  // 			  yhat = y0;
			  //
			  // 			  // set steps in correct direction
			  // 			  if (x0 > x1) { xstep = -1; }
			  // 			  else         { xstep =  1; }
			  //
			  // 			  if (y0 > y1) { ystep = -1; }
			  // 			  else         { ystep =  1; }
			  //
			  // 			  for ((xhat = x0); (xhat != (x1 + xstep)); (xhat += xstep))  {
				// 				// if x,y swapped above, swap them back now
				// 				if (st) { (cx = yhat); (cy = xhat); }
				// 				else { cx = xhat; cy = yhat; }
				//
				// 				//check to see if map grid is occupied
				// 				if (height-elev[cx+map_size_x*cy]>0) {
				  // 				  score += KNOWN - cover_map[cx+map_size_x*cy];
				  // 				  cover_map[cx+map_size_x*cy] = KNOWN;
				  // 				}
				  // 				else {
					// 				  //xend = cx; yend = cy;
					// 				  return;
					// 				}
					//
					// 				//switch(cost_map[cx+map_size_x*cy]) 	{
					  // 				//case 255:
					  // 				//case 254:
					  // 				//case 253:
					  // 				//case 252:  {
						// 				  //break;
						// 				  //}
						// 				  //case OBSTACLE: { xend = cx; yend = cy;
						// 				  //cout << " OBST SCORE: " << xend << "," << yend << ":" << score << endl;
						// 				  //return true; }
						// 				  //default: {
						  // 					//score += KNOWN - cover_map[cx+map_size_x*cy];
						  // 					//cover_map[cx+map_size_x*cy] = KNOWN;
						  // 					//break;
						  // 					//}
						  // 					//} // switch on map
						  //
						  // 					// take step
						  // 					(error -= deltay); // converge toward end of line
						  //
						  // 					if (error < 0) { // not done yet
						  //
						  // 			(error += deltax);
						  // 			// if x,y swapped above, swap them back now
						  // 			if  ((xhat != x1) && (yhat != y1)) {
							// 			  xhat +=xstep;
							// 			  if (st) { (cx = yhat); (cy = xhat); }
							// 			  else { cx = xhat; cy = yhat; }
							// 			  xhat -=xstep;
							// 			  //check to see if map grid is occupied
							// 			  if (height-elev[cx+map_size_x*cy]>0) {
							  // 				score += KNOWN - cover_map[cx+map_size_x*cy];
							  // 				cover_map[cx+map_size_x*cy] = KNOWN;
							  // 			  }
							  // 			  else {
								// 				//xend = cx; yend = cy;
								// 				return;
								// 			  }
								// 			  //switch(cost_map[cx+map_size_x*cy]) 	{
								  // 					  //    case 255:
								  // 					  //    case 254:
								  // 					  //    case 253:
								  // 					  //    case 252:  {
									// 						//        break;
									// 						//        }
									// 						//    case OBSTACLE: { xend = cx; yend = cy;
									// 						//                       //cout << " obst2 SCORE: "<< xend << "," << yend << ":" << score << endl;
									// 						//                       return true; }
									// 						//    default: {
									  // 						  //        score += KNOWN - cover_map[cx+map_size_x*cy];
									  // 						  //        cover_map[cx+map_size_x*cy] = KNOWN;
									  // 						  //        break;
									  // 						  //        }
									  // 						  //} // switch on map
									  // 			}
									  // 			(yhat += ystep);
									  // 			  } // if error
									  // 			  } //for xhat
									  // 			  //cout << " SCORE: "<< xend << "," << yend << ":" << score << endl;
									  // 			  return;
									  // 			  } // cast modified single ray
									  //
									  //

