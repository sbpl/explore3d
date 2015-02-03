#include <explore3d/exploration.hpp>

/* Functions written by Jonathan Michael Butzke except where otherwise indicated
 * uses Bresenham line algorithm for rays
 * (c) 2009, 2014
 */

bool ExplorationPlanner::bresenham_line_3D(int x1, int y1, int z1, int x2, int y2, int z2)
{
    /// code from http://www.ict.griffith.edu.au/anthony/info/graphics/bresenham.procs as accessed 28oct2014 - copyright retained by original author
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
    }
    else if ((m >= l) && (m >= n)) {
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
    }
    else {
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
    // TODO: multi-thread the casting to different points
    for (size_t pidx = 0; pidx < VisibilityRings_[robotnum][start.z].size(); pidx++) {
        int x, y, z;
        x = VisibilityRings_[robotnum][start.z][pidx].x + start.x;
        y = VisibilityRings_[robotnum][start.z][pidx].y + start.y;
        z = robots_[robotnum].SensorHeight_;
        if (coverage_.Getval(x, y, z) == FREESPACE) {
            bool result = bresenham_line_3D(start.x, start.y, start.z, x, y, z);
            if (result) {
                counts_[robotnum](x, y, VisibilityRings_[robotnum][start.z][pidx].theta) += 1.0;
            }
        }
    }
}
