#include "sensor_footprint.h"
#include <math.h>
#include "geometry.h"
#include <algorithm>
#include <iostream>

SensorFootprint::SensorFootprint(double x, double y, double theta,
                                 double depth, double width) : width(width) {
  xs[0] = x;  
  ys[0] = y;  

  xs[1] = x + depth*cos(theta) + width*cos(theta + M_PI/2)/2;
  ys[1] = y + depth*sin(theta) + width*sin(theta + M_PI/2)/2;

  xs[2] = x + depth*cos(theta) - width*cos(theta + M_PI/2)/2;
  ys[2] = y + depth*sin(theta) - width*sin(theta + M_PI/2)/2;
} 

void SensorFootprint::computeVisibleCells(unordered_set<int> &visible_cells,
                                          const Map &map) {
  // Split the far end of the sensor footprint (i.e. the base of the cone) into
  // segments with length at most map.getCellLength() (if they're longer,
  // we might skip over cells). Cast a ray to the endpoints of each
  // segment using digital differential analysis, stopping when we hit
  // a cell that's UNMAPPED or OCCUPIED. The cell we stop on is the last
  // cell we add to visible cells that's associated with this ray
  double cell_length = map.getCellLength();
  int segments = (int)((width + cell_length)/cell_length);
  int samples = segments + 1;
  for (int sample = 0; sample < samples; ++sample) {
    double frac = sample/(double)segments;
    double xend = xs[1] + frac*(xs[2] - xs[1]);
    double yend = ys[1] + frac*(ys[2] - ys[1]);

    double x = xs[0];
    double y = ys[0];

    double dispx = xend - x;
    double dispy = yend - y;
    double dist = sqrt(dispx*dispx + dispy*dispy);

    double dydx = dispy/dispx;
    double xhyp = sqrt(1 + dydx*dydx);
    double dxdy = dispx/dispy;
    double yhyp = sqrt(1 + dxdy*dxdy);

    bool onlyx = yend == y;
    bool onlyy = xend == x;

    double xstep = (xend < x) ? -cell_length : cell_length;
    double ystep = (yend < y) ? -cell_length : cell_length;

    double left;
    double right;
    double bottom;
    double top;
    map.getCellExtent(left, right, bottom, top, map.getMapIdx(x, y));

    double newx = (xstep < 0) ? left : right;
    double newy = (ystep < 0) ? bottom : top;

    double xray = abs(newx - x)*xhyp;
    double yray = abs(newy - y)*yhyp;

    int index = map.getMapIdx(x, y);
    CellStatus status = map.getStatus(index); 

    int end_idx = map.getMapIdx(xend, yend);

    double raylen = 0;
    while (index != end_idx && raylen < dist && 
           status != OCCUPIED && status != UNMAPPED) {
      if (onlyy || xray > yray) {
        x += (newy - y)*dxdy;
        y = newy;
        raylen = yray;

        newy += ystep;
        yray += cell_length*yhyp;
      } else if (onlyx || yray >= xray) {
        y += (newx - x)*dydx;
        x = newx;
        raylen = xray;

        newx += xstep;
        xray += cell_length*xhyp;
      }

      if (!map.inMap(x, y)) {
        break;
      }

      index = map.getMapIdx(x, y);
      visible_cells.insert(index);
      status = map.getStatus(index); 
    }
  }
}

bool SensorFootprint::containsPoint(double x, double y) {
  // Algorithm from here: http://totologic.blogspot.com/2014/01/accurate-point-in-triangle-test.html
  double denom = (ys[1] - ys[2])*(xs[0] - xs[2]) + (xs[2] - xs[1])*(ys[0] - ys[2]);
  double a = ((ys[1] - ys[2])*(x - xs[2]) + (xs[2] - xs[1])*(y - ys[2])) / denom;
  if (a < 0 || a > 1) {
    return false;
  }
  double b = ((ys[2] - ys[0])*(x - xs[2]) + (xs[0] - xs[2])*(y - ys[2])) / denom;
  if (b < 0 || b > 1) {
    return false;
  }
  double c = 1 - a - b;
  return 0 <= c && c <= 1;
}


void SensorFootprint::computeBoundingBox(double &left, double &right, double &bottom, double &top) {
  left = *(std::min_element(xs.begin(), xs.end()));
  right = *(std::max_element(xs.begin(), xs.end()));
  bottom = *(std::min_element(ys.begin(), ys.end()));
  top = *(std::max_element(ys.begin(), ys.end()));
}
