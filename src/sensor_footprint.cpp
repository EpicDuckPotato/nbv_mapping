#include "sensor_footprint.h"
#include <math.h>
#include "geometry.h"
#include <algorithm>

SensorFootprint::SensorFootprint(double x, double y, double theta,
                                 double depth, double width) {
  xs[0] = x;  
  ys[0] = y;  

  xs[1] = x + depth*cos(theta) + width*cos(theta + M_PI/2)/2;
  ys[1] = y + depth*sin(theta) + width*sin(theta + M_PI/2)/2;

  xs[2] = x + depth*cos(theta) - width*cos(theta + M_PI/2)/2;
  ys[2] = y + depth*sin(theta) - width*sin(theta + M_PI/2)/2;
} 

void SensorFootprint::computeViewedCells(unordered_set<int> &viewed_cells,
                                         const Map &map) {
  // Examine all cells intersecting bounding box
  double left;
  double right;
  double bottom;
  double top;
  computeBoundingBox(left, right, bottom, top);

  double cell_length = map.getCellLength();
  for (double y = bottom; y <= top; y += cell_length) {
    for (double x = left; x <= right; x += cell_length) {
      int index = map.getMapIdx(x, y);

      // If this cell is already mapped, skip it
      if (map.getStatus(index) != 2) {
        continue;
      }

      double cell_left;
      double cell_right;
      double cell_bottom;
      double cell_top;
      map.getCellExtent(cell_left, cell_right, cell_bottom, cell_top, index);

      // Check if any corner of this cell is inside the footprint
      array<double, 4> cell_xs;
      array<double, 4> cell_ys;
      getBoxCorners(cell_xs[0], cell_ys[0],
                    cell_xs[1], cell_ys[1],
                    cell_xs[2], cell_ys[2],
                    cell_xs[3], cell_ys[3],
                    cell_left, cell_right, cell_bottom, cell_top);
      if (containsPoint(cell_xs[0], cell_ys[0]) ||
          containsPoint(cell_xs[1], cell_ys[1]) ||
          containsPoint(cell_xs[2], cell_ys[2]) ||
          containsPoint(cell_xs[3], cell_ys[3])) {
        viewed_cells.insert(index);
        continue;
      }

      // Check if any corner of the footprint is inside the cell
      if (boxContainsPoint(xs[0], ys[0], cell_left, cell_right, cell_bottom, cell_top) || 
          boxContainsPoint(xs[1], ys[1], cell_left, cell_right, cell_bottom, cell_top) ||
          boxContainsPoint(xs[2], ys[2], cell_left, cell_right, cell_bottom, cell_top)) {
        viewed_cells.insert(index);
        continue;
      }

      // Check if any edge of the footprint intersects any edge of the cell

      // Iterate over starting coordinate for footprint edge (ending coordinate is a given)
      bool edges_intersect = false;
      for (int t = 0; t < 3 && !edges_intersect; ++t) {
        // Iterate over starting coordinate for cell edge (ending coordinate is a given)
        double x1i = xs[t];
        double y1i = ys[t];
        int tp1 = (t + 1)%3;
        double x1f = xs[tp1];
        double y1f = ys[tp1];
        for (int c = 0; c < 4; ++c) {
          double x2i = cell_xs[c];
          double y2i = cell_ys[c];
          int cp1 = (c + 1)%4;
          double x2f = cell_xs[cp1];
          double y2f = cell_ys[cp1];
          if (lineSegmentsIntersect(x1i, y1i, x1f, y1f, x2i, y2i, x2f, y2f)) {
            viewed_cells.insert(index);
            edges_intersect = true;
            break;
          }
        }
      }
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
