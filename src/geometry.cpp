#include "geometry.h"

bool boxContainsPoint(double x, double y, double left, double right, double top, double bottom) {
  return left <= x && x <= right && bottom <= y && y <= top;
}

void getBoxCorners(double &x1, double &y1,
                   double &x2, double &y2,
                   double &x3, double &y3,
                   double &x4, double &y4,
                   double left, double right,
                   double top, double bottom) {
  x1 = left;
  y1 = bottom;

  x2 = left;
  y2 = top;

  x3 = right;
  y3 = top;

  x4 = right;
  y4 = bottom;
}

bool lineSegmentsIntersect(double x1i, double y1i, double x1f, double y1f,
                           double x2i, double y2i, double x2f, double y2f) {
  double a = x1f - x1i;
  double b = x2f - x2i;
  double c = y1f - y1i;
  double d = y2f - y2i;

  double det = a*d - b*c;

  if (det == 0) {
    // Segments are parallel. Even if they're collinear,
    // this is practically not an intersection anyway, so just
    // return false
    return false;
  }

  double lx = x1i - x2i;
  double ly = y1i - y2i;

  double tx = -(d*lx - b*ly)/det;
  double ty = (c*lx - a*ly)/det;
  return 0 < tx && tx < 1 && 0 < ty && ty < 1;
}
