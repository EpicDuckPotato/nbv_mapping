#ifndef GEOMETRY_H
#define GEOMETRY_H

/*
 * boxContainsPoint: determines whether a box contains a point
 * ARGUMENTS
 * x, y: coordinates of the point
 * left, right, bottom, top: extent of the box
 * RETURN: true i the box contains the point, false otherwise
 */
bool boxContainsPoint(double x, double y, double left, double right, double bottom, double top);

/*
 * getBoxCorners: get corners of a box, given the extent.
 * Guaranteed to go clockwise or counterclockwise
 * ARGUMENTS
 * x1, y1: populated with first coordinate
 * x2, y2: populated with second coordinate
 * x3, y3: populated with third coordinate
 * x4, y4: populated with fourth coordinate
 * left, right, bottom, top: extent of box
 */
void getBoxCorners(double &x1, double &y1,
                   double &x2, double &y2,
                   double &x3, double &y3,
                   double &x4, double &y4,
                   double left, double right,
                   double bottom, double top);

/*
 * lineSegmentsIntersect: determines whether two line segments intersect
 * ARGUMENTS
 * x1i, y1i, x1f, y1f: initial and final coordinates of segment 1
 * x2i, y2i, x2f, y2f: initial and final coordinates of segment 2
 * RETURN: true if the segments intersect, false if not
 */
bool lineSegmentsIntersect(double x1i, double y1i, double x1f, double y1f,
                           double x2i, double y2i, double x2f, double y2f);

#endif
