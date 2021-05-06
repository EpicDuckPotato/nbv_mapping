#ifndef SENSOR_FOOTPRINT_H
#define SENSOR_FOOTPRINT_H
#include "map.h"
#include <unordered_set>

class SensorFootprint {
  public:
    /*
     * CONSTRUCTOR: initializes the three points of the triangle
     * indicating the sensor footprint using the given robot pose
     * ARGUMENTS
     * x, y, theta: robot pose in meters and radians
     * depth: depth of sensor footprint. In 3D, this would be the
     * height of the cone
     * width: width of sensor footprint. In 3D, this would be the
     * cone diameter
     */
    SensorFootprint(double x, double y, double theta, double depth, double width);

    /*
     * computeVisibleCells: determines which cells the robot can definitely see right now.
     * If a cell is behind an OCCUPIED cell, it is not inserted into the result
     * ARGUMENTS
     * visible_cells: populated with map indices of cells that are definitely visible
     * map: map
     */
    void computeVisibleCells(unordered_set<int> &visible_cells,
                             const Map &map);

  private:
    array<double, 3> xs;
    array<double, 3> ys;
    double width;

    /*
     * computeBoundingBox: gets the box that bounds the sensor footprint
     * and whose sides align with the x and y axes
     * ARGUMENTS
     * left: populated with lowest x coordinate of bounding box
     * right: populated with highest x coordinate of bounding box
     * bottom: populated with lowest y coordinate of bounding box
     * top: populated with highest y coordinate of bounding box
     */
    void computeBoundingBox(double &left, double &right, double &bottom, double &top);

    /*
     * containsPoint: determines whether the sensor footprint contains
     * a certain point
     * ARGUMENTS
     * x, y: point to check
     * RETURN: true if the point is contained in the sensor footprint, false if not
     */
    bool containsPoint(double x, double y);
};
#endif
