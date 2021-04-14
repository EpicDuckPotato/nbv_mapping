#ifndef SENSOR_FOOTPRINT_H
#define SENSOR_FOOTPRINT_H
#include "map.h"
#include <unordered_set>

class SensorFootprint {
  public:
    /*
     * CONSTRUCTOR: initializes the three points of the triangle
     * indicating the sensor footprint using the given robot pose.
     * The current implementation uses a triangle with 1 m height
     * and 1 m base
     * ARGUMENTS
     * x, y, theta: robot pose in meters and radians
     */
    SensorFootprint(double x, double y, double theta);

    /*
     * computeViewedCells: determines which unmapped cells in the map
     * intersect with the sensor footprint 
     * ARGUMENTS
     * viewed_cells: populated with map indices of viewed cells
     * footprint: sensor footprint
     */
    void computeViewedCells(unordered_set<int> &viewed_cells,
                            const Map &map);

  private:
    array<double, 3> xs;
    array<double, 3> ys;

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
