#ifndef PLANNER_H
#define PLANNER_H

#include <unordered_map>
#include "map.h"
#include "sensor_footprint.h"

class Planner {
  public:
    /*
     * CONSTRUCTOR: initializes empty map 
     * ARGUMENTS
     * maprows, mapcols: map size
     * cell_length: side length of each cell in the map
     * x, y, theta: starting pose
     * sf_depth, sf_width: sensor footprint dimensions. See sensor_footprint.h
     * for more info
     */
    Planner(int maprows, int mapcols, double cell_length, double x, double y, double theta,
            double sf_depth, double sf_width);

    /*
     * updateMap: updates status of cells in the map
     * cells: maps map indices to status values
     */
    void updateMap(const unordered_map<int, CellStatus> &cells);

    /*
     * updatePose: updates robot pose
     * x, y, theta: new pose
     */
    void updatePose(double x, double y, double theta);

    /*
     * computeNextStep: computes next pose
     * ARGUMENTS
     * newx, newy, newtheta: populated with new pose
     */
    void computeNextStep(double &newx, double &newy, double &newtheta);

    /*
     * getMap: gets the map for visualization
     * RETURN: constant reference to map
     */
    const Map &getMap();

  private:
    Map map;
    double x;
    double y;
    double theta;
    double sf_depth;
    double sf_width;
};

#endif
