#include "planner.h"
#include "math.h"

Planner::Planner(int maprows, int mapcols, double cell_length, double x, double y, double theta) : map(maprows, mapcols, cell_length, vector<CellStatus>(maprows*mapcols, UNMAPPED)), x(x), y(y), theta(theta) {
}

void Planner::updateMap(const unordered_map<int, CellStatus> &cells) {
  for (unordered_map<int, CellStatus>::const_iterator it = cells.begin();
       it != cells.end(); ++it ) {
    map.setStatus(it->first, it->second);
  }
}

void Planner::updatePose(double x, double y, double theta) {
  this->x = x;
  this->y = y;
  this->theta = theta;
}

void Planner::computeNextStep(double &newx, double &newy, double &newtheta) {
  // Turn until the cell in front of us is unmapped
  double turn_limit = 2*M_PI;
  double dtheta = M_PI/4;
  double dpos = 0.1;
  newtheta = theta;
  newx = x + dpos*cos(newtheta);
  newy = y + dpos*sin(newtheta);
  unordered_set<int> viewed_cells;

  SensorFootprint sensor_footprint(newx, newy, theta);
  sensor_footprint.computeViewedCells(viewed_cells, map);

  while ((!map.inMap(newx, newy) || viewed_cells.size() == 0) && abs(newtheta - theta) < turn_limit) {
    newtheta += dtheta;
    newx = x + dpos*cos(newtheta);
    newy = y + dpos*sin(newtheta);
  }
}

const Map &Planner::getMap() {
  return map;
}
