#include "planner.h"
#include "math.h"

Planner::Planner(int maprows, int mapcols, double cell_length, double x, double y, double theta, double sf_depth, double sf_width) : map(maprows, mapcols, cell_length, vector<CellStatus>(maprows*mapcols, UNMAPPED)), x(x), y(y), theta(theta), sf_depth(sf_depth), sf_width(sf_width) {
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
  newx = x + dpos;
  newy = y + 0.51*dpos;
  newtheta = M_PI;
}

const Map &Planner::getMap() {
  return map;
}
