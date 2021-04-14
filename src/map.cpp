#include "map.h"

Map::Map(int rows, int cols, double cell_length, const vector<CellStatus> &cells) : rows(rows),
                                                                                    cols(cols),
                                                                                    cell_length(cell_length),
                                                                                    cells(cells) {
  xdim = cell_length*cols;
  ydim = cell_length*rows;
}

int Map::getMapIdx(double x, double y) const {
  int row = (int)((y/ydim)*rows);
  int col = (int)((x/xdim)*cols);
  return row*cols + col;
}

CellStatus Map::getStatus(double x, double y) const {
  return cells[getMapIdx(x, y)];
}

CellStatus Map::getStatus(int index) const {
  return cells[index];
}

double Map::getCellLength() const {
  return cell_length;
}

void Map::getCellExtent(double &left, double &right, double &bottom, double &top, int index) const {
  int row = index/cols;
  int col = index%cols;
  left = col*cell_length;
  right = left + cell_length;
  bottom = row*cell_length;
  top = bottom + cell_length;
}

void Map::getCellPos(double &left, double &bottom, int index) const {
  int row = index/cols;
  int col = index%cols;
  left = col*cell_length;
  bottom = row*cell_length;
}

void Map::setStatus(int index, CellStatus val) {
  cells[index] = val;
}
