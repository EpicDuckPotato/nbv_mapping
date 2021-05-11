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

bool Map::inMap(double x, double y) const {
  return 0 <= x && x < xdim && 0 <= y && y < ydim;
}

void Map::getMapDim(double &xdim_, double &ydim_) const {
  xdim_ = xdim;
  ydim_ = ydim;
}

void Map::getCellCenter(int index, double &cx, double &cy){
  int row = index/cols;
  int col = index%cols;
  cx = col*cell_length + cell_length/2;
  cy = row*cell_length + cell_length/2;
}

// if an index is -1, dist is 0
double Map::compute_center_distance(int index1, int index2) {
  if (index1 == -1 || index2 == -1)
    return 0;
  double cx1, cy1, cx2, cy2;
  getCellCenter(index1, cx1, cy1);
  getCellCenter(index2, cx2, cy2);
  return sqrt((cx1-cx2)*(cx1-cx2) + (cy1-cy2)*(cy1-cy2));
}

void Map::findUnmappedCells(vector<int> &unmapped_idxs){
    for (int i = 0; i < rows * cols; i++){
      if (getStatus(i) == UNMAPPED)
        unmapped_idxs.push_back(i);
    }     
}

void Map::getFreeNeighbors(int idx, vector<int>& free_neighbors){
  int row = idx/cols;
  int col = idx%cols;
  for (int i = -1; i < 2; i++){
    int row_new = row + i;
    for (int j = -1; j < 2; j++){
      int col_new = col + j;
      if (row_new != row && col_new != col && row_new >= 0 && row_new < rows && col_new >= 0 && col_new < cols){
        int idx_tmp = row_new * cols + col_new;
        if (getStatus(idx_tmp) == FREE)
          free_neighbors.push_back(idx_tmp);
      }
    }
  }
}

double Map::computeThetaFin(int idx){
  // find a neighboring cell that's unmapped, calculate theta_fin
  int row = idx/cols;
  int col = idx%cols;
  for (int i = -1; i < 2; i++){
    int row_new = row + i;
    for (int j = -1; j < 2; j++){
      int col_new = col + j;
      if (row_new != row && col_new != col && row_new >= 0 && row_new < rows && col_new >= 0 && col_new < cols){
        int idx_tmp = row_new * cols + col_new;
        if (getStatus(idx_tmp) == UNMAPPED)
          return get_angle(j, i);
      }
    }
  }
}

double Map::get_angle(int dx, int dy){
  if (dx == 0){
    if (dy == 1)
      return M_PI/2;
    else
      return M_PI*3/2;
  } else if (dx == 1) {
    if (dy = 0)
      return 0;
    else if (dy == 1)
      return M_PI/4;
    else
      return M_PI*7/4;
  } else { // dx == -1
    if (dy = 0)
      return M_PI;
    else if (dy == 1)
      return M_PI*3/4;
    else
      return M_PI*5/4;
  }
}