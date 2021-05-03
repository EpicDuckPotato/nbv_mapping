#include "tree.h"

Q::Q(){
    gain = 0;
}

Q::Q(vector<double> state_){
    gain = 0;
    state = state_;
}

Q::Q(double x_, double y_, double theta_) {
    gain = 0;
    state.push_back(x_);
    state.push_back(y_);
    state.push_back(theta_);
}

bool check_if_viewed(int index, Q& qnew, Q& qstart, Tree& tree) {
  Q qtmp = qnew;
  vector<double> key;
  while (qtmp.state != qstart.state){
    key = qtmp.prev_state;
    qtmp = tree[key];
    if (qtmp.gain_cells.find(index) != qtmp.gain_cells.end()){
      return true;
    }
  }
  if (qtmp.gain_cells.find(index) != qtmp.gain_cells.end()){
      return true;
  }
  return false;
}
