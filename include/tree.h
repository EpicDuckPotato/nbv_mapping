#ifndef TREE_H
#define TREE_H

#include <stdlib.h>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <map>
#include <unordered_set>

using namespace std;

// data structure to store each config
class Q {
public:
    int gain; // total gain up till this node
    vector<double> state;
    vector<double> prev_state;
    unordered_set<int> gain_cells; // newly gained cells gaind at node
    Q(){
        gain = 0;
    }

    Q(vector<double> state_){
        gain = 0;
        state = state_;
    }

    Q(double x_, double y_, double theta_){
        gain = 0;
        state.push_back(x_);
        state.push_back(y_);
        state.push_back(theta_);
    }

    
};


typedef map<vector<double>, Q> Tree;

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


#endif
