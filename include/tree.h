#ifndef TREE_H
#define TREE_H

#include <stdlib.h>
#include <unordered_map>
#include <vector>
#include <algorithm>

using namespace std;

// data structure to store each config
class Q {
public:
    vector<double> state;
    vector<double> prev_state;
    unordered_set<int> gain_cells;
    Q();
    Q(vector<double> state_);
    Q(double x_, double y_, double theta_);

    
};

Q::Q(){
}

Q::Q(vector<double> state_) {
   state = state_;
}

Q::Q(double x_, double y_, double theta_) {
    state.push_back(x_);
    state.push_back(y_);
    state.push_back(theta_);
}


typedef map<vector<double>, Q> Tree;


#endif