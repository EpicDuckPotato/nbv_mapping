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
    double gain; // total gain up till this node
    vector<double> state;
    vector<double> prev_state;
    unordered_set<int> gain_cells; // newly gained cells gaind at node
    Q();

    Q(vector<double> state_);

    Q(double x_, double y_, double theta_);
};


typedef map<vector<double>, Q> Tree;

bool check_if_viewed(int index, Q& qnew, Q& qstart, Tree& tree);

#endif
