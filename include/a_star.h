#ifndef A_STAR_H
#define A_STAR_H

#include <stdlib.h>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <map>
#include <unordered_set>
#include <queue>

using namespace std;

// data structure to store each state
class Node {
public:
    int index;
    Node *predecessor;
    double g;
    double h;
    Node();
    Node(int index_);
};



class AugmentedNode{
public:
    Node *node;

    AugmentedNode();

    AugmentedNode(Node *node_);

    friend bool operator<(const AugmentedNode& s1, const AugmentedNode& s2){
        if(s1.node->g + s1.node->h > s2.node->g + s2.node->h)
            return true;
        return false;
    }

};


#endif