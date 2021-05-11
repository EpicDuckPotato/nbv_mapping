#include "a_star.h"

Node::Node(){
    g = -1;
    h = -1;
}

Node::Node(int index_){
    index = index_;
    g = -1;
    h = -1;
}


AugmentedNode::AugmentedNode(){}

AugmentedNode::AugmentedNode(Node *node_){
    node = node_;
}


