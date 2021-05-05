#ifndef PLANNER_H
#define PLANNER_H

#include <unordered_map>
#include "map.h"
#include "tree.h"
#include "sensor_footprint.h"
#include <math.h>
#include <iostream>
#include <vector>
#include <time.h>
#include "KDTree.h"
#include <stdio.h>
#include <stdlib.h>
#include<time.h>
#include <bits/stdc++.h>
#include <chrono>
#include <unistd.h>


#define DIM 3

class Planner {
  public:
    /*
     * CONSTRUCTOR: initializes empty map 
     * ARGUMENTS
     * maprows, mapcols: map size
     * cell_length: side length of each cell in the map
     * x, y, theta: starting pose
     * sf_depth, sf_width: sensor footprint dimensions. See sensor_footprint.h
     * for more info
     */
    Planner(int maprows, int mapcols, double cell_length, double x, double y, double theta,
            double sf_depth, double sf_width);

    /*
     * updateMap: updates status of cells in the map
     * cells: maps map indices to status values
     */
    void updateMap(const unordered_map<int, CellStatus> &cells);

    /*
     * updatePose: updates robot pose
     * x, y, theta: new pose
     */
    void updatePose(double x, double y, double theta);

    /*
     * computeNextStep: computes next pose
     * ARGUMENTS
     * newx, newy, newtheta: populated with new pose
     */
    void computeNextStep(double &newx, double &newy, double &newtheta);


    /*
     * getMap: gets the map for visualization
     * RETURN: constant reference to map
     */
    const Map &getMap();

  private:
    Map map;
    double x;
    double y;
    double theta;
    double sf_depth;
    double sf_width;

    /* we need this map for sensing */
    // Map ground_truth_map;

    /* 
     * we store the trees using back pointers
     * we use kd tree to find nn
     */
    KDTree<DIM, vector<double>> kd_tree;
    vector<vector<double>> banked_steps_stack;
    int exploration_number = 0;
    int counter = 0;
    int N_max = 15;
    int N_tol = 100000;
    Q qstart;
    Q q_best;
    Tree tree;
    Tree next_tree;
    // interpolation resolution, currently set to arbitrary values
    double X_RES = 0.1;
    double Y_RES = 0.05;
    double THETA_RES = M_PI/20;
    // delta_theta between two steps is in range [-pi/4, pi/4)
    double DTHETA_MAX = M_PI/4;

    double EPS = 0.64; // distance squared. distance is 0.8m

    int stuck_time = 0;
    int MAX_STUCK_TIME = 3;
    /*
     * sample_new: samples a new (x,y,dtheta), where dtheta is the the difference between the new theta and its predecessor's theta, which is currently unknown
     * ARGUMENTS
     * xdim, ydim: dimention of the map
     * RETURN: new node 
     */
    Q sample_new(double xdim, double ydim);


    Q sample_new(double left, double right, double bottom, double top);

    /*
     * point_from_q: converts a Q into kdtree-acceptable Point format
     * ARGUMENTS
     * q: the node to convert
     * RETURN: the Point correspoinding to q.state
     */
    Point<DIM> point_from_q(Q& q);

    /*
     * add_new: adds a new node to the tree and the kdtree, also compute the information gain
     * ARGUMENTS
     * t: the tree
     * kd: the kdtree
     * qnew, qnear: goal node and start node
     * RETURN: the information gain after moving from qnear to qnew
     */
    int add_new(Tree& t, KDTree<DIM, vector<double>>& kd, Q& qnew, Q& qnear);

    /*
     * print_vector: prints a vector for debugging
     * ARGUMENTS
     * vec: vector to be printed
     */
    void print_vector(vector<double> vec);

    void print_tree();



    /*
     * extend: tries to extend the tree to node q, fills out the actual extended node qnew, and computes the information gain
     * ARGUMENTS
     * q: extension target
     * qnew: actual node extended 
     * RETURN: the information gain after moving from qnear to qnew
     */
    int extend(Q& q, Q& qnew);

    /*
     * vec_to_array: turns a vector into an array
     * ARGUMENTS
     * vec: the source vector
     * arr: the array to be filled out
     * arrLength: length of the arr to be filled
     */
    void vec_to_array(vector<double>& vec, double *arr, int arrLength);

    /*
     * array_to_vector: turns an array into a vector
     * ARGUMENTS
     * the source vector
     * arr: the source array
     * arrLength: length of the arr
     * RETURN: destination vector
     */
    vector<double> array_to_vector(double *arr, int arrLength);

    /*
     * new_config: tries to connect qfrom with qto, and fills out the actual extended node qnew
     * ARGUMENTS
     * qfrom, qto: start and goal of the connection process
     * qnew: the actual extended node, or nothing if trapped
     * RETURN: true if the connection process isn't trapped
     */
    bool new_config(Q& qfrom, Q& qto, Q& qnew);

    /*
     * get_plan: backtracks the best branch in the tree, saves it for the next exploration step, and fills out the next step
     * ARGUMENTS
     * newx, newy, newtheta: next step to take, which is the 1st waypoint after qstart in the best branch
     * qlast, qstart: the last and the 1st nodes in the best branch
     * RETURN: true if they're in the map, false if not
     */
    void get_plan(double &newx, double &newy, double &newtheta, Q& qlast);

    void store_all_steps(double& newx, double& newy, double& newtheta, Q& qlast);

    void pop_from_step_bank(double& newx, double& newy, double& newtheta);


    void get_random_move(double &newx, double& newy, double& newtheta, Q& qbest, double &xdim, double &ydim);

    void get_Astar_move(double &newx, double& newy, double& newtheta, Q& qbest);

    /*
     * isValidConfiguration: checks if the given x, y coordinates are FREE
     * ARGUMENTS
     * x, y: coordinates to check
     * RETURN: true if the cell is FREE, false if not
     */
    bool isValidConfiguration(double x, double y) const;

    /*
     * inInRange: checks if two (x,y,theta) vectors are within sqrt(EPS) euclidean distance
     * ARGUMENTS
     * new_state, state: states to check
     * RETURN: true if they're in range, false if not
     */
    bool isInRange(vector<double>& new_state, vector<double>& state);

    /*
     * updateGain: computes and fills out the TOTAL mapped cells up till we finished extending the tree from qprev to qnew. We also update the gain_cells for qnew. 
     * ARGUMENTS
     * qnew, qprev: previous node and newly added node
     * RETURN: the information gain at qnew, which equals the size of qnew.gain_cells
     */
    int updateGain(Q& qnew, Q& qprev);

    /*
     * process_angles: converts an angle in range [-2pi, 4pi) to the canonical range [0, 2pi)
     * ARGUMENTS
     * angle: source angle
     * RETURN: destination angle
     */
    double process_angle(double angle);

    /*
     * wrap_around_theta: changes the value of one of the input thetas such that abs(theta1-theta2) <= pi
     * ARGUMENTS
     * theta1, theta2: angles to wrap around
     */
    void wrap_around_theta(double& theta1, double& theta2);
};

#endif
