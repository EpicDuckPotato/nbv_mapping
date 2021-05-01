#ifndef PLANNER_H
#define PLANNER_H

#include <unordered_map>
#include "map.h"
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
#include "tree.h"

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

    Map ground_truth_map;
    // we store the trees using back pointers
    // we use kd tree to find nn
    KDTree<DIM, vector<double>> kd_tree;

    int counter = 0;
    int N_max = 15;
    int N_tol = 100;
    Tree tree;
    Tree next_tree;
    // currently set to arbitrary values
    double X_RES = 0.1;
    double Y_RES = 0.05;
    double THETA_RES = M_PI/20;

    Q sample_new(double left, double right, double bottom, double top);
    Point<DIM> point_from_q(Q& q);
    int add_new(Tree& t, KDTree<DIM, vector<double>>& kd, Q& qnew, Q& qnear);
    void print_vector(vector<double> vec);
    int extend(Q& q, Q& qnew, int numofDOFs);
    void vec_to_array(vector<double>& vec, double *arr, int arrLength);
    vector<double> array_to_vector(double *arr, int arrLength);
    bool new_config(Q& qfrom, Q& qto, Q& qnew, int numofDOFs);
    void get_plan(double& newx, double& newy, double& newtheta, Q& qlast, Q& qstart);

    bool isValidConfiguration(double x, double y) const;

    int updateGain(Q& qnew, Q& qprev);
};

#endif
