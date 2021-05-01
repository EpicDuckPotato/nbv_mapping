#include "planner.h"
#include "math.h"


#define TRAPPED 100
#define ADVANCED 101
#define REACHED 102

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif


Planner::Planner(int maprows, int mapcols, double cell_length, double x, double y, double theta, double sf_depth, double sf_width) : map(maprows, mapcols, cell_length, vector<CellStatus>(maprows*mapcols, UNMAPPED)), x(x), y(y), theta(theta), sf_depth(sf_depth), sf_width(sf_width) {
}

const Map &Planner::getMap() {
  return map;
}

void Planner::updateMap(const unordered_map<int, CellStatus> &cells) {
  for (unordered_map<int, CellStatus>::const_iterator it = cells.begin();
       it != cells.end(); ++it ) {
    map.setStatus(it->first, it->second);
  }
}

void Planner::updatePose(double x, double y, double theta) {
  this->x = x;
  this->y = y;
  this->theta = theta;
}

void Planner::computeNextStep(double &newx, double &newy, double &newtheta) {

  //ξ0 ← current vehicle configuration
  //Initialize T with ξ0 and, unless first planner call, also previous best branch
  //gbest ← 0 . Set best gain to zero
  //nbest ← n0(ξ0) . Set best node to root
  //NT ← Number of initial nodes in T
  //while NT < Nmax or gbest = 0 do
    //Incrementally build T by adding nnew(ξnew)
    //NT ← NT + 1
    //if Gain(nnew) > gbest then
      //nbest ← nnew
      //gbest ← Gain(nnew)
    //if NT > N_TOL then
      //Terminate exploration
  //σ ← ExtractBestPathSegment(nbest)
  //Delete T


  Q qstart = Q(x, y, theta);
  SensorFootprint sf_start(x, y, theta, sf_depth, sf_width);
  sf_start.computeViewedCells(qstart.gain_cells, map);
  tree[qstart.state] = qstart;
  Point<DIM> ps = point_from_q(qstart);
  kd_tree.insert(ps, qstart.state);
  double left, right, bottom, top;
  map.getROI(left, right, bottom, top, x, y);
  int g_best = 0;
  Q qbest = qstart;
  Q qnew;
  // for k in 1 to K:
  while (counter < N_max || g_best == 0){
    // sample a point qrand in config space
    Q qrand = sample_new(left, right, bottom, top);
    int gain;
    gain = extend(qrand, qnew, DIM);
    if (gain == 0) // we're trapped
      continue;
    counter += 1;
    if (gain > g_best){
      qbest = qnew;
      g_best = gain;
    }
    if (counter > N_tol)
      break;
  }
  // TODO back up best branch
  // first step in the best branch
  get_plan(newx, newy, newtheta, qnew, qstart);
}

bool Planner::isValidConfiguration(double x, double y) const {
  if (map.inMap(x, y)){
    if (map.getStatus(map.getMapIdx(x, y)) == FREE){
      return true;
    }
  }
  return false;
}

Q Planner::sample_new(double left, double right, double bottom, double top){
  vector<double> state;
  // x
  state.push_back(left + (right - left) * ((double)rand() / RAND_MAX));
  // y
  state.push_back(top + (bottom - top) * ((double)rand() / RAND_MAX));
  // theta
  state.push_back(2*M_PI * ((double)rand() / RAND_MAX));
  
  return Q(state);
}


Point<DIM> Planner::point_from_q(Q& q){
  Point<DIM> q_point;
  vector<double> q_state = q.state;
  copy(q_state.begin(), q_state.end(), q_point.begin());
  return q_point;
}

// compute gain (# mapped cells up till now) and cells in the set
int Planner::updateGain(Q& qnew, Q& qprev){
  vector<double> qnew_state = qnew.state;
  qnew.gain_cells = qprev.gain_cells;
  SensorFootprint sf_new(qnew_state.at(0), qnew_state.at(1), qnew_state.at(2), sf_depth, sf_width);
  sf_new.computeViewedCells(qnew.gain_cells, map);
  return qnew.gain_cells.size();
}

// returns total gain up till now
int Planner::add_new(Tree& t, KDTree<DIM, vector<double>>& kd, Q& qnew, Q& qnear){
  qnew.prev_state = qnear.state;
  int gain = updateGain(qnew, qnear);
  t[qnew.state] = qnew;
  // add to kd tree
  Point<DIM> new_kd_pt = point_from_q(qnew);
  kd.insert(new_kd_pt, qnew.state);
  return gain;
}

void Planner::print_vector(vector<double> vec){
  for(int i = 0; i < vec.size(); i++){
    printf("%.7f  ", vec.at(i));
  }
  printf("\n");
}

// find the nearest neighbor qnear of q and extend the tree t from qnear to q
int Planner::extend(Q& q, Q& qnew, int numofDOFs){
  Point<DIM> q_point = point_from_q(q);
  // TODO write custom distance function
  vector<double> qnear_state = kd_tree.kNNValue(q_point, 1);
  Q qnear = tree[qnear_state]; // look up qnear from map
  // if new_config(q, qnear, qnew) then
  // note: if trapped, new_config returns false, and gain is 0
  // i.e. gain = 0 iff no qnew is added to the tree
  int gain = 0;
  if (new_config(qnear, q, qnew, numofDOFs)){
    // add qnew to tree and kd tree
    // ensure qnew isn't in tree
    if (tree.count(qnew.state) == 0){
      gain = add_new(tree, kd_tree, qnew, qnear);
    }
    double arr1[numofDOFs];
    double arr2[numofDOFs];
    vec_to_array(qnew.state, arr1, numofDOFs);
    vec_to_array(q.state, arr2, numofDOFs);
  }
  return gain;
}

// arrLength = DIM
void Planner::vec_to_array(vector<double>& vec, double *arr, int arrLength){
  for (int i = 0; i < arrLength; i++){
    arr[i] = vec.at(i);
  }
}

vector<double> Planner::array_to_vector(double *arr, int arrLength){
  vector<double> vec;
  for (int i = 0; i < arrLength; i++){
    vec.push_back(arr[i]);
  }
  return vec;
}

bool Planner::new_config(Q& qfrom, Q& qto, Q& qnew, int numofDOFs){
  // interpolate a line in config space from qfrom to qto
  // check each point on the line until invalid
  int i,j;
  if(has_reached(qfrom.state, qto.state, numofDOFs)){
      qnew = qto;
      return true;
  }
  int x_nos = (int)(fabs(qfrom.state.at(0) - qto.state.at(0)) / X_RES);
  int y_nos = (int)(fabs(qfrom.state.at(1) - qto.state.at(1)) / Y_RES);
  // TODO wrap arround
  int t_nos = (int)(fabs(qfrom.state.at(2) - qto.state.at(2)) / THETA_RES);
  int numofsamples = MAX(MAX(x_nos, y_nos), t_nos);
  vector<double> state_frontier(numofDOFs);
  vector<double> state_frontier_prev(numofDOFs);
  for (i = 0; i < numofsamples; i++){
    for(j = 0; j < numofDOFs; j++){
      state_frontier.at(j) = qfrom.state.at(j) + ((double)(i)/(numofsamples-1))*(qto.state.at(j)- qfrom.state.at(j));
    }

    if(!isValidConfiguration(state_frontier.at(0), state_frontier.at(1)))
    {
      if (i == 1) // we're trapped
        return false;
      qnew = Q(state_frontier_prev);
      return true;
    }
    // back up the frontier
    state_frontier_prev = state_frontier;
  }    
  // we reached qto
  qnew = qto;
  return true;
}

// when ta and tb are connected, call this
void Planner::get_plan(double& newx, double& newy, double& newtheta, Q& qlast, Q& qstart){
  // find the path backward
  vector<double> key;
  Q qtmp = qlast;
  vector<vector<double>> waypoints_vec;
  std::set<vector<double>> waypoints_set;
  KDTree<DIM, vector<double>> kd_tree_new;
  Point<DIM> new_kd_pt;
  while(1){
    key = qtmp.prev_state;
    qtmp = tree[key];
    if (qtmp.state != qstart.state){
      waypoints_vec.push_back(key);
      waypoints_set.insert(key);
      new_kd_pt = point_from_q(qtmp);
      kd_tree_new.insert(new_kd_pt, key);
    } else {
      break;
    }
  }
  //printf("got half of the plan\n");
  reverse(waypoints_vec.begin(), waypoints_vec.end());
  waypoints_vec.push_back(qlast.state);
  waypoints_set.insert(qlast.state);
  new_kd_pt = point_from_q(qlast);
  kd_tree_new.insert(new_kd_pt, qlast.state);
  vector<double> newwp = waypoints_vec.at(0);
  newx = newwp.at(0);
  newy = newwp.at(1);
  newtheta = newwp.at(2);

  //  prune tree (back up best branch)
  kd_tree = kd_tree_new;
  for (Tree::const_iterator itr = tree.begin() ; itr != tree.end();){
    vector<double> state = itr->first;
    if (waypoints_set.find(state) == waypoints_set.end())
      itr = tree.erase(itr);
    else
      itr = std::next(itr);
  }

  // interpolate between x y theta and newx newy newtheta
  int total_steps = 0;
  double distance;
  int i,j;
  int p;
  int num_waypoints = waypoints_vec.size();
  int numofsamples;
  vector<vector<double>> plan_vec;
  vector<double> vec1, vec2, vec_step;
  // for each pair of waypoints
  for (p = 0; p < num_waypoints-1; p++){
    distance = 0;
    vec1 = waypoints_vec.at(p);
    vec2 = waypoints_vec.at(p+1);
    // determine how to interpolate based on greatest d_theta
    for (j = 0; j < numofDOFs; j++){
      if(distance < fabs(vec1.at(j) - vec2.at(j)))
        distance = fabs(vec1.at(j) - vec2.at(j));
    }
    numofsamples = (int)(distance/(M_PI/RES));
    for (i = 0; i < numofsamples; i++){
      vector<double> vec_step;
      for(j = 0; j < numofDOFs; j++){
        vec_step.push_back(vec1.at(j) + ((double)(i)/(numofsamples-1))*(vec2.at(j) - vec1.at(j)));
      }
      plan_vec.push_back(vec_step);
    }   
    total_steps += numofsamples; 
  }
}

