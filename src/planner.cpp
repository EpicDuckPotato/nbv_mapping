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
  srand(time(0));
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

bool Planner::computeNextStep(double &newx, double &newy, double &newtheta) {
  /*
  double dtheta = M_PI/16;
  double dpos = 0.1;
  newx = x + dpos;
  newy = y + 0.9*dpos;
  newtheta = theta + dtheta;
  return;
*/
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

  // TODO: 1st iteration? Proceeding iterations?
  cout <<"start exploration " << exploration_number << endl;

  double gbest;
  Q qbest_tmp;

  qstart = Q(x, y, theta);
  qstart.prev_state.clear();
  qstart.prev_state.push_back(x);
  qstart.prev_state.push_back(y);
  qstart.prev_state.push_back(theta);
  qstart.path_length = 0;
  tree[qstart.state] = qstart;
  Point<DIM> ps = point_from_q(qstart);
  kd_tree.insert(ps, qstart.state);

  SensorFootprint sf_start(x, y, theta, sf_depth, sf_width);

  if (tree.size() > 1) {
    Q qtmp = qbest;
    vector<vector<double>> waypoints_vec;
    while (qtmp.state != qtmp.prev_state) {
      waypoints_vec.push_back(qtmp.state);
      qtmp = tree[qtmp.prev_state];
    }
    tree[qtmp.state].prev_state = qstart.state;
    waypoints_vec.push_back(qtmp.state);

    if (qtmp.state != qstart.state) {
      waypoints_vec.push_back(qstart.state);
    }

    for (int i = waypoints_vec.size() - 2; i >= 0; --i) {
      tree[waypoints_vec[i]].gain = 0;
      tree[waypoints_vec[i]].gain_cells.clear();

      double dispx = tree[waypoints_vec[i]].state[0] - tree[waypoints_vec[i]].prev_state[0];
      double dispy = tree[waypoints_vec[i]].state[1] - tree[waypoints_vec[i]].prev_state[1];
      tree[waypoints_vec[i]].path_length = tree[waypoints_vec[i + 1]].path_length + sqrt(dispx*dispx + dispy*dispy);

      updateGain(tree[waypoints_vec[i]], tree[waypoints_vec[i + 1]]);
    }
    qbest = tree[waypoints_vec[0]];
    gbest = qbest.gain;
    qbest_tmp = qbest;
  } else {
    gbest = qstart.gain;
    qbest_tmp = qstart;
  }

  double xdim, ydim;
  map.getMapDim(xdim, ydim);

  while (tree.size() < N_max || gbest == 0){
    Q qnew;
    // sample a point qrand in config space, theta is d_theta
    Q qrand = sample_new(xdim, ydim);
    double gain = extend(qrand, qnew);
    if (gain >= gbest){
      qbest_tmp = qnew;
      gbest = gain;
    }
    if (tree.size() > N_tol){
      cout << "assume area is fully mapped"<< endl;
      return true;
    }
  }

  cout << "getting plan" << endl;
  get_plan(newx, newy, newtheta, qbest_tmp);

  exploration_number += 1;
  return false;
}

bool Planner::isValidConfiguration(double x, double y) const {
  if (map.inMap(x, y)){
    if (map.getStatus(map.getMapIdx(x, y)) == FREE){
      return true;
    }
  }
  return false;
}

bool Planner::isInRange(vector<double>& new_state, vector<double>& state) {
  double result = 0.0;
  for (int i = 0; i < DIM-1; i++)
      result += (new_state.at(i) - state.at(i)) * (new_state.at(i) - state.at(i));
  return result <= EPS;
}

// the theta of the state is d_theta
Q Planner::sample_new(double xdim, double ydim){
  vector<double> state;
  // x
  state.push_back(xdim * ((double)rand() / RAND_MAX));
  // y
  state.push_back(ydim * ((double)rand() / RAND_MAX));
  // theta
  state.push_back(-DTHETA_MAX + DTHETA_MAX*2 * ((double)rand() / RAND_MAX));
  
  return Q(state);
}

// the theta of the state is d_theta
Q Planner::sample_new(double left, double right, double bottom, double top){
  vector<double> state;
  // x
  state.push_back(left + (right-left) * ((double)rand() / RAND_MAX));
  // y
  state.push_back(bottom + (top-bottom) * ((double)rand() / RAND_MAX));
  // theta
  state.push_back(-DTHETA_MAX + DTHETA_MAX*2 * ((double)rand() / RAND_MAX));
  
  return Q(state);
}


Point<DIM> Planner::point_from_q(Q& q){
  Point<DIM> q_point;
  vector<double> q_state = q.state;
  copy(q_state.begin(), q_state.end(), q_point.begin());
  return q_point;
}

// compute gain (TOTAL # mapped cells up till now) and 
// and add ONLY the newly viewed cells into the gain_cell set
double Planner::updateGain(Q& qnew, Q& qprev){
  SensorFootprint sf_new(qnew.state[0], qnew.state[1], qnew.state[2], sf_depth, sf_width);

  unordered_set<int> visible_cells;
  sf_new.computeVisibleCells(visible_cells, map);
  for (unordered_set<int>::iterator it = visible_cells.begin();
       it != visible_cells.end(); ++it) {
    if (map.getStatus(*it) == UNMAPPED && !check_if_viewed(*it, qnew, qstart, tree)) {
      qnew.gain_cells.insert(*it);
    }
  }
  qnew.gain = qnew.gain_cells.size()*exp(-0.05*qnew.path_length) + qprev.gain;
  return qnew.gain;
}

// returns total gain up till now
double Planner::add_new(Tree& t, KDTree<DIM, vector<double>>& kd, Q& qnew, Q& qnear){
  qnew.prev_state = qnear.state;
  t[qnew.state] = qnew;

  double dispx = qnew.state[0] - qnew.prev_state[0];
  double dispy = qnew.state[1] - qnew.prev_state[1];
  tree[qnew.state].path_length = qnear.path_length + sqrt(dispx*dispx + dispy*dispy);

  double gain = updateGain(tree[qnew.state], qnear);
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

void Planner::print_tree(){
  printf("tree: \n");
  for(auto it = tree.begin(); it != tree.end(); ++it){
    print_vector(it->first);
    print_vector(it->second.state);
    print_vector(it->second.prev_state);
    printf("\n");
  }
  //printf("\n");
}




// find the nearest neighbor qnear of q and extend the tree t from qnear to q
double Planner::extend(Q& q, Q& qnew){
  Point<DIM> q_point = point_from_q(q);
  // TODO write custom distance function
  vector<double> qnear_state = kd_tree.kNNValue(q_point, 1);
  Q qnear = tree[qnear_state]; // look up qnear from map

  q.state.at(2) += qnear.state.at(2); // recall the theta in q was d_theta
  q.state.at(2) = process_angle(q.state.at(2));
  // if new_config(q, qnear, qnew) then
  // note: if trapped, new_config returns false, and gain is 0
  // i.e. gain = 0 iff qnew is NOT added to the tree
  double gain = 0;
  if (new_config(qnear, q, qnew)){
    if (tree.count(qnew.state) == 0){
      gain = add_new(tree, kd_tree, qnew, qnear);
    }
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

void Planner::wrap_around_theta(double& theta1, double& theta2){
  if (theta1 - theta2 > M_PI){
    theta1 -= 2*M_PI;
  }
  else if(theta2 - theta1 > M_PI){
    theta2 -= 2*M_PI;
  }
}

bool Planner::new_config(Q& qfrom_, Q& qto_, Q& qnew){
  // interpolate a line in config space from qfrom to qto
  // check each point on the line until invalid
  //cout << "new_config..." << endl;
  Q qfrom = qfrom_;
  Q qto = qto_;
  int i,j;
  int x_nos = (int)(fabs(qfrom.state.at(0) - qto.state.at(0)) / X_RES);
  int y_nos = (int)(fabs(qfrom.state.at(1) - qto.state.at(1)) / Y_RES);
  // wrap arround
  double from_theta = qfrom.state.at(2);
  double to_theta = qto.state.at(2);
  //wrap_around_theta(from_theta, to_theta);
  //qfrom.state.at(2) = from_theta;
  //qto.state.at(2) = to_theta;
  //int t_nos = (int)(fabs(from_theta - to_theta) / THETA_RES);
  //int numofsamples = MAX(MAX(x_nos, y_nos), t_nos) + 1;
  int numofsamples = MAX(x_nos, y_nos) + 1;
  if (numofsamples < 2) // qto is essentially qfrom
    return false;
  vector<double> state_frontier(DIM);
  vector<double> state_frontier_prev(DIM);
  //cout << state_frontier.size()<< " " << state_frontier_prev.size() << endl;
  // numofsamples >= 2
  for (i = 0; i < numofsamples; i++){
    for(j = 0; j < DIM-1; j++){
      state_frontier.at(j) = qfrom.state.at(j) + (double)(i)*((qto.state.at(j)- qfrom.state.at(j))/(double)(numofsamples-1));
    }

    if(!isValidConfiguration(state_frontier.at(0), state_frontier.at(1)) || !isInRange(state_frontier, qfrom.state))
    {
      if (i == 1) // we're trapped
        return false;
      state_frontier_prev.at(2) = to_theta;
      qnew = Q(state_frontier_prev);
      return true;
    }
    // back up the frontier
    state_frontier_prev = state_frontier;
  }    
  // we reached qto
  qnew = qto_;
  return true;
}


void Planner::get_plan(double &newx, double &newy, double &newtheta, Q& qlast){
  // find the path backward
  Q qtmp = qlast;
  vector<vector<double>> waypoints_vec;
  KDTree<DIM, vector<double>> kd_tree_new;
  Point<DIM> new_kd_pt;
  while (qtmp.state != qstart.state) {
    new_kd_pt = point_from_q(qtmp);
    kd_tree_new.insert(new_kd_pt, qtmp.state);
    waypoints_vec.push_back(qtmp.state);
    qtmp = tree[qtmp.prev_state];
  }

  reverse(waypoints_vec.begin(), waypoints_vec.end());

  newx = waypoints_vec[0][0];
  newy = waypoints_vec[0][1];
  newtheta = waypoints_vec[0][2];

  // Update tree and kdtree (back up best branch)
  kd_tree = kd_tree_new;

  // if a tree (and kd tree) is qstart->q1->q2->qlast,
  // then after the backup, the tree (and kd tree) is q1->q2->qlast
  // also update qstart to q1, with an updated set of gain_cells
  Tree next_tree;
  for (int i = 0; i < waypoints_vec.size(); i++){
    vector<double> key = waypoints_vec.at(i);
    next_tree[key] = tree[key];
    if (i == 0) {
      next_tree[key].prev_state = next_tree[key].state;
    }
  }

  prev_tree = tree;
  tree = next_tree;
  qbest = qlast;
}

// processed angles are in [0, 2*PI)
double Planner::process_angle(double angle){
  double angle_out = angle;
  if (angle >= 2*M_PI)
    angle_out -= 2*M_PI;
  else if (angle < 0)
    angle_out += 2*M_PI;
  return angle_out;
}


const Tree &Planner::getTree() {
  return tree;
}

const Tree &Planner::getPrevTree() {
  return prev_tree;
}
