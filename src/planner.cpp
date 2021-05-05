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
  cout << x << ", " << y << ", " << theta<< endl;
  srand(time(0));
  int g_best;
  Q qbest;
  if (exploration_number == 0) {
    qstart = Q(x, y, theta);
    tree[qstart.state] = qstart;
    Point<DIM> ps = point_from_q(qstart);
    kd_tree.insert(ps, qstart.state);
  } else {
    // we always prepared new qstart at then end of last run
    // and the new qstart is already in the new tree and kdtree
    // and the qstart carries the whole gain_cells of its previous branch
  }
  SensorFootprint sf_start(x, y, theta, sf_depth, sf_width);
  // update qstart.gain_cells and qstart.gain
  sf_start.computeViewedCells(qstart, qstart, tree, map);
  tree[qstart.state] = qstart; // make sure the tree is up to date
  double xdim, ydim;
  map.getMapDim(xdim, ydim);
  if (exploration_number == 0){
    g_best = qstart.gain;
    qbest = qstart;
  }
  else{
    g_best = q_best.gain;
    qbest = q_best;
  }
  //cout << "done initialization" << endl;
  // for k in 1 to K:
  while (counter < N_max || g_best == 0){
    Q qnew;
    // sample a point qrand in config space, theta is d_theta
    Q qrand = sample_new(xdim, ydim);
    int gain;
    gain = extend(qrand, qnew);
    if (gain == 0) // we're trapped
      continue;
    cout << "counter = " << counter << endl;
    counter += 1;
    print_vector(qnew.state);
    if (gain > g_best){
      qbest = qnew;
      g_best = gain;
    }
    if (counter > N_tol)
      break;
  }
  // back up best branch
  // and get first step in the best branch
  print_vector(qbest.state);
  cout << qbest.gain << endl;

  counter = 0;

  // we're stuck if qbest = qstart
  // if the new tree size has been 1 for too long, we move randomely
  // until the new tree size isn't 1
  Q qstart_old = qstart;
  get_plan(newx, newy, newtheta, qbest);
  if (tree.size() <= 1) {
    stuck_time += 1;
  } else{
    stuck_time = 0;
  }
  if (stuck_time > MAX_STUCK_TIME){
    cout << "stuck!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
    get_random_move(newx, newy, newtheta, qstart_old, xdim, ydim);
  }

  cout << "done" << endl;
  exploration_number += 1;
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
int Planner::updateGain(Q& qnew, Q& qprev){
  vector<double> qnew_state = qnew.state;
  SensorFootprint sf_new(qnew_state.at(0), qnew_state.at(1), qnew_state.at(2), sf_depth, sf_width);
  sf_new.computeViewedCells(qnew, qstart, tree, map);
  return qnew.gain;
}

// returns total gain up till now
int Planner::add_new(Tree& t, KDTree<DIM, vector<double>>& kd, Q& qnew, Q& qnear){
  //cout << "add_new..." << endl;
  qnew.prev_state = qnear.state;
  t[qnew.state] = qnew;
  int gain = updateGain(qnew, qnear);
  // the reason i'm doing this again is i'm afraid that
  // after updateGain modifie qnew, the thing int the tree is outdated
  t[qnew.state] = qnew;
  // add to kd tree
  Point<DIM> new_kd_pt = point_from_q(qnew);
  kd.insert(new_kd_pt, qnew.state);
  //cout << "done add_new" << endl;
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
int Planner::extend(Q& q, Q& qnew){
  //cout << "extend..." << endl;
  //cout << q.state.size() << endl;
  Point<DIM> q_point = point_from_q(q);
  // TODO write custom distance function
  vector<double> qnear_state = kd_tree.kNNValue(q_point, 1);
  Q qnear = tree[qnear_state]; // look up qnear from map
  if (qnear.state.size() < 3) {
    cout << qnear.state.size() << endl;
    print_vector(q.state);
    print_vector(qnear_state);
    print_tree();
  }
  q.state.at(2) += qnear.state.at(2); // recall the theta in q was d_theta
  q.state.at(2) = process_angle(q.state.at(2));
  // if new_config(q, qnear, qnew) then
  // note: if trapped, new_config returns false, and gain is 0
  // i.e. gain = 0 iff qnew is NOT added to the tree
  //print_vector(q.state);
  //print_vector(qnear_state);
  //cout << "ok" << endl;
  int gain = 0;
  if (new_config(qnear, q, qnew)){
    //cout << "qnew added" << endl;
    // add qnew to tree and kd tree
    // ensure qnew isn't in tree
    //cout << qnew.state.size() << endl;
    if (tree.count(qnew.state) == 0){
      gain = add_new(tree, kd_tree, qnew, qnear);
    }
  } else {
    //cout << "qnew NOT added" << endl;
  }
  //cout << "done extend" << endl;
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
  cout << "tree size: " << tree.size() << endl;
  vector<double> key;
  Q qtmp = qlast;
  vector<vector<double>> waypoints_vec;
  std::set<vector<double>> waypoints_set;
  KDTree<DIM, vector<double>> kd_tree_new;
  Point<DIM> new_kd_pt;
  if (qlast.state != qstart.state){
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
  }
  reverse(waypoints_vec.begin(), waypoints_vec.end());
  waypoints_vec.push_back(qlast.state);
  waypoints_set.insert(qlast.state);
  new_kd_pt = point_from_q(qlast);
  kd_tree_new.insert(new_kd_pt, qlast.state);

  if (qlast.state == qstart.state){
    newx = qlast.state.at(0);
    newy = qlast.state.at(1);
    newtheta = qlast.state.at(2);
  } else {
    vector<double> newwp = waypoints_vec.at(0);
    newx = newwp.at(0);
    newy = newwp.at(1);
    newtheta = newwp.at(2);
    cout << "waypoint sent to planner node: " << endl;
    print_vector(newwp);
  }

  // Update tree and kdtree (back up best branch)
  kd_tree = kd_tree_new;
  if (waypoints_vec.size() <= 1){
    next_tree[waypoints_vec.at(0)] = qstart;
  } else {
    // if a tree (and kd tree) is qstart->q1->q2->qlast,
    // then after the backup, the tree (and kd tree) is q1->q2->qlast
    // also update qstart to q1, with an updated set of gain_cells
    Q q1;
    for (int i = 0; i < waypoints_vec.size(); i++){
      vector<double> key = waypoints_vec.at(i);
      next_tree[key] = tree[key];
      if (i == 0){
        q1 = tree[key];
      }
    }
    Q qstart_old = qstart;
    qstart = q1;
    qstart.gain_cells = qstart_old.gain_cells;
    qstart.gain = qstart_old.gain;
    next_tree[qstart.state] = qstart;
  }
  tree = next_tree;
  next_tree.clear();
  q_best = qlast;
  
  cout << "new tree size: " << tree.size() << endl;
  /*
  cout << "new qstart: " << endl;
  print_vector(qstart.state);
  print_vector(qstart.prev_state);
  cout << "is qstart in tree?" << endl;
  Q qtest = tree[qstart.state];
  print_vector(qtest.state);
  print_vector(qtest.prev_state);
  */
}

void Planner::get_random_move(double &newx, double& newy, double& newtheta, Q& qbest, double &xdim, double &ydim){
  double tl_left, tl_right, tl_bottom, tl_top;
  double br_left, br_right, br_bottom, br_top;
  double x, y, theta;
  x = qbest.state.at(0);
  y = qbest.state.at(1);
  theta = qbest.state.at(2);
  map.getCellExtent(tl_left, tl_right, tl_bottom, tl_top, map.getMapIdx(MAX(0,x-map.getCellLength()), MIN(y+map.getCellLength(), ydim-0.1)));
  map.getCellExtent(br_left, br_right, br_bottom, br_top, map.getMapIdx(MIN(xdim-0.1, x+map.getCellLength()), MAX(0, y-map.getCellLength())));
  bool valid = false;
  Q qrand;
  while (!valid) {
    qrand = sample_new(tl_left, br_right, br_bottom, tl_top);
    if (isValidConfiguration(qrand.state.at(0), qrand.state.at(1)))
      valid = true;
  }
  qrand.state.at(2) = process_angle(qrand.state.at(2) + qbest.state.at(2));
  // back up tree and move
  qrand.gain_cells = qbest.gain_cells;
  qrand.gain = qbest.gain;
  next_tree[qrand.state] = qrand;
  KDTree<DIM, vector<double>> kd_tree_new;
  Point<DIM> new_kd_pt = point_from_q(qrand);
  kd_tree_new.insert(new_kd_pt, qrand.state);
  kd_tree = kd_tree_new;
  tree = next_tree;
  next_tree.clear();
  newx = qrand.state.at(0);
  newy = qrand.state.at(1);
  newtheta = qrand.state.at(2);
  q_best = qrand;
  qstart = qrand;
}

void Planner::get_Astar_move(double &newx, double& newy, double& newtheta, Q& qbest){
  // run multigoal a*
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

