#include "hough_scan_matcher.h"

#include <math.h>
#include <random>
#include <memory>
#include <vector>
#include <set>
#include <functional>
#include "extra_util_functions.h"

#ifdef GL_MODE
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif

using namespace std;
using HT = HoughTransform;

template <typename T>
struct PointMax{
  T value;
  int sigma;
  long ind;
};

template <typename T>
inline void print(ostream& ostr, const vector<T>& array) {
  for (const auto& elem : array) {
    ostr << elem << " ";
  }
  ostr << endl;
}

template <typename T>
inline void smooth_zeros(vector<T>& array) {
  auto size = array.size();
  if (size == 0)
    return;
  auto begin = 0;
  for (auto curr = 0; curr < size; curr++) {
    if (array[curr] != 0) {
      double a = begin, b = curr, A = array[begin], B = array[curr];
      double k = (B-A)/(b-a), v = (A*b-a*B)/(b-a);
      for (auto local = begin+1; local < curr; local++) {
        array[local] = k*local+v;
      }
      begin = curr;
    }
  }
}

template <typename TFloat = double, typename TArgs = int>
inline TFloat avg(const vector<TArgs>& vec) {
  TFloat summ = 0;
  for(const auto& elem : vec) {
    summ += elem;
  }
  return summ/(double)vec.size();
}

template <typename T>
inline shared_ptr<vector<double>> smooth(vector<T> input, int window) {
  long size = input.size();
  shared_ptr<vector<double>> output(new vector<double>(size));
  long z,k1,k2,hw;
  double tmp;
  if(fmod(window,2)==0) window++;
  hw=(window-1)/2;
  output->at(0)=input[0];

  for (int i = 1; i < size; i++){
      tmp = 0;
      if(i < hw){
        k1 = 0;
        k2 = 2*i;
        z = k2+1;
      }
      else if((i+hw)>(size-1)){
        k1 = i-size+i+1;
        k2 = size-1;
        z = k2-k1+1;
      }
      else{
        k1 = i-hw;
        k2 = i+hw;
        z = window;
      }

      for (int j=k1; j<=k2; j++){
        tmp += input[j];
      }
      output->at(i) = tmp/(double)z;
  }
  return output;
}

template <typename T>
using _set_ = set<PointMax<T>, function<bool(PointMax<T>,PointMax<T>)>>;
template <typename T>
using _set_sp = shared_ptr<_set_<T>>;
template <typename T>
inline _set_sp<T> find_suits(const vector<T>& array, long x0, long sigma) {

  _set_sp<T> out(new _set_<T>([](PointMax<T> p1, PointMax<T> p2) -> bool {
                                    return p2.sigma <=p1.sigma;
                                  }));
  for (long  i = x0-sigma; i < x0+sigma; i++) {
    T current_val = i >= 0 && i < (long)array.size()? array[i] : 0;
    int j = 1;
    T left_val;
    T right_val;
    do {
      left_val = i-j < 0? 0 : array[i-j];
      right_val = i+j >= (long)array.size() ? 0 : array[i+j];
      j++;
    } while (current_val < left_val && current_val < right_val && j < sigma);
    if (3 < j)
      out->insert({get<T>(array,i),j,i});
  }
  return out;
}

int HoughScanMatcher::count = 0;

bool is_in_correct_sector(const DiscretePoint2D& init,
                          double a1, double b1, double c1,
                          double a2, double b2, double c2,
                          const DiscretePoint2D& control,
                          const DiscretePoint2D& victim) {
  bool control_sign_to_line1 = std::signbit(a1*control.x + b1*control.y + c1);
  bool control_sign_to_line2 = std::signbit(a2*control.x + b2*control.y + c2);

  bool victim_sign_to_line1  = std::signbit(a1*victim.x + b1*victim.y + c1);
  bool victim_sign_to_line2  = std::signbit(a2*victim.x + b2*victim.y + c2);

  bool init_sign_to_line1 = std::signbit(a1*init.x + b1*init.y + c1);
  bool init_sign_to_line2 = std::signbit(a2*init.x + b2*init.y + c2);

  //if (init_sign_to_line1 == control_sign_to_line1 &&
  //    init_sign_to_line2 == control_sign_to_line2) {
  //  return victim_sign_to_line1 == control_sign_to_line1 &&
  //         victim_sign_to_line2 == control_sign_to_line2;
  //}
  //else {
  //  return !(victim_sign_to_line1 == control_sign_to_line1 &&
  //           victim_sign_to_line2 == control_sign_to_line2);
  //}
  return (
          init_sign_to_line1 == control_sign_to_line1 &&
          init_sign_to_line2 == control_sign_to_line2
         ) ==
         (
          victim_sign_to_line1 == control_sign_to_line1 &&
          victim_sign_to_line2 == control_sign_to_line2
         );
}

vector<DiscretePoint2D> getCells(const RobotPose &init_pose,
                                 const TransformedLaserScan &scan,
                                 const GridMap& map,
                                 int left, int right, int bot, int top,
                                 double low_bound_value) {
  if (top - bot < 0 || right - left < 0) {
    return {};
  }
  std::cout << "l: " << left << " r: " << right << " b: " << bot << " t: " << top << std::endl;
  vector<DiscretePoint2D> out;
  double a1, a2, b1, b2, c1, c2, x0, x1, x2, y0, y1, y2;
  x0 = 0, y0 = 0;
  x1 = scan.points[0].range * cos(init_pose.theta + scan.points[0].angle);
  y1 = scan.points[0].range * sin(init_pose.theta + scan.points[0].angle);
  size_t end = scan.points.size()-1;
  x2 = scan.points[end].range * cos(init_pose.theta + scan.points[end].angle);
  y2 = scan.points[end].range * sin(init_pose.theta + scan.points[end].angle);

  DiscretePoint2D p0 = map.world_to_cell(x0, y0);
  DiscretePoint2D p1 = map.world_to_cell(x1, y1);
  DiscretePoint2D p2 = map.world_to_cell(x2, y2);
  DiscretePoint2D p_init = map.world_to_cell(init_pose.x, init_pose.y);
  DiscretePoint2D p_init_dir = map.world_to_cell(x0+5*cos(init_pose.theta),
                                                 y0+5*sin(init_pose.theta));
  a1 = p1.y-p0.y;           a2 = p2.y-p0.y;
  b1 = p0.x-p1.x;           b2 = p0.x-p2.x;
  c1 = p0.y*p1.x-p1.y*p0.x; c2 = p0.y*p2.x-p2.y*p0.x;
  for (int x = left; x <= right; x++) {
    for (int y = bot; y <= top; y++) {
      if (is_in_correct_sector(p_init_dir, a1, b1, c1, a2, b2, c2,
                               {(p1.x+p2.x)/2, (p1.y+p2.y)/2}, {x,y})) {
	//std::cout << "x = " <<x+p_init.x << " y = " << y+p_init.y << std::endl;
        double curr_value = map[{x+p_init.x, y+p_init.y}];
        if (low_bound_value < curr_value) {
          out.push_back({x, y});
        }
      }
    }
  }
  return out;
}

double HoughScanMatcher::process_scan(const RobotPose &init_pose,
                                      const TransformedLaserScan &scan,
                                      const GridMap &map,
                                      RobotPoseDelta &pose_delta) {

  if(count < 1) {
    count++;
    pose_delta.x = 0;
    pose_delta.y = 0;
    pose_delta.theta = 0;
    return 0;
  }

  HoughTransform scan_HT(theta_partition, delta_rho);
  HoughTransform local_map_HT(theta_partition, delta_rho);
  int right = 0, left = 0, top = 0, bot = 0;
  DiscretePoint2D prev_pt{0,0};
  #ifdef GL_MODE
    glutSetWindow(window_scan);
    glClearColor(1.0,1.0,1.0,0.0);
    glClear (GL_COLOR_BUFFER_BIT);
    glFlush();
  #endif
  for (size_t i = 0; i < scan.points.size(); i++) {
    if (!scan.points[i].is_occupied)
      continue;

    double x = scan.points[i].range * cos(init_pose.theta+scan.points[i].angle);
    double y = scan.points[i].range * sin(init_pose.theta+scan.points[i].angle);
    DiscretePoint2D curr_pt = map.world_to_cell(x,y);
    if (curr_pt.x != prev_pt.x || curr_pt.y != prev_pt.y) {
      scan_HT.transform({(HT::DOUBLE)curr_pt.x,(HT::DOUBLE)curr_pt.y});
      if (    right < curr_pt.x) right = curr_pt.x;
      if (curr_pt.x < left)       left = curr_pt.x;
      if (      top < curr_pt.y)   top = curr_pt.y;
      if (curr_pt.y < bot)         bot = curr_pt.y;
      #ifdef GL_MODE
        printRect(curr_pt.y + 150, curr_pt.y+151,
                  curr_pt.x+150, curr_pt.x+151,0.1,0.1,0.1);
      #endif
    }
    prev_pt.x = curr_pt.x; prev_pt.y = curr_pt.y;
  }
  #ifdef GL_MODE
    glFlush();
  #endif
  auto local_map = getCells(init_pose, scan, map,
                            left, right, bot, top, min_sensity);
  #ifdef GL_MODE
    glutSetWindow(window_local_map);
    glClearColor(1.0,1.0,1.0,0.0);
    glClear (GL_COLOR_BUFFER_BIT);
    glFlush();
  #endif
  for (const auto& cell : local_map) {
    #ifdef GL_MODE
      printRect(cell.y+150,cell.y+151,cell.x+150, cell.x+151,0.0);
    #endif
    local_map_HT.transform({(HT::DOUBLE)cell.x, (HT::DOUBLE)cell.y});
  }
  #ifdef GL_MODE
    glFlush();
  #endif

  shared_ptr<HT::Array_cov> scanSpectr = scan_HT.spectrum();
  shared_ptr<HT::Array_cov> mapSpectr  = local_map_HT.spectrum();


  long cov_range = 2*max_theta_steps;
  HT::Array_cov covariance(2*cov_range);
  for (int i = -cov_range; i < cov_range; i++) {
    covariance[i+cov_range] = difference(*mapSpectr, *scanSpectr, i, 0);
  }

  #ifdef GL_MODE
    print(*scanSpectr, "curr_spectr", true, 1.0,0,0);
    print(*mapSpectr, "curr_spectr", false, 0,1.0,0);
    print(covariance,"cov", true);
  #endif
  _set_sp<HT::Cov_type> better_angle_array = find_suits(covariance, cov_range,
                                                        max_theta_steps);
  int offset = better_angle_array->size() ?
                 better_angle_array->begin()->ind - cov_range
               :
                 0;

  HT::Array_cells& rho_scan_x = scan_HT.getCells()[0];
  HT::Array_cells& rho_scan_y = scan_HT.getCells()[theta_partition/4];
  HT::Array_cells& rho_map_x  = get(local_map_HT.getCells(), offset);
  HT::Array_cells& rho_map_y  = get(local_map_HT.getCells(),
                                    offset+theta_partition/4);
  shared_ptr<vector<double>> rho_smooth_scan_x = smooth(rho_scan_x, 0);
  shared_ptr<vector<double>> rho_smooth_map_x  = smooth(rho_map_x,  0);
  shared_ptr<vector<double>> rho_smooth_scan_y = smooth(rho_scan_y, 0);
  shared_ptr<vector<double>> rho_smooth_map_y  = smooth(rho_map_y,  0);

  int cov_rho_size = 10;
  HT::Array_cov covarianceRO_x(2*cov_rho_size);
  HT::Array_cov covarianceRO_y(2*cov_rho_size);
  for(long long i = -cov_rho_size; i < cov_rho_size; i++) {
    covarianceRO_x[i+cov_rho_size] = difference(*rho_smooth_map_x,
                                                *rho_smooth_scan_x, i, 0);
    covarianceRO_y[i+cov_rho_size] = difference(*rho_smooth_map_y,
                                                *rho_smooth_scan_y, i, 0);
  }
  #ifdef GL_MODE
    print(*rho_smooth_scan_x, "rho_x", true, 1.0, 0, 0);
    print(*rho_smooth_map_x, "rho_x", false, 0, 1.0, 0);
    print(*rho_smooth_scan_y, "rho_y", true, 1.0, 0, 0);
    print(*rho_smooth_map_y, "rho_y", false, 0, 1.0, 0);
    print(covarianceRO_x,    "covRho", true, 1.0, 0, 0);
    print(covarianceRO_y,   "covRho", false, 0, 1.0, 0);
  #endif

  auto better_rho_array_x = find_suits(covarianceRO_x,
                                       cov_rho_size, cov_rho_size/2);
  auto better_rho_array_y = find_suits(covarianceRO_y,
                                       cov_rho_size, cov_rho_size/2);
  int offsetX = better_rho_array_x->size() ?
                  better_rho_array_x->begin()->ind - cov_rho_size
                :
                  0;
  int offsetY = better_rho_array_y->size() ?
                  better_rho_array_y->begin()->ind - cov_rho_size
                :
                  0;

  double init_cost = GridScanMatcher::cost_estimator()->
                               estimate_scan_cost(init_pose, scan, map, 100000);
  pose_delta.theta = offset*scan_HT.delta_theta();

  pose_delta.x = offsetX*scan_HT.delta_ro();
  pose_delta.y = offsetY*scan_HT.delta_ro();
  double sm_cost = GridScanMatcher::cost_estimator()->estimate_scan_cost
                                            (
                                              {
                                               init_pose.x,
                                               init_pose.y,
                                               init_pose.theta+pose_delta.theta
                                              },
                                              scan,
                                              map,
                                              init_cost
                                            );
  double sm_cost_full = GridScanMatcher::cost_estimator()->estimate_scan_cost
                                            (
                                              {
                                               init_pose.x+pose_delta.x,
                                               init_pose.y+pose_delta.y,
                                               init_pose.theta+pose_delta.theta
                                              },
                                              scan,
                                              map,
                                              init_cost
                                            );
  if (sm_cost > init_cost ) {
    pose_delta.theta = 0;
  }
  if (sm_cost_full > init_cost ) {
    pose_delta.x = 0;
    pose_delta.y = 0;
    }
  return std::min(sm_cost,init_cost);
}
