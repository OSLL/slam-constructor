#pragma once

#include "hough.h"
#include "../grid_scan_matcher.h"

#ifdef GL_MODE
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif

#include "extra_util_functions.h"
#include <string>
#include <memory>

class HoughScanMatcher : public GridScanMatcher {
public:
  HoughScanMatcher(std::shared_ptr<ScanCostEstimator> estimator):
                     GridScanMatcher(estimator),
                     min_sensity(0.5),
                     max_theta(0.2),
                     delta_rho(0.1),
                     theta_partition(720),
                     max_theta_steps(theta_partition*M_1_PI/2.0*max_theta) {
    #ifdef GL_MODE
      int i = 1;
      char* c = new char[3]; c[0]='~'; c[1]='/'; c[2] = '\0';
      glutInit(&i,&c);
      window_scan = create_window(300,300,"scan");
      window_local_map = create_window(300,300,"local_map");
    #endif
  }

  virtual double process_scan (const RobotPose &init_pose,
                               const TransformedLaserScan &scan,
                               const GridMap &map,
                               RobotPoseDelta &pose_delta) override;
  static int count;
private: //fields
  const double min_sensity;
  const double max_theta;
  const double delta_rho;
  const long theta_partition;
  const long max_theta_steps;

  #ifdef GL_MODE
    int window_scan;
    int window_local_map;
  #endif
  //std::shared_ptr<HoughTransform::Array_cov> prev_spectr;
  //std::shared_ptr<HoughTransform::Array_cov> prev_spectr_rho;
  //std::shared_ptr<HoughTransform> prev_HT;
};
