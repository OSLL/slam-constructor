#ifndef SLAM_CTOR_CORE_MONTE_CARLO_SCAN_MATCHER_H
#define SLAM_CTOR_CORE_MONTE_CARLO_SCAN_MATCHER_H

#include <random>
#include <memory>
#include <list>
#include <math.h>

#include "../random_utils.h"
#include "pose_enumeration_scan_matcher.h"

class GaussianPoseEnumerator : public PoseEnumerator {
protected:
  using Engine = std::mt19937;
public:
  GaussianPoseEnumerator(unsigned seed,
                         double translation_dispersion,
                         double rotation_dispersion,
                         unsigned max_dispersion_failed_attempts,
                         unsigned max_poses_nm)
    : _max_failed_attempts_per_shift{max_dispersion_failed_attempts}
    , _max_poses_nm{max_poses_nm}
    , _base_translation_dispersion{translation_dispersion}
    , _base_rotation_dispersion{rotation_dispersion}
    , _pose_shift_rv{GaussianRV1D<Engine>{0, 0},
                     GaussianRV1D<Engine>{0, 0},
                     GaussianRV1D<Engine>{0, 0}}
    , _pr_generator{seed} {
    reset();
  }

  bool has_next() const override {
    return _failed_attempts_per_shift < _max_failed_attempts_per_shift &&
           _poses_nm < _max_poses_nm;
  }

  RobotPose next(const RobotPose &prev_pose) override {
    return prev_pose + _pose_shift_rv.sample(_pr_generator);
  }

  void reset() override {
    _poses_nm = 0;
    reset_shift(_base_translation_dispersion, _base_rotation_dispersion);
  }

  void feedback(bool pose_is_acceptable) override {
    ++_poses_nm;
    if (!pose_is_acceptable) {
      ++_failed_attempts_per_shift;
    } else {
      if (_failed_attempts_per_shift <= _max_failed_attempts_per_shift / 3) {
        // we haven't failed enough times to shrink the lookup area
        return;
      }
      reset_shift(0.5);
    }
  }



private:
  void reset_shift(double new_translation_dispersion,
                   double new_rotation_dispersion) {
    _failed_attempts_per_shift = 0;
    _translation_dispersion = new_translation_dispersion;
    _rotation_dispersion = new_rotation_dispersion;
    _pose_shift_rv = {GaussianRV1D<Engine>{0, _translation_dispersion},
                      GaussianRV1D<Engine>{0, _translation_dispersion},
                      GaussianRV1D<Engine>{0, _rotation_dispersion}};
                      
  }

  void reset_shift(double dispersion_scale_factor) {
    reset_shift(_translation_dispersion * 0.5, _rotation_dispersion * 0.5);
  }

private:
  // number of poses
  unsigned _max_failed_attempts_per_shift, _max_poses_nm;
  unsigned _failed_attempts_per_shift, _poses_nm;
  // sampling params
  double _base_translation_dispersion, _base_rotation_dispersion;
  double _translation_dispersion, _rotation_dispersion;

  RobotPoseDeltaRV<Engine> _pose_shift_rv;
  Engine _pr_generator;
};

class MonteCarloScanMatcher : public PoseEnumerationScanMatcher {
public:
  // FIXME: update enumerator on set_lookup_ranges update
  MonteCarloScanMatcher(std::shared_ptr<ScanProbabilityEstimator> estimator,
                        unsigned seed,
                        double translation_dispersion,
                        double rotation_dispersion,
                        unsigned failed_attempts_per_dispersion,
                        unsigned total_attempts)
    : PoseEnumerationScanMatcher{
        estimator,
        std::make_shared<GaussianPoseEnumerator>(
          seed, translation_dispersion, rotation_dispersion,
          failed_attempts_per_dispersion, total_attempts
        )
      } {}

  double process_scan(const TransformedLaserScan &raw_scan,
                      const RobotPose &init_pose,
                      const GridMap &map,
                      RobotPoseDelta &pose_delta) override {
    auto histogram = make_mean_range_hist(raw_scan);
    
    double correlation = calc_buf_correlation(histogram);
    std::cout << "correlation " <<correlation << std::endl;
    add_scan_to_buf(histogram);
    double scan_info = calc_scan_information(raw_scan);
    bool required_scan = scan_info > 0.4;
    if(correlation > 0.9 and skipped_combo++ < 10 and !required_scan) {
      pose_delta = {0,0,0};
      std::cout << "------------------------------"<< std::endl << "scan skipped" << std::endl;
      std::cout << skipped_scans++ << std::endl;
      return -1.0;

    }
    else {
      if(required_scan)
        std::cout <<"!!!!!!!!!!!!!!!";
      skipped_combo = 0;
      std::cout << "------------------------------"<< std::endl << "scan proccesed" << std::endl;
      std::cout << total_scans++ << std::endl;
      return PoseEnumerationScanMatcher::process_scan(raw_scan, init_pose, map, pose_delta);
      
    }
  }

private:
  
using my_t = double;

  std::vector<long> make_hist(const TransformedLaserScan &scan){
std::cout << "!!" << std::endl;
    int column_amount = 30; ///////////////////////////////////////////////////// 20 <======== magic constant
    std::vector<long> histogram(column_amount + 1);
    double range_size = scan.range_max - scan.range_min;

    for (auto& p : scan.scan.points()) {
      histogram[int(column_amount * (p.range() - scan.range_min)/range_size)]++;
    }
    return histogram;
  }

  std::vector<long> make_uniform_hist(const TransformedLaserScan &scan){
    int column_amount = 200; ///////////////////////////////////////////////////// 200 <======== magic constant
    std::vector<long> histogram(column_amount + 1);
    double range_size = 60;

    for (auto& p : scan.scan.points()) {
        histogram[int((double)column_amount * (p.range())/range_size)]++;
    }
    return histogram;
  }

  std::vector<double> make_range_hist(const TransformedLaserScan &scan) {
    int column_amount = 30;
    std::vector<double> histogram(column_amount + 1, 0);
    std::vector<double> means(column_amount + 1, 0);
    std::vector<double> sq_means(column_amount + 1, 0);

    double column_part = (double) column_amount / (double) scan.scan.points().size();

    for (int i = 0; i < scan.scan.points().size(); i++) {
      if((int)((double) i * column_part) <= column_amount) {
        //std::cout << (int) ((double)i * column_part) << std::endl;
        means[ (int) ((double)i * column_part)] += scan.scan.points()[i].range();
        sq_means[ (int) ((double)i * column_part)] += scan.scan.points()[i].range() * scan.scan.points()[i].range();
      }
      //std::cout << i << " " << i / items_in_column << std::endl;
    }

    for (int i = 0; i < column_amount + 1; i++) {
      histogram[i] = sq_means[i]/ (double)column_amount - means[i] / (double)(column_amount * column_amount);
    }

    return histogram;
  }

  std::vector<double> make_mean_range_hist(const TransformedLaserScan &scan) {
    int column_amount = 30;
    std::vector<double> histogram(column_amount + 1, 0);
    std::vector<double> means(column_amount + 1, 0);
    std::vector<double> sq_means(column_amount + 1, 0);

    double column_part = (double) column_amount / (double) scan.scan.points().size();

    for (int i = 0; i < scan.scan.points().size(); i++) {
      if((int)((double) i * column_part) <= column_amount) {
        //std::cout << (int) ((double)i * column_part) << std::endl;
        means[ (int) ((double)i * column_part)] += scan.scan.points()[i].range();
      }
      //std::cout << i << " " << i / items_in_column << std::endl;
    }

    for (int i = 0; i < column_amount + 1; i++) {
      histogram[i] = means[i]/ (double)column_amount;
    }

    return histogram;
  }


  std::vector<double> make_TBM_hist(const TransformedLaserScan &scan) {
    int column_amount = 30;
    std::vector<double> histogram(column_amount + 1, 0);
    std::vector<MTBM> coherences(column_amount + 1, 0);
    double range_size = scan.range_max - scan.range_min;

    auto& points = scan.scan.points();
    double column_part = (double) column_amount / (double) points.size();

    for (int i = 0; i < points.size(); i++) {
      if((int)((double) i * column_part) <= column_amount) {
        double close_to_max = (points[i].range() - scan.range_min)/range_size;
        double close_to_min = 1 - close_to_max;
        coherences[(int)((double)i * column_part)] *= MTBM(close_to_max*0.9, close_to_min*0.9, 0.1);
      }
    }

    for (int i = 0; i < column_amount + 1; i++) {
      histogram[i] = coherences[i].a;
    }

    return histogram;
  }

  template <typename T>
  void add_scan_to_buf(const std::vector<T> &raw_scan) {
    if (scan_buffer.size() > 4) {
      scan_buffer.pop_front();
    }
    scan_buffer.push_back(raw_scan);
  }

  template <typename T>
  double calc_buf_correlation(const std::vector<T>& histogram) {
    double scan_corelation = 1;
    if(scan_buffer.size() > 1)
      for (auto it = scan_buffer.begin(); it != scan_buffer.end(); it++) {
          scan_corelation *= calc_Pearson_correlation<T>(*it, histogram);
      }
    return scan_corelation;
  }

  // https://en.wikipedia.org/wiki/Pearson_correlation_coefficient
  // https://www.geeksforgeeks.org/program-find-correlation-coefficient/
  // O(5*n) where n - amount of points in X and Y
  template <typename T>
  double calc_Pearson_correlation(const std::vector<T>& X, const std::vector<T>& Y) {
    T sum_X = 0, sum_Y = 0, sum_XY = 0;
    T sq_sum_X = 0, sq_sum_Y = 0;
    T n = X.size();

    for (int i = 0; i < n; i++) {
      sum_X    += X[i];
      sum_Y    += Y[i];
      sum_XY   += X[i] * Y[i];
      sq_sum_X += X[i] * X[i];
      sq_sum_Y += Y[i] * Y[i];
    }


    return (double)                           (n * sum_XY - sum_X*sum_Y) /
             //---------------------------------------------------------------------
               (sqrt((n * sq_sum_X - sum_X * sum_X) * (n * sq_sum_Y - sum_Y * sum_Y)));

  }

  double calc_scan_information(const TransformedLaserScan &scan) {
    auto sgn = [] (double val) {return (0.0 < val) - (val < 0.0); };
    int score = 0;
    bool point_is_in_front = false;
    bool point_is_in_left = false;
    auto& points = scan.scan.points();
    //std::cout << scan.scan.points()[1].angle() << std::endl;
    std::vector<int> score_buf = {0,0,0,0};
    for (unsigned i = 0; i < points.size() - 3; i++) {

      point_is_in_front = (-M_PI_2 < points[i].angle()) && (points[i].angle() < M_PI_2);
      point_is_in_left = points[i].angle() < 0.0;
      score += sgn(points[i].range() - points[i+3].range()) * (point_is_in_front ? 1 : -1) * (point_is_in_left ? 1 : -1);;
    }
    return (double) std::abs(score) / (double) points.size();
    if (abs(score) > 500)
      std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ";
    std::cout << score << std::endl;
  }

private:
  std::list<std::vector<my_t>> scan_buffer;
  int total_scans = 0, skipped_scans = 0;
  int skipped_combo = 0;

  struct MTBM {
    double a;
    double b;
    double ab;
    bool is_zero(double val) {
      return (val < 0.00001)  && (-0.00001 < val);
    }

    MTBM(double A = 0, double B = 0, double AB = 0) : a(A), b(B), ab(AB) {}

    MTBM operator * (MTBM x) {
      
      if (is_zero(a) && is_zero(b) && is_zero(ab))
        return x;
      MTBM res;
      double conflict = x.a * this->b + x.b * this->a;
      conflict = 1/(1 - conflict);
      res.a = (x.a * this->a + x.a * this->ab + x.ab * this->a) * conflict;
      res.b = (x.b * this->b + x.b * this->ab + x.ab * this->b) * conflict;
      res.ab = (x.ab * this->ab) * conflict;
      return res;
    }

    MTBM operator *= (MTBM x) {
      *this = *this * x;
      return *this; 
    }
  };

};

#endif

