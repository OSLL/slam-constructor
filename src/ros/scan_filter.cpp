
#include <fstream>
#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int64.h>
#include <boost/shared_ptr.hpp>

class Scan_filter {
private:
  using my_t = double;


  std::list<std::vector<my_t>> scan_buffer;
  int total_scans = 0, skipped_scans = 0;
  int skipped_combo = 0;

  struct LaserScan_info {
    struct Point {
      double range;
      double angle;
      Point (double r, double a) : range(r), angle(a) {}
    };

    double range_min, range_max;
    std::vector<Point> points;

    LaserScan_info(const sensor_msgs::LaserScan& ls) {
      int size = int((ls.angle_max - ls.angle_min)/ls.angle_increment + 0.5);
      points.reserve(size);
      double range_min = ls.range_max + 1.0;
      double range_max = -1.0;
      for (int i = 0; i < size; i++) {
        if (ls.ranges[i] >= ls.range_max || ls.ranges[i] < ls.range_min)
          continue;

        points.push_back(Point(ls.ranges[i], ls.angle_min + i * ls.angle_increment));
        if (range_max < ls.ranges[i])
          range_max =  ls.ranges[i];

        if (ls.ranges[i] < range_min)
          range_min = ls.ranges[i];
      }
    }

  };

public:
  Scan_filter(){}

  bool useful_scan(const sensor_msgs::LaserScan &raw_scan) {
    LaserScan_info my_scan(raw_scan);
    

    auto t1 = std::chrono::high_resolution_clock::now();

    auto histogram = make_mean_range_hist(my_scan);
    
    double correlation = calc_buf_correlation(histogram);

    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "TIME FILTER: " << std::chrono::duration_cast<std::chrono::nanoseconds> (t2 - t1).count() << std::endl;
    std::cout << "correlation " <<correlation << std::endl;
    add_scan_to_buf(histogram);
    double scan_info = calc_scan_information(my_scan);
    bool required_scan = scan_info > 0.4;
    if(correlation > 0.9 and skipped_combo++ < 10 and !required_scan) {
      std::cout << "------------------------------"<< std::endl << "scan skipped" << std::endl;
      std::cout << skipped_scans++ << std::endl;
      return false;

    }
    else {
      if(required_scan)
        std::cout <<"!!!!!!!!!!!!!!!";
      skipped_combo = 0;
      std::cout << "------------------------------"<< std::endl << "scan proccesed" << std::endl;
      std::cout << total_scans++ << std::endl;
      return true;
    }
  }

private:
  std::vector<long> make_hist(const LaserScan_info &scan){
    int column_amount = 20; ///////////////////////////////////////////////////// 20 <======== magic constant
    std::vector<long> histogram(column_amount + 1);
    double range_size = scan.range_max - scan.range_min;

    for (auto& p : scan.points) {
      histogram[int(column_amount * (p.range - scan.range_min)/range_size)]++;
    }
    return histogram;
  }

  std::vector<long> make_uniform_hist(const LaserScan_info &scan){
    int column_amount = 200; ///////////////////////////////////////////////////// 200 <======== magic constant
    std::vector<long> histogram(column_amount + 1);
    double range_size = 60;

    for (auto& p : scan.points) {
        histogram[int((double)column_amount * (p.range)/range_size)]++;
    }
    return histogram;
  }

  std::vector<double> make_range_hist(const LaserScan_info &scan) {
    int column_amount = 30;
    std::vector<double> histogram(column_amount + 1, 0);
    std::vector<double> means(column_amount + 1, 0);
    std::vector<double> sq_means(column_amount + 1, 0);

    double column_part = (double) column_amount / (double) scan.points.size();

    for (unsigned i = 0; i < scan.points.size(); i++) {
      if((int)((double) i * column_part) <= column_amount) {
        means[ (int) ((double)i * column_part)] += scan.points[i].range;
        sq_means[ (int) ((double)i * column_part)] += scan.points[i].range * scan.points[i].range;
      }
    }

    for (int i = 0; i < column_amount + 1; i++) {
      histogram[i] = sq_means[i]/ (double)column_amount - means[i] / (double)(column_amount * column_amount);
    }

    return histogram;
  }

  std::vector<double> make_mean_range_hist(const LaserScan_info &scan) {
    int column_amount = 30;
    std::vector<double> histogram(column_amount + 1, 0);
    std::vector<double> means(column_amount + 1, 0);
    std::vector<double> sq_means(column_amount + 1, 0);

    double column_part = (double) column_amount / (double) scan.points.size();

    for (unsigned i = 0; i < scan.points.size(); i++) {
      if((int)((double) i * column_part) <= column_amount) {
        means[ (int) ((double)i * column_part)] += scan.points[i].range;
      }
    }

    for (int i = 0; i < column_amount + 1; i++) {
      histogram[i] = means[i]/ (double)column_amount;
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

  double calc_scan_information(const LaserScan_info &scan) {
    auto sgn = [] (double val) {return (0.0 < val) - (val < 0.0); };
    int score = 0;
    bool point_is_in_front = false;
    bool point_is_in_left = false;
    auto& points = scan.points;
    //std::cout << scan.scan.points()[1].angle() << std::endl;
    std::vector<int> score_buf = {0,0,0,0};
    for (unsigned i = 0; i < points.size() - 1; i++) {
      if (points[i].angle < -M_PI_2)
        score_buf[0] += sgn(points[i].range - points[i+3].range);
      if((-M_PI_2 < points[i].angle) && (points[i].angle < 0))
        score_buf[1] += sgn(points[i].range - points[i+3].range);
      if((0 < points[i].angle) && (points[i].angle <M_PI_2))
        score_buf[2] += sgn(points[i].range - points[i+3].range);
      if(M_PI_2 < points[i].angle)
        score_buf[3] += sgn(points[i].range - points[i+3].range);

      point_is_in_front = (-M_PI_2 < points[i].angle) && (points[i].angle < M_PI_2);
      point_is_in_left = points[i].angle < 0.0;
      score += sgn(points[i].range - points[i+1].range) * (point_is_in_front ? 1 : -1) * (point_is_in_left ? 1 : -1);
    }

    std::cout << "quadrics: " << score_buf[0] << " " << score_buf[1] << " " << score_buf[2] << " " << score_buf[3] << std::endl << 
                 "score: " << score << std::endl;
    return (double) std::abs(score) / (double) points.size();
    if (abs(score) > 500)
      std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ";
    std::cout << score << std::endl;
  }
};


class ROS_filter {
private:
  ros::Publisher p;
  ros::Subscriber s;
  Scan_filter sf;
  ros::NodeHandle nh;
  
public:
  ROS_filter(std::string in_topic, std::string out_topic) {
    p = nh.advertise<sensor_msgs::LaserScan>(out_topic, 1000);
    s = nh.subscribe(in_topic, 1000, &ROS_filter::handle_scan, this);
  }

  void handle_scan(const sensor_msgs::LaserScan& msg) {
    if (sf.useful_scan(msg))
      p.publish(msg);
  }
};



int main(int argc, char** argv) {
  ros::init(argc, argv, "scan_filter");

  ROS_filter r("/base_scan", "/scan");
  ros::spin();

}