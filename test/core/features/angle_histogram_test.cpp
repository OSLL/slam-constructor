#include <gtest/gtest.h>
#include <cmath>

#include "../../../src/core/states/sensor_data.h"
#include "../../../src/core/features/angle_histogram.h"

class AHAngleEstimationTest : public ::testing::Test {
protected:
  void test_angle_estimation(const Point2D &p1, const Point2D &p2, double exp) {
    auto sp1 = ScanPoint2D::make_cartesian({p1.x, p1.y}, false);
    auto sp2 = ScanPoint2D::make_cartesian({p2.x, p2.y}, false);
    verify_angle(exp, AngleHistogram::estimate_ox_based_angle(sp1, sp2));
  }

  void verify_angle(double expected_deg, double actual_rad) {
    ASSERT_NEAR(expected_deg, rad2deg(actual_rad), 0.1);
  }
};

TEST_F(AHAngleEstimationTest, angle0) {
  test_angle_estimation({5, 3}, {7, 3}, 0);
}

TEST_F(AHAngleEstimationTest, angle30) {
  test_angle_estimation({5, 3}, {6, 3 + std::tan(deg2rad(30))}, 30);
}

TEST_F(AHAngleEstimationTest, angle45) {
  test_angle_estimation({5, 3}, {15, 13}, 45);
}

TEST_F(AHAngleEstimationTest, angle90) {
  test_angle_estimation({5, 3}, {5, 10}, 90);
}

TEST_F(AHAngleEstimationTest, angle135) {
  test_angle_estimation({5, 3}, {-5, 13}, 135);
}

TEST_F(AHAngleEstimationTest, angle180) {
  // NB: the estimated angle must be 0 (not 180),
  //     since 180 is equivalent to 0 according to the AH logic.
  test_angle_estimation({5, 3}, {0, 3}, 0);
}

TEST_F(AHAngleEstimationTest, angle225) {
  test_angle_estimation({5, 3}, {-5, -7}, 45);
}

TEST_F(AHAngleEstimationTest, angle270) {
  test_angle_estimation({5, 3}, {5, -10}, 90);
}

TEST_F(AHAngleEstimationTest, angle315) {
  test_angle_estimation({5, 3}, {15, -7}, 135);
}

TEST_F(AHAngleEstimationTest, angle330) {
  test_angle_estimation({5, 3}, {6, 3 - std::tan(deg2rad(30))}, 150);
}

//----------------------------------------------------------------------------//

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
