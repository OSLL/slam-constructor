#include <gtest/gtest.h>

#include <limits>

#include "../../src/core/geometry_utils.h"

//----------------------------------------------------------------------------//
// Trigonometric Cache Test

class TrigonometricCacheTest : public ::testing::Test {
protected: // consts
  static constexpr double Acc_Trig_Err = std::numeric_limits<double>::epsilon();
protected: // methods
  void verify_cache(const TrigonometricCache &cache, double rotation,
                    double min, double max, double step) {
    for (double a = min; a < max; a += step) {
      ASSERT_NEAR(std::cos(a + rotation), cache.cos(a), Acc_Trig_Err);
      ASSERT_NEAR(std::sin(a + rotation), cache.sin(a), Acc_Trig_Err);
    }
  }
};

TEST_F(TrigonometricCacheTest, sector135Step30NoRotation) {
  const double Min = deg2rad(-135), Max = deg2rad(135), Step = deg2rad(30);

  auto cache = TrigonometricCache{};
  cache.update(Min, Max, Step);
  verify_cache(cache, 0, Min, Max, Step);
}

TEST_F(TrigonometricCacheTest, sector120Step7Rotation3) {
  const double Min = deg2rad(-120), Max = deg2rad(120), Step = deg2rad(7),
               D_Theta = deg2rad(3);

  auto cache = TrigonometricCache{};
  cache.update(Min, Max, Step);
  cache.set_theta(D_Theta);
  verify_cache(cache, D_Theta, Min, Max, Step);
}

//----------------------------------------------------------------------------//

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
