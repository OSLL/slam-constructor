#include <gtest/gtest.h>

#include <limits>
#include "../../src/core/math_utils.h"
#include "../../src/core/trigonometry_utils.h"

//----------------------------------------------------------------------------//
// Cached Trigorometry Provider Test

class CachedTrigonometryProviderTest : public ::testing::Test {
protected: // consts
  static constexpr double Acc_Trig_Err = std::numeric_limits<double>::epsilon();
protected: // methods
  void verify_cache(const CachedTrigonometryProvider &cache, double rotation,
                    double min, double max, double step) {
    for (double a = min; a < max; a += step) {
      ASSERT_NEAR(std::cos(a + rotation), cache.cos(a), Acc_Trig_Err);
      ASSERT_NEAR(std::sin(a + rotation), cache.sin(a), Acc_Trig_Err);
    }
  }
};

TEST_F(CachedTrigonometryProviderTest, sector135Step30NoRotation) {
  const double Min = deg2rad(-135), Max = deg2rad(135), Step = deg2rad(30);

  auto ctp = CachedTrigonometryProvider{};
  ctp.update(Min, Max, Step);
  verify_cache(ctp, 0, Min, Max, Step);
}

TEST_F(CachedTrigonometryProviderTest, sector120Step7Rotation3) {
  const double Min = deg2rad(-120), Max = deg2rad(120), Step = deg2rad(7),
               D_Theta = deg2rad(3);

  auto ctp = CachedTrigonometryProvider{};
  ctp.update(Min, Max, Step);
  ctp.set_base_angle(D_Theta);
  verify_cache(ctp, D_Theta, Min, Max, Step);
}

//----------------------------------------------------------------------------//

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
